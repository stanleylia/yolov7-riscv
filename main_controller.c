/*
 * ARM A53 (Linux) - 主控制程式
 * main_controller.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <pthread.h>
#include <time.h>

// 硬體暫存器基址
#define RISC_V_BASE_ADDR    0xA0000000
#define DPU_BASE_ADDR       0xA0010000
#define VIDEO_BASE_ADDR     0xA0020000

// 共享記憶體區域
#define SHARED_MEM_BASE     0x70000000
#define SHARED_MEM_SIZE     0x10000000  // 256MB

// RISC-V 控制暫存器
#define RISCV_CMD_REG       (RISC_V_BASE_ADDR + 0x00)
#define RISCV_STATUS_REG    (RISC_V_BASE_ADDR + 0x04)
#define RISCV_DATA_ADDR_REG (RISC_V_BASE_ADDR + 0x08)
#define RISCV_IRQ_REG       (RISC_V_BASE_ADDR + 0x0C)

// DPU 控制暫存器
#define DPU_CTRL_REG        (DPU_BASE_ADDR + 0x00)
#define DPU_STATUS_REG      (DPU_BASE_ADDR + 0x04)
#define DPU_INPUT_ADDR_REG  (DPU_BASE_ADDR + 0x08)
#define DPU_OUTPUT_ADDR_REG (DPU_BASE_ADDR + 0x0C)
#define DPU_WIDTH_REG       (DPU_BASE_ADDR + 0x10)
#define DPU_HEIGHT_REG      (DPU_BASE_ADDR + 0x14)

// 命令定義
#define CMD_PROCESS_FRAME   0x01
#define CMD_UPDATE_PARAMS   0x02
#define CMD_GET_RESULTS     0x03

// 全域變數
static volatile uint32_t *hw_regs;
static void *shared_memory;
static int fd_mem;

// 系統狀態
typedef struct {
    int frame_count;
    int detection_count;
    double avg_fps;
    double avg_latency;
    struct timespec last_frame_time;
} system_stats_t;

static system_stats_t stats = {0};

// 硬體初始化
int init_hardware(void) {
    // 開啟記憶體設備
    fd_mem = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd_mem < 0) {
        perror("無法開啟 /dev/mem");
        return -1;
    }
    
    // 映射硬體暫存器
    hw_regs = mmap(NULL, 0x100000, PROT_READ | PROT_WRITE, 
                   MAP_SHARED, fd_mem, RISC_V_BASE_ADDR);
    if (hw_regs == MAP_FAILED) {
        perror("無法映射硬體暫存器");
        close(fd_mem);
        return -1;
    }
    
    // 映射共享記憶體
    shared_memory = mmap(NULL, SHARED_MEM_SIZE, PROT_READ | PROT_WRITE,
                        MAP_SHARED, fd_mem, SHARED_MEM_BASE);
    if (shared_memory == MAP_FAILED) {
        perror("無法映射共享記憶體");
        munmap((void*)hw_regs, 0x100000);
        close(fd_mem);
        return -1;
    }
    
    printf("硬體初始化完成\n");
    return 0;
}

// 向 RISC-V 發送命令
int send_to_riscv(uint32_t cmd, uint32_t data_addr, uint32_t param) {
    // 檢查 RISC-V 狀態
    uint32_t status = *(volatile uint32_t*)(hw_regs + (RISCV_STATUS_REG - RISC_V_BASE_ADDR) / 4);
    if (status & 0x1) {  // 忙碌中
        printf("RISC-V 忙碌中，等待...\n");
        return -1;
    }
    
    // 設定參數
    *(volatile uint32_t*)(hw_regs + (RISCV_DATA_ADDR_REG - RISC_V_BASE_ADDR) / 4) = data_addr;
    
    // 發送命令
    *(volatile uint32_t*)(hw_regs + (RISCV_CMD_REG - RISC_V_BASE_ADDR) / 4) = (param << 16) | cmd;
    
    // 觸發中斷
    *(volatile uint32_t*)(hw_regs + (RISCV_IRQ_REG - RISC_V_BASE_ADDR) / 4) = 0x1;
    
    return 0;
}

// 控制 DPU 推理
int run_dpu_inference(uint32_t input_addr, uint32_t output_addr, 
                     uint16_t width, uint16_t height) {
    // 設定 DPU 參數
    *(volatile uint32_t*)(hw_regs + (DPU_INPUT_ADDR_REG - RISC_V_BASE_ADDR) / 4) = input_addr;
    *(volatile uint32_t*)(hw_regs + (DPU_OUTPUT_ADDR_REG - RISC_V_BASE_ADDR) / 4) = output_addr;
    *(volatile uint32_t*)(hw_regs + (DPU_WIDTH_REG - RISC_V_BASE_ADDR) / 4) = width;
    *(volatile uint32_t*)(hw_regs + (DPU_HEIGHT_REG - RISC_V_BASE_ADDR) / 4) = height;
    
    // 啟動 DPU
    *(volatile uint32_t*)(hw_regs + (DPU_CTRL_REG - RISC_V_BASE_ADDR) / 4) = 0x1;
    
    printf("DPU 推理開始 - 輸入: 0x%08X, 輸出: 0x%08X, 尺寸: %dx%d\n", 
           input_addr, output_addr, width, height);
    
    return 0;
}

// 等待 DPU 完成
int wait_dpu_done(void) {
    int timeout = 1000; // 1秒超時
    
    while (timeout--) {
        uint32_t status = *(volatile uint32_t*)(hw_regs + (DPU_STATUS_REG - RISC_V_BASE_ADDR) / 4);
        if (status & 0x1) {  // DPU 完成
            printf("DPU 推理完成\n");
            return 0;
        }
        usleep(1000); // 等待 1ms
    }
    
    printf("DPU 推理超時\n");
    return -1;
}

// 處理單幀的主函數
int process_frame(uint8_t *frame_data, int width, int height) {
    struct timespec start_time, end_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    
    // 計算記憶體地址
    uint32_t input_addr = SHARED_MEM_BASE;
    uint32_t output_addr = SHARED_MEM_BASE + 0x1000000;  // 16MB 偏移
    uint32_t processed_addr = SHARED_MEM_BASE + 0x2000000; // 32MB 偏移
    
    // 1. 複製影像資料到共享記憶體
    memcpy(shared_memory, frame_data, width * height * 3);
    
    // 2. 啟動 DPU 推理
    if (run_dpu_inference(input_addr, output_addr, width, height) < 0) {
        return -1;
    }
    
    // 3. 等待 DPU 完成
    if (wait_dpu_done() < 0) {
        return -1;
    }
    
    // 4. 將結果傳送給 RISC-V 進行後處理
    if (send_to_riscv(CMD_PROCESS_FRAME, output_addr, (width << 16) | height) < 0) {
        return -1;
    }
    
    // 5. 計算效能統計
    clock_gettime(CLOCK_MONOTONIC, &end_time);
    double latency = (end_time.tv_sec - start_time.tv_sec) * 1000.0 +
                    (end_time.tv_nsec - start_time.tv_nsec) / 1000000.0;
    
    stats.frame_count++;
    stats.avg_latency = (stats.avg_latency * (stats.frame_count - 1) + latency) / stats.frame_count;
    
    printf("幀 %d 處理完成，延遲: %.2f ms\n", stats.frame_count, latency);
    
    return 0;
}

// 視訊處理執行緒
void* video_processing_thread(void* arg) {
    uint8_t *dummy_frame = malloc(1920 * 1080 * 3); // 示例幀資料
    
    // 模擬視訊幀
    for (int i = 0; i < 1920 * 1080 * 3; i++) {
        dummy_frame[i] = rand() % 256;
    }
    
    while (1) {
        if (process_frame(dummy_frame, 1920, 1080) < 0) {
            printf("幀處理失敗\n");
            break;
        }
        
        // 模擬 30 FPS
        usleep(33333);  // ~30ms
    }
    
    free(dummy_frame);
    return NULL;
}

// 統計監控執行緒
void* stats_monitor_thread(void* arg) {
    while (1) {
        sleep(5);  // 每 5 秒報告一次統計
        
        printf("=== 系統統計 ===\n");
        printf("已處理幀數: %d\n", stats.frame_count);
        printf("平均延遲: %.2f ms\n", stats.avg_latency);
        printf("平均 FPS: %.2f\n", stats.frame_count > 0 ? 1000.0 / stats.avg_latency : 0);
        printf("================\n");
    }
    
    return NULL;
}

int main(int argc, char *argv[]) {
    printf("Kria KV260 YOLO + RISC-V 主控制程式啟動\n");
    
    // 初始化硬體
    if (init_hardware() < 0) {
        return -1;
    }
    
    // 建立執行緒
    pthread_t video_thread, stats_thread;
    
    if (pthread_create(&video_thread, NULL, video_processing_thread, NULL) != 0) {
        perror("無法建立視訊處理執行緒");
        return -1;
    }
    
    if (pthread_create(&stats_thread, NULL, stats_monitor_thread, NULL) != 0) {
        perror("無法建立統計監控執行緒");
        return -1;
    }
    
    // 等待執行緒完成
    pthread_join(video_thread, NULL);
    pthread_join(stats_thread, NULL);
    
    // 清理資源
    munmap(shared_memory, SHARED_MEM_SIZE);
    munmap((void*)hw_regs, 0x100000);
    close(fd_mem);
    
    return 0;
}

/*
 * =============================================================================
 * RISC-V (Custom) - 後處理程式
 * riscv_postprocess.c
 * =============================================================================
 */

#include <stdint.h>

// 硬體暫存器地址 (從 RISC-V 角度)
#define CMD_REG         0x60000000
#define STATUS_REG      0x60000004
#define DATA_ADDR_REG   0x60000008
#define IRQ_CLR_REG     0x6000000C
#define RESULT_REG      0x60000010

// 共享記憶體 (透過 AXI 存取)
#define SHARED_MEM_BASE 0x70000000

// YOLO 檢測結果結構
typedef struct {
    float x, y, w, h;  // 邊界框
    float confidence;  // 信心度
    int class_id;      // 類別 ID
} detection_t;

// 全域變數
static volatile uint32_t processed_frames = 0;
static detection_t detections[100];  // 最多 100 個檢測

// 讀取硬體暫存器
static inline uint32_t read_reg(uint32_t addr) {
    return *(volatile uint32_t*)addr;
}

// 寫入硬體暫存器
static inline void write_reg(uint32_t addr, uint32_t value) {
    *(volatile uint32_t*)addr = value;
}

// 非最大值抑制 (NMS)
int apply_nms(detection_t *dets, int num_dets, float nms_threshold) {
    // 簡化的 NMS 實現
    int keep[100] = {0};
    int final_count = 0;
    
    for (int i = 0; i < num_dets; i++) {
        if (dets[i].confidence < 0.5) continue;  // 信心度過濾
        
        keep[i] = 1;
        for (int j = i + 1; j < num_dets; j++) {
            if (dets[j].confidence < 0.5) continue;
            
            // 計算 IoU
            float x1 = dets[i].x > dets[j].x ? dets[i].x : dets[j].x;
            float y1 = dets[i].y > dets[j].y ? dets[i].y : dets[j].y;
            float x2 = (dets[i].x + dets[i].w) < (dets[j].x + dets[j].w) ? 
                      (dets[i].x + dets[i].w) : (dets[j].x + dets[j].w);
            float y2 = (dets[i].y + dets[i].h) < (dets[j].y + dets[j].h) ? 
                      (dets[i].y + dets[i].h) : (dets[j].y + dets[j].h);
            
            float intersection = (x2 - x1) * (y2 - y1);
            if (intersection <= 0) continue;
            
            float area1 = dets[i].w * dets[i].h;
            float area2 = dets[j].w * dets[j].h;
            float iou = intersection / (area1 + area2 - intersection);
            
            if (iou > nms_threshold) {
                if (dets[i].confidence > dets[j].confidence) {
                    keep[j] = 0;
                } else {
                    keep[i] = 0;
                    break;
                }
            }
        }
        
        if (keep[i]) final_count++;
    }
    
    return final_count;
}

// 處理 YOLO 檢測結果
void process_yolo_results(uint32_t result_addr, uint32_t frame_info) {
    uint16_t width = (frame_info >> 16) & 0xFFFF;
    uint16_t height = frame_info & 0xFFFF;
    
    // 從共享記憶體讀取 DPU 輸出結果
    float *raw_output = (float*)(SHARED_MEM_BASE + (result_addr - SHARED_MEM_BASE));
    
    // 解析 YOLO 輸出 (簡化版本)
    int num_detections = 0;
    for (int i = 0; i < 25200; i++) {  // YOLO v7 輸出大小
        if (raw_output[i * 85 + 4] > 0.25) {  // 物件信心度閾值
            if (num_detections >= 100) break;
            
            detections[num_detections].x = raw_output[i * 85 + 0] * width;
            detections[num_detections].y = raw_output[i * 85 + 1] * height;
            detections[num_detections].w = raw_output[i * 85 + 2] * width;
            detections[num_detections].h = raw_output[i * 85 + 3] * height;
            detections[num_detections].confidence = raw_output[i * 85 + 4];
            
            // 找到最高機率的類別
            float max_class_prob = 0;
            int max_class_id = 0;
            for (int j = 5; j < 85; j++) {
                if (raw_output[i * 85 + j] > max_class_prob) {
                    max_class_prob = raw_output[i * 85 + j];
                    max_class_id = j - 5;
                }
            }
            
            detections[num_detections].confidence *= max_class_prob;
            detections[num_detections].class_id = max_class_id;
            num_detections++;
        }
    }
    
    // 應用非最大值抑制
    int final_count = apply_nms(detections, num_detections, 0.45);
    
    // 將結果寫回共享記憶體
    uint32_t *result_buffer = (uint32_t*)(SHARED_MEM_BASE + 0x3000000); // 48MB 偏移
    result_buffer[0] = final_count;
    
    for (int i = 0; i < final_count; i++) {
        if (detections[i].confidence > 0.5) {
            result_buffer[1 + i * 6 + 0] = *(uint32_t*)&detections[i].x;
            result_buffer[1 + i * 6 + 1] = *(uint32_t*)&detections[i].y;
            result_buffer[1 + i * 6 + 2] = *(uint32_t*)&detections[i].w;
            result_buffer[1 + i * 6 + 3] = *(uint32_t*)&detections[i].h;
            result_buffer[1 + i * 6 + 4] = *(uint32_t*)&detections[i].confidence;
            result_buffer[1 + i * 6 + 5] = detections[i].class_id;
        }
    }
    
    processed_frames++;
}

// 中斷處理程式
void interrupt_handler(void) {
    uint32_t cmd = read_reg(CMD_REG);
    uint32_t data_addr = read_reg(DATA_ADDR_REG);
    uint32_t param = cmd >> 16;
    cmd = cmd & 0xFFFF;
    
    switch (cmd) {
        case 0x01:  // CMD_PROCESS_FRAME
            process_yolo_results(data_addr, param);
            break;
            
        case 0x02:  // CMD_UPDATE_PARAMS
            // 更新算法參數
            break;
            
        case 0x03:  // CMD_GET_RESULTS
            // 返回處理統計
            write_reg(RESULT_REG, processed_frames);
            break;
    }
    
    // 清除中斷
    write_reg(IRQ_CLR_REG, 0x1);
    
    // 更新狀態暫存器
    write_reg(STATUS_REG, 0x0);  // 設為空閒狀態
}

// RISC-V 主程式
int main(void) {
    // 初始化狀態暫存器
    write_reg(STATUS_REG, 0x0);  // 空閒狀態
    
    // 啟用中斷
    asm volatile ("csrsi mstatus, 0x8");  // 啟用全域中斷
    asm volatile ("csrsi mie, 0x800");    // 啟用外部中斷
    
    // 主迴圈
    while (1) {
        // 等待中斷
        asm volatile ("wfi");  // Wait for Interrupt
    }
    
    return 0;
}
