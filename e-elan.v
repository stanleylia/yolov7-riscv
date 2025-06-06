// =============================================================================
// E-ELAN (Extended Efficient Layer Aggregation Network) 硬體加速器
// 這是 YOLO V7 的核心創新，需要專門的硬體支援
// =============================================================================

module elan_accelerator #(
    parameter integer INPUT_WIDTH = 64,
    parameter integer OUTPUT_WIDTH = 128,
    parameter integer FEATURE_CHANNELS = 256
)(
    input wire clk,
    input wire rst_n,
    
    // AXI4-Stream 輸入介面
    input wire [INPUT_WIDTH-1:0] s_axis_tdata,
    input wire s_axis_tvalid,
    output wire s_axis_tready,
    input wire s_axis_tlast,
    
    // AXI4-Stream 輸出介面
    output wire [OUTPUT_WIDTH-1:0] m_axis_tdata,
    output wire m_axis_tvalid,
    input wire m_axis_tready,
    output wire m_axis_tlast,
    
    // 控制介面
    input wire [3:0] group_size,        // 分組大小
    input wire [3:0] cardinality,       // 基數
    input wire shuffle_enable,          // 啟用特徵重組
    input wire [7:0] channel_multiplier // 通道倍數
);

    // 內部信號定義
    reg [INPUT_WIDTH-1:0] input_buffer [0:15];
    reg [3:0] buffer_write_ptr;
    reg [3:0] buffer_read_ptr;
    
    // 分組卷積單元
    wire [31:0] group_conv_0_out, group_conv_1_out, group_conv_2_out, group_conv_3_out;
    wire group_conv_valid [0:3];
    
    // 特徵重組緩衝區
    reg [31:0] shuffle_buffer [0:255];
    reg [7:0] shuffle_addr;
    
    // 合併單元
    reg [OUTPUT_WIDTH-1:0] merge_result;
    reg merge_valid;

    // 分組卷積實例化
    group_conv_3x3 group_conv_0 (
        .clk(clk),
        .rst_n(rst_n),
        .input_data(input_buffer[buffer_read_ptr]),
        .input_valid(s_axis_tvalid),
        .output_data(group_conv_0_out),
        .output_valid(group_conv_valid[0]),
        .channels(FEATURE_CHANNELS / 4)
    );
    
    group_conv_3x3 group_conv_1 (
        .clk(clk),
        .rst_n(rst_n),
        .input_data(input_buffer[buffer_read_ptr+1]),
        .input_valid(s_axis_tvalid),
        .output_data(group_conv_1_out),
        .output_valid(group_conv_valid[1]),
        .channels(FEATURE_CHANNELS / 4)
    );
    
    group_conv_3x3 group_conv_2 (
        .clk(clk),
        .rst_n(rst_n),
        .input_data(input_buffer[buffer_read_ptr+2]),
        .input_valid(s_axis_tvalid),
        .output_data(group_conv_2_out),
        .output_valid(group_conv_valid[2]),
        .channels(FEATURE_CHANNELS / 4)
    );
    
    group_conv_3x3 group_conv_3 (
        .clk(clk),
        .rst_n(rst_n),
        .input_data(input_buffer[buffer_read_ptr+3]),
        .input_valid(s_axis_tvalid),
        .output_data(group_conv_3_out),
        .output_valid(group_conv_valid[3]),
        .channels(FEATURE_CHANNELS / 4)
    );

    // 輸入緩衝區管理
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            buffer_write_ptr <= 0;
            buffer_read_ptr <= 0;
        end else begin
            if (s_axis_tvalid && s_axis_tready) begin
                input_buffer[buffer_write_ptr] <= s_axis_tdata;
                buffer_write_ptr <= buffer_write_ptr + 1;
            end
            
            if (|group_conv_valid) begin
                buffer_read_ptr <= buffer_read_ptr + 1;
            end
        end
    end

    // 特徵重組 (Shuffle) 單元
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shuffle_addr <= 0;
        end else begin
            if (shuffle_enable && (|group_conv_valid)) begin
                // 實現特徵重組邏輯
                case (group_size)
                    4'h2: begin
                        shuffle_buffer[shuffle_addr] <= {group_conv_1_out[15:0], group_conv_0_out[15:0]};
                        shuffle_buffer[shuffle_addr+1] <= {group_conv_3_out[15:0], group_conv_2_out[15:0]};
                    end
                    4'h4: begin
                        shuffle_buffer[shuffle_addr] <= {group_conv_3_out[7:0], group_conv_2_out[7:0], 
                                                        group_conv_1_out[7:0], group_conv_0_out[7:0]};
                    end
                    default: begin
                        shuffle_buffer[shuffle_addr] <= group_conv_0_out;
                    end
                endcase
                shuffle_addr <= shuffle_addr + 1;
            end
        end
    end

    // 合併單元 (Merge Cardinality)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            merge_result <= 0;
            merge_valid <= 0;
        end else begin
            if (shuffle_addr >= cardinality) begin
                // 執行基數合併
                merge_result <= {shuffle_buffer[3], shuffle_buffer[2], 
                               shuffle_buffer[1], shuffle_buffer[0]};
                merge_valid <= 1;
                shuffle_addr <= 0;
            end else begin
                merge_valid <= 0;
            end
        end
    end

    // 輸出介面
    assign m_axis_tdata = merge_result;
    assign m_axis_tvalid = merge_valid;
    assign m_axis_tlast = s_axis_tlast;
    assign s_axis_tready = !buffer_write_ptr[3]; // 當緩衝區未滿時準備接收

endmodule

// =============================================================================
// Upsample 硬體加速器 (雙線性插值)
// 用於 YOLO V7 Neck 部分的特徵圖上採樣
// =============================================================================

module upsample_accelerator #(
    parameter integer DATA_WIDTH = 24,
    parameter integer MAX_WIDTH = 1920,
    parameter integer MAX_HEIGHT = 1080
)(
    input wire clk,
    input wire rst_n,
    
    // 輸入特徵圖
    input wire [DATA_WIDTH-1:0] input_pixel,
    input wire input_valid,
    output wire input_ready,
    input wire input_sof,      // Start of Frame
    input wire input_eol,      // End of Line
    
    // 輸出特徵圖
    output reg [DATA_WIDTH-1:0] output_pixel,
    output reg output_valid,
    input wire output_ready,
    output reg output_sof,
    output reg output_eol,
    
    // 配置介面
    input wire [1:0] scale_factor,    // 00: 2x, 01: 4x, 10: 8x
    input wire [1:0] interp_method,   // 00: nearest, 01: bilinear
    input wire [15:0] input_width,
    input wire [15:0] input_height
);

    // 內部參數
    reg [15:0] output_width, output_height;
    reg [15:0] current_x, current_y;
    reg [15:0] output_x, output_y;
    
    // 雙線性插值需要的4個像素緩衝區
    reg [DATA_WIDTH-1:0] pixel_buffer [0:3];  // TL, TR, BL, BR
    reg [DATA_WIDTH-1:0] line_buffer [0:MAX_WIDTH-1];
    reg [15:0] line_buffer_addr;
    
    // 插值係數 (定點數，8bit小數部分)
    reg [15:0] weight_x, weight_y;
    reg [7:0] frac_x, frac_y;
    
    // 狀態機
    typedef enum logic [2:0] {
        IDLE,
        WAIT_INPUT,
        COMPUTE_WEIGHTS,
        INTERPOLATE,
        OUTPUT_PIXEL
    } state_t;
    
    state_t current_state, next_state;

    // 計算輸出尺寸
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            output_width <= 0;
            output_height <= 0;
        end else begin
            case (scale_factor)
                2'b00: begin  // 2x
                    output_width <= input_width << 1;
                    output_height <= input_height << 1;
                end
                2'b01: begin  // 4x
                    output_width <= input_width << 2;
                    output_height <= input_height << 2;
                end
                2'b10: begin  // 8x
                    output_width <= input_width << 3;
                    output_height <= input_height << 3;
                end
                default: begin
                    output_width <= input_width;
                    output_height <= input_height;
                end
            endcase
        end
    end

    // 狀態機
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
        end
    end

    // 狀態轉移邏輯
    always @(*) begin
        next_state = current_state;
        case (current_state)
            IDLE: begin
                if (input_valid && input_sof) begin
                    next_state = WAIT_INPUT;
                end
            end
            WAIT_INPUT: begin
                if (input_valid) begin
                    next_state = COMPUTE_WEIGHTS;
                end
            end
            COMPUTE_WEIGHTS: begin
                next_state = INTERPOLATE;
            end
            INTERPOLATE: begin
                next_state = OUTPUT_PIXEL;
            end
            OUTPUT_PIXEL: begin
                if (output_ready) begin
                    if (output_x < output_width - 1) begin
                        next_state = COMPUTE_WEIGHTS;
                    end else if (output_y < output_height - 1) begin
                        next_state = WAIT_INPUT;
                    end else begin
                        next_state = IDLE;
                    end
                end
            end
        endcase
    end

    // 座標計算
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_x <= 0;
            current_y <= 0;
            output_x <= 0;
            output_y <= 0;
        end else begin
            case (current_state)
                WAIT_INPUT: begin
                    if (input_valid) begin
                        current_x <= current_x + 1;
                        if (input_eol) begin
                            current_x <= 0;
                            current_y <= current_y + 1;
                        end
                    end
                end
                OUTPUT_PIXEL: begin
                    if (output_ready) begin
                        output_x <= output_x + 1;
                        if (output_x >= output_width - 1) begin
                            output_x <= 0;
                            output_y <= output_y + 1;
                        end
                    end
                end
            endcase
        end
    end

    // 雙線性插值權重計算
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            weight_x <= 0;
            weight_y <= 0;
            frac_x <= 0;
            frac_y <= 0;
        end else begin
            if (current_state == COMPUTE_WEIGHTS) begin
                // 計算源座標
                case (scale_factor)
                    2'b00: begin  // 2x
                        frac_x <= output_x[0] ? 8'h80 : 8'h00;
                        frac_y <= output_y[0] ? 8'h80 : 8'h00;
                    end
                    2'b01: begin  // 4x
                        frac_x <= output_x[1:0] << 6;
                        frac_y <= output_y[1:0] << 6;
                    end
                    2'b10: begin  // 8x
                        frac_x <= output_x[2:0] << 5;
                        frac_y <= output_y[2:0] << 5;
                    end
                endcase
                
                weight_x <= {8'h00, frac_x};
                weight_y <= {8'h00, frac_y};
            end
        end
    end

    // 雙線性插值計算
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            output_pixel <= 0;
            output_valid <= 0;
            output_sof <= 0;
            output_eol <= 0;
        end else begin
            case (current_state)
                INTERPOLATE: begin
                    if (interp_method == 2'b01) begin  // 雙線性插值
                        // 實現雙線性插值公式
                        // result = TL*(1-fx)*(1-fy) + TR*fx*(1-fy) + BL*(1-fx)*fy + BR*fx*fy
                        reg [31:0] tl_contrib, tr_contrib, bl_contrib, br_contrib;
                        
                        tl_contrib = pixel_buffer[0] * (256 - frac_x) * (256 - frac_y);
                        tr_contrib = pixel_buffer[1] * frac_x * (256 - frac_y);
                        bl_contrib = pixel_buffer[2] * (256 - frac_x) * frac_y;
                        br_contrib = pixel_buffer[3] * frac_x * frac_y;
                        
                        output_pixel <= (tl_contrib + tr_contrib + bl_contrib + br_contrib) >> 16;
                    end else begin  // 最近鄰插值
                        output_pixel <= pixel_buffer[0];
                    end
                    
                    output_valid <= 1;
                    output_sof <= (output_x == 0) && (output_y == 0);
                    output_eol <= (output_x >= output_width - 1);
                end
                default: begin
                    output_valid <= 0;
                    output_sof <= 0;
                    output_eol <= 0;
                end
            endcase
        end
    end

    // 線緩衝區管理
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            line_buffer_addr <= 0;
        end else begin
            if (input_valid && current_state == WAIT_INPUT) begin
                line_buffer[line_buffer_addr] <= input_pixel;
                line_buffer_addr <= line_buffer_addr + 1;
                if (input_eol) begin
                    line_buffer_addr <= 0;
                end
            end
        end
    end

    assign input_ready = (current_state == WAIT_INPUT);

endmodule

// =============================================================================
// Concat 記憶體最佳化器
// 針對 YOLO V7 中大量的特徵圖拼接操作進行最佳化
// =============================================================================

module concat_memory_optimizer #(
    parameter integer NUM_INPUTS = 4,
    parameter integer DATA_WIDTH = 64,
    parameter integer MAX_CHANNELS = 1024
)(
    input wire clk,
    input wire rst_n,
    
    // 多路輸入特徵圖
    input wire [DATA_WIDTH-1:0] input_data [0:NUM_INPUTS-1],
    input wire input_valid [0:NUM_INPUTS-1],
    output wire input_ready [0:NUM_INPUTS-1],
    input wire input_sof [0:NUM_INPUTS-1],
    input wire input_eol [0:NUM_INPUTS-1],
    
    // 輸出拼接特徵圖
    output reg [DATA_WIDTH*NUM_INPUTS-1:0] concat_output,
    output reg concat_valid,
    input wire concat_ready,
    output reg concat_sof,
    output reg concat_eol,
    
    // DMA 介面 (零拷貝最佳化)
    output reg [31:0] dma_src_addr [0:NUM_INPUTS-1],
    output reg [31:0] dma_dst_addr,
    output reg [15:0] dma_length,
    output reg dma_start,
    input wire dma_done,
    
    // 配置介面
    input wire [15:0] channels_per_input [0:NUM_INPUTS-1],
    input wire [31:0] base_addr [0:NUM_INPUTS-1],
    input wire zero_copy_enable
);

    // 內部狀態
    typedef enum logic [2:0] {
        IDLE,
        WAIT_ALL_INPUTS,
        DMA_TRANSFER,
        DIRECT_CONCAT,
        OUTPUT_READY
    } concat_state_t;
    
    concat_state_t current_state, next_state;
    
    // 輸入同步信號
    reg all_inputs_valid;
    reg all_inputs_sof;
    reg all_inputs_eol;
    
    // DMA 控制
    reg [2:0] dma_input_select;
    reg [31:0] output_offset;
    
    // 檢查所有輸入是否就緒
    integer i;
    always @(*) begin
        all_inputs_valid = 1;
        all_inputs_sof = 1;
        all_inputs_eol = 1;
        
        for (i = 0; i < NUM_INPUTS; i = i + 1) begin
            all_inputs_valid = all_inputs_valid & input_valid[i];
            all_inputs_sof = all_inputs_sof & input_sof[i];
            all_inputs_eol = all_inputs_eol & input_eol[i];
        end
    end

    // 狀態機
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
        end
    end

    // 狀態轉移邏輯
    always @(*) begin
        next_state = current_state;
        case (current_state)
            IDLE: begin
                if (all_inputs_valid) begin
                    if (zero_copy_enable) begin
                        next_state = DMA_TRANSFER;
                    end else begin
                        next_state = DIRECT_CONCAT;
                    end
                end else begin
                    next_state = WAIT_ALL_INPUTS;
                end
            end
            WAIT_ALL_INPUTS: begin
                if (all_inputs_valid) begin
                    if (zero_copy_enable) begin
                        next_state = DMA_TRANSFER;
                    end else begin
                        next_state = DIRECT_CONCAT;
                    end
                end
            end
            DMA_TRANSFER: begin
                if (dma_done && dma_input_select >= NUM_INPUTS-1) begin
                    next_state = OUTPUT_READY;
                end
            end
            DIRECT_CONCAT: begin
                next_state = OUTPUT_READY;
            end
            OUTPUT_READY: begin
                if (concat_ready) begin
                    next_state = IDLE;
                end
            end
        endcase
    end

    // DMA 控制邏輯
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dma_input_select <= 0;
            output_offset <= 0;
            dma_start <= 0;
        end else begin
            case (current_state)
                DMA_TRANSFER: begin
                    if (!dma_start) begin
                        // 設置 DMA 傳輸參數
                        dma_src_addr[dma_input_select] <= base_addr[dma_input_select];
                        dma_dst_addr <= base_addr[NUM_INPUTS] + output_offset;
                        dma_length <= channels_per_input[dma_input_select] * 64 / 8; // 轉換為bytes
                        dma_start <= 1;
                    end else if (dma_done) begin
                        dma_start <= 0;
                        output_offset <= output_offset + (channels_per_input[dma_input_select] * 64 / 8);
                        
                        if (dma_input_select < NUM_INPUTS-1) begin
                            dma_input_select <= dma_input_select + 1;
                        end else begin
                            dma_input_select <= 0;
                            output_offset <= 0;
                        end
                    end
                end
                IDLE: begin
                    dma_input_select <= 0;
                    output_offset <= 0;
                    dma_start <= 0;
                end
            endcase
        end
    end

    // 直接拼接邏輯 (非零拷貝模式)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            concat_output <= 0;
            concat_valid <= 0;
            concat_sof <= 0;
            concat_eol <= 0;
        end else begin
            case (current_state)
                DIRECT_CONCAT: begin
                    // 將所有輸入數據拼接
                    for (i = 0; i < NUM_INPUTS; i = i + 1) begin
                        concat_output[DATA_WIDTH*(i+1)-1:DATA_WIDTH*i] <= input_data[i];
                    end
                    concat_valid <= 1;
                    concat_sof <= all_inputs_sof;
                    concat_eol <= all_inputs_eol;
                end
                OUTPUT_READY: begin
                    if (zero_copy_enable) begin
                        // DMA 模式下，輸出為記憶體位址
                        concat_output <= {base_addr[NUM_INPUTS], {(DATA_WIDTH*NUM_INPUTS-32){1'b0}}};
                        concat_valid <= 1;
                    end
                    // 保持其他信號
                end
                default: begin
                    concat_valid <= 0;
                    concat_sof <= 0;
                    concat_eol <= 0;
                end
            endcase
        end
    end

    // 輸入準備信號
    genvar j;
    generate
        for (j = 0; j < NUM_INPUTS; j = j + 1) begin : gen_ready
            assign input_ready[j] = (current_state == WAIT_ALL_INPUTS) || 
                                  (current_state == IDLE);
        end
    endgenerate

endmodule

// =============================================================================
// RepConv 推理模式轉換器
// 將訓練時的 RepConv 結構轉換為推理最佳化的標準卷積
// =============================================================================

module repconv_inference_converter #(
    parameter integer DATA_WIDTH = 32,
    parameter integer KERNEL_SIZE = 3,
    parameter integer INPUT_CHANNELS = 256,
    parameter integer OUTPUT_CHANNELS = 512
)(
    input wire clk,
    input wire rst_n,
    
    // 輸入特徵圖
    input wire [DATA_WIDTH-1:0] input_data,
    input wire input_valid,
    output wire input_ready,
    
    // 輸出特徵圖
    output reg [DATA_WIDTH-1:0] output_data,
    output reg output_valid,
    input wire output_ready,
    
    // 權重載入介面
    input wire [DATA_WIDTH-1:0] weight_data,
    input wire weight_valid,
    input wire [15:0] weight_addr,
    input wire weight_load_enable,
    
    // 控制信號
    input wire repconv_enable,      // 啟用 RepConv 模式
    input wire inference_mode       // 推理模式標誌
);

    // 權重存儲
    reg [DATA_WIDTH-1:0] conv3x3_weights [0:KERNEL_SIZE*KERNEL_SIZE*INPUT_CHANNELS*OUTPUT_CHANNELS-1];
    reg [DATA_WIDTH-1:0] conv1x1_weights [0:INPUT_CHANNELS*OUTPUT_CHANNELS-1];
    reg [DATA_WIDTH-1:0] identity_weights [0:INPUT_CHANNELS-1];
    reg [DATA_WIDTH-1:0] merged_weights [0:KERNEL_SIZE*KERNEL_SIZE*INPUT_CHANNELS*OUTPUT_CHANNELS-1];
    
    // 控制信號
    reg weights_merged;
    reg conversion_done;
    
    // 卷積計算單元
    wire [DATA_WIDTH-1:0] conv_result;
    wire conv_result_valid;
    
    // 標準 3x3 卷積單元實例
    conv3x3_unit #(
        .DATA_WIDTH(DATA_WIDTH),
        .INPUT_CHANNELS(INPUT_CHANNELS),
        .OUTPUT_CHANNELS(OUTPUT_CHANNELS)
    ) conv_unit (
        .clk(clk),
        .rst_n(rst_n),
        .input_data(input_data),
        .input_valid(input_valid && inference_mode),
        .input_ready(input_ready),
        .output_data(conv_result),
        .output_valid(conv_result_valid),
        .output_ready(output_ready),
        .weights(weights_merged),
        .weight_update(conversion_done)
    );

    // 權重載入邏輯
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            weights_merged <= 0;
            conversion_done <= 0;
        end else begin
            if (weight_load_enable && weight_valid) begin
                // 根據地址範圍載入不同的權重
                if (weight_addr < KERNEL_SIZE*KERNEL_SIZE*INPUT_CHANNELS*OUTPUT_CHANNELS) begin
                    conv3x3_weights[weight_addr] <= weight_data;
                end else if (weight_addr < KERNEL_SIZE*KERNEL_SIZE*INPUT_CHANNELS*OUTPUT_CHANNELS + 
                           INPUT_CHANNELS*OUTPUT_CHANNELS) begin
                    conv1x1_weights[weight_addr - KERNEL_SIZE*KERNEL_SIZE*INPUT_CHANNELS*OUTPUT_CHANNELS] <= weight_data;
                end else begin
                    identity_weights[weight_addr - KERNEL_SIZE*KERNEL_SIZE*INPUT_CHANNELS*OUTPUT_CHANNELS - 
                                   INPUT_CHANNELS*OUTPUT_CHANNELS] <= weight_data;
                end
            end
            
            // 在推理模式下執行權重合併
            if (inference_mode && !weights_merged) begin
                merge_repconv_weights();
                weights_merged <= 1;
                conversion_done <= 1;
            end
        end
    end

    // RepConv 權重合併任務
    task merge_repconv_weights();
        integer i, j, k, l;
        reg [DATA_WIDTH-1:0] temp_weight;
        
        begin
            // 合併 3x3 卷積、1x1 卷積和恆等映射的權重
            for (i = 0; i < OUTPUT_CHANNELS; i = i + 1) begin
                for (j = 0; j < INPUT_CHANNELS; j = j + 1) begin
                    for (k = 0; k < KERNEL_SIZE; k = k + 1) begin
                        for (l = 0; l < KERNEL_SIZE; l = l + 1) begin
                            temp_weight = conv3x3_weights[i*INPUT_CHANNELS*KERNEL_SIZE*KERNEL_SIZE + 
                                                        j*KERNEL_SIZE*KERNEL_SIZE + k*KERNEL_SIZE + l];
                            
                            // 加上 1x1 卷積的貢獻 (中心位置)
                            if (k == KERNEL_SIZE/2 && l == KERNEL_SIZE/2) begin
                                temp_weight = temp_weight + conv1x1_weights[i*INPUT_CHANNELS + j];
                                
                                // 加上恆等映射的貢獻 (僅對角元素)
                                if (i == j) begin
                                    temp_weight = temp_weight + identity_weights[i];
                                end
                            end
                            
                            merged_weights[i*INPUT_CHANNELS*KERNEL_SIZE*KERNEL_SIZE + 
                                         j*KERNEL_SIZE*KERNEL_SIZE + k*KERNEL_SIZE + l] = temp_weight;
                        end
                    end
                end
            end
        end
    endtask

    // 輸出邏輯
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            output_data <= 0;
            output_valid <= 0;
        end else begin
            if (inference_mode && conversion_done) begin
                output_data <= conv_result;
                output_valid <= conv_result_valid;
            end else begin
                output_data <= 0;
                output_valid <= 0;
            end
        end
    end

endmodule

// =============================================================================
// 輔助模組：3x3 分組卷積單元
// =============================================================================

module group_conv_3x3 #(
    parameter integer DATA_WIDTH = 32,
    parameter integer CHANNELS = 64
)(
    input wire clk,
    input wire rst_n,
    input wire [DATA_WIDTH-1:0] input_data,
    input wire input_valid,
    output reg [DATA_WIDTH-1:0] output_data,
    output reg output_valid,
    input wire [7:0] channels
);

    // 卷積核緩衝區
    reg [DATA_WIDTH-1:0] kernel_buffer [0:8];
    reg [DATA_WIDTH-1:0] line_buffer [0:255];
    reg [7:0] buffer_addr;
    
    // 卷積計算
    reg [DATA_WIDTH+16-1:0] conv_result;
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            output_data <= 0;
            output_valid <= 0;
            buffer_addr <= 0;
        end else begin
            if (input_valid) begin
                // 更新線緩衝區
                line_buffer[buffer_addr] <= input_data;
                buffer_addr <= buffer_addr + 1;
                
                // 當有足夠數據時執行卷積
                if (buffer_addr >= 8) begin
                    conv_result = 0;
                    for (i = 0; i < 9; i = i + 1) begin
                        conv_result = conv_result + line_buffer[buffer_addr-8+i] * kernel_buffer[i];
                    end
                    output_data <= conv_result[DATA_WIDTH+15:16];  // 右移16位 (定點數)
                    output_valid <= 1;
                end else begin
                    output_valid <= 0;
                end
            end else begin
                output_valid <= 0;
            end
        end
    end

endmodule

// =============================================================================
// 輔助模組：標準 3x3 卷積單元
// =============================================================================

module conv3x3_unit #(
    parameter integer DATA_WIDTH = 32,
    parameter integer INPUT_CHANNELS = 256,
    parameter integer OUTPUT_CHANNELS = 512
)(
    input wire clk,
    input wire rst_n,
    input wire [DATA_WIDTH-1:0] input_data,
    input wire input_valid,
    output wire input_ready,
    output reg [DATA_WIDTH-1:0] output_data,
    output reg output_valid,
    input wire output_ready,
    input wire [DATA_WIDTH-1:0] weights [0:9*INPUT_CHANNELS*OUTPUT_CHANNELS-1],
    input wire weight_update
);

    // 內部實現標準 3x3 卷積邏輯
    // 這裡簡化實現，實際需要完整的卷積計算邏輯
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            output_data <= 0;
            output_valid <= 0;
        end else begin
            if (input_valid) begin
                // 簡化的卷積計算 (實際需要完整實現)
                output_data <= input_data;  // 簡化處理
                output_valid <= 1;
            end else begin
                output_valid <= 0;
            end
        end
    end
    
    assign input_ready = output_ready;

endmodule
