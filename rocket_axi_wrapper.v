// rocket_axi_wrapper.v - RISC-V Rocket Core AXI Wrapper for KV260
// 這個模組將 Rocket Core 的 TileLink 介面轉換為 AXI4 介面

module rocket_axi_wrapper #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 32,
    parameter integer C_M_AXI_DATA_WIDTH = 64,
    parameter integer C_M_AXI_ADDR_WIDTH = 32
)(
    // 全域信號
    input wire aclk,
    input wire aresetn,
    
    // AXI4-Lite Slave Interface (控制介面)
    input wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_awaddr,
    input wire [2:0] s_axi_awprot,
    input wire s_axi_awvalid,
    output wire s_axi_awready,
    input wire [C_S_AXI_DATA_WIDTH-1:0] s_axi_wdata,
    input wire [(C_S_AXI_DATA_WIDTH/8)-1:0] s_axi_wstrb,
    input wire s_axi_wvalid,
    output wire s_axi_wready,
    output wire [1:0] s_axi_bresp,
    output wire s_axi_bvalid,
    input wire s_axi_bready,
    input wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_araddr,
    input wire [2:0] s_axi_arprot,
    input wire s_axi_arvalid,
    output wire s_axi_arready,
    output wire [C_S_AXI_DATA_WIDTH-1:0] s_axi_rdata,
    output wire [1:0] s_axi_rresp,
    output wire s_axi_rvalid,
    input wire s_axi_rready,
    
    // AXI4 Master Interface (記憶體存取)
    output wire [C_M_AXI_ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [7:0] m_axi_awlen,
    output wire [2:0] m_axi_awsize,
    output wire [1:0] m_axi_awburst,
    output wire m_axi_awlock,
    output wire [3:0] m_axi_awcache,
    output wire [2:0] m_axi_awprot,
    output wire [3:0] m_axi_awqos,
    output wire m_axi_awvalid,
    input wire m_axi_awready,
    output wire [C_M_AXI_DATA_WIDTH-1:0] m_axi_wdata,
    output wire [(C_M_AXI_DATA_WIDTH/8)-1:0] m_axi_wstrb,
    output wire m_axi_wlast,
    output wire m_axi_wvalid,
    input wire m_axi_wready,
    input wire [1:0] m_axi_bresp,
    input wire m_axi_bvalid,
    output wire m_axi_bready,
    output wire [C_M_AXI_ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [7:0] m_axi_arlen,
    output wire [2:0] m_axi_arsize,
    output wire [1:0] m_axi_arburst,
    output wire m_axi_arlock,
    output wire [3:0] m_axi_arcache,
    output wire [2:0] m_axi_arprot,
    output wire [3:0] m_axi_arqos,
    output wire m_axi_arvalid,
    input wire m_axi_arready,
    input wire [C_M_AXI_DATA_WIDTH-1:0] m_axi_rdata,
    input wire [1:0] m_axi_rresp,
    input wire m_axi_rlast,
    input wire m_axi_rvalid,
    output wire m_axi_rready,
    
    // 中斷輸出
    output wire interrupt,
    
    // GPIO 介面
    input wire [7:0] gpio_in,
    output wire [7:0] gpio_out
);

    // Rocket Core 時脈和重置
    wire rocket_clock;
    wire rocket_reset;
    
    assign rocket_clock = aclk;
    assign rocket_reset = ~aresetn;
    
    // TileLink 信號
    wire tl_a_valid, tl_a_ready;
    wire [2:0] tl_a_opcode;
    wire [2:0] tl_a_param;
    wire [2:0] tl_a_size;
    wire [4:0] tl_a_source;
    wire [31:0] tl_a_address;
    wire [7:0] tl_a_mask;
    wire [63:0] tl_a_data;
    wire tl_a_corrupt;
    
    wire tl_d_valid, tl_d_ready;
    wire [2:0] tl_d_opcode;
    wire [1:0] tl_d_param;
    wire [2:0] tl_d_size;
    wire [4:0] tl_d_source;
    wire [1:0] tl_d_sink;
    wire tl_d_denied;
    wire [63:0] tl_d_data;
    wire tl_d_corrupt;
    
    // Rocket Core 實例 (基於您原本的 Rocket Chip 設計修改)
    TestHarness rocket_core (
        .clock(rocket_clock),
        .reset(rocket_reset),
        
        // TileLink Master Port
        .mem_axi4_0_aw_valid(tl_a_valid),
        .mem_axi4_0_aw_ready(tl_a_ready),
        .mem_axi4_0_aw_bits_id(tl_a_source),
        .mem_axi4_0_aw_bits_addr(tl_a_address),
        .mem_axi4_0_aw_bits_len(8'h0), // 單次傳輸
        .mem_axi4_0_aw_bits_size(tl_a_size),
        .mem_axi4_0_aw_bits_burst(2'b01), // INCR
        
        .mem_axi4_0_w_valid(tl_a_valid),
        .mem_axi4_0_w_ready(tl_a_ready),
        .mem_axi4_0_w_bits_data(tl_a_data),
        .mem_axi4_0_w_bits_strb(tl_a_mask),
        .mem_axi4_0_w_bits_last(1'b1),
        
        .mem_axi4_0_b_valid(tl_d_valid),
        .mem_axi4_0_b_ready(tl_d_ready),
        .mem_axi4_0_b_bits_resp(2'b00),
        .mem_axi4_0_b_bits_id(tl_d_source),
        
        .mem_axi4_0_ar_valid(tl_a_valid),
        .mem_axi4_0_ar_ready(tl_a_ready),
        .mem_axi4_0_ar_bits_id(tl_a_source),
        .mem_axi4_0_ar_bits_addr(tl_a_address),
        .mem_axi4_0_ar_bits_len(8'h0),
        .mem_axi4_0_ar_bits_size(tl_a_size),
        .mem_axi4_0_ar_bits_burst(2'b01),
        
        .mem_axi4_0_r_valid(tl_d_valid),
        .mem_axi4_0_r_ready(tl_d_ready),
        .mem_axi4_0_r_bits_id(tl_d_source),
        .mem_axi4_0_r_bits_data(tl_d_data),
        .mem_axi4_0_r_bits_resp(2'b00),
        .mem_axi4_0_r_bits_last(1'b1),
        
        // MMIO 介面 (保持原本的設計)
        .uart_rx(1'b1),
        .uart_tx(),
        .led(gpio_out),
        
        // 中斷
        .io_success(interrupt)
    );
    
    // TileLink 到 AXI4 轉換器
    tilelink_to_axi4 #(
        .AXI_ADDR_WIDTH(C_M_AXI_ADDR_WIDTH),
        .AXI_DATA_WIDTH(C_M_AXI_DATA_WIDTH)
    ) tl_axi_converter (
        .clk(aclk),
        .rst(rocket_reset),
        
        // TileLink A Channel (Master to Slave)
        .tl_a_valid(tl_a_valid),
        .tl_a_ready(tl_a_ready),
        .tl_a_opcode(tl_a_opcode),
        .tl_a_param(tl_a_param),
        .tl_a_size(tl_a_size),
        .tl_a_source(tl_a_source),
        .tl_a_address(tl_a_address),
        .tl_a_mask(tl_a_mask),
        .tl_a_data(tl_a_data),
        .tl_a_corrupt(tl_a_corrupt),
        
        // TileLink D Channel (Slave to Master)
        .tl_d_valid(tl_d_valid),
        .tl_d_ready(tl_d_ready),
        .tl_d_opcode(tl_d_opcode),
        .tl_d_param(tl_d_param),
        .tl_d_size(tl_d_size),
        .tl_d_source(tl_d_source),
        .tl_d_sink(tl_d_sink),
        .tl_d_denied(tl_d_denied),
        .tl_d_data(tl_d_data),
        .tl_d_corrupt(tl_d_corrupt),
        
        // AXI4 Master Interface
        .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awlen(m_axi_awlen),
        .m_axi_awsize(m_axi_awsize),
        .m_axi_awburst(m_axi_awburst),
        .m_axi_awlock(m_axi_awlock),
        .m_axi_awcache(m_axi_awcache),
        .m_axi_awprot(m_axi_awprot),
        .m_axi_awqos(m_axi_awqos),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata),
        .m_axi_wstrb(m_axi_wstrb),
        .m_axi_wlast(m_axi_wlast),
        .m_axi_wvalid(m_axi_wvalid),
        .m_axi_wready(m_axi_wready),
        .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(m_axi_bvalid),
        .m_axi_bready(m_axi_bready),
        .m_axi_araddr(m_axi_araddr),
        .m_axi_arlen(m_axi_arlen),
        .m_axi_arsize(m_axi_arsize),
        .m_axi_arburst(m_axi_arburst),
        .m_axi_arlock(m_axi_arlock),
        .m_axi_arcache(m_axi_arcache),
        .m_axi_arprot(m_axi_arprot),
        .m_axi_arqos(m_axi_arqos),
        .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready),
        .m_axi_rdata(m_axi_rdata),
        .m_axi_rresp(m_axi_rresp),
        .m_axi_rlast(m_axi_rlast),
        .m_axi_rvalid(m_axi_rvalid),
        .m_axi_rready(m_axi_rready)
    );
    
    // AXI4-Lite Slave 控制介面
    axi_lite_slave #(
        .C_S_AXI_DATA_WIDTH(C_S_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH(C_S_AXI_ADDR_WIDTH)
    ) axi_lite_ctrl (
        .S_AXI_ACLK(aclk),
        .S_AXI_ARESETN(aresetn),
        .S_AXI_AWADDR(s_axi_awaddr),
        .S_AXI_AWPROT(s_axi_awprot),
        .S_AXI_AWVALID(s_axi_awvalid),
        .S_AXI_AWREADY(s_axi_awready),
        .S_AXI_WDATA(s_axi_wdata),
        .S_AXI_WSTRB(s_axi_wstrb),
        .S_AXI_WVALID(s_axi_wvalid),
        .S_AXI_WREADY(s_axi_wready),
        .S_AXI_BRESP(s_axi_bresp),
        .S_AXI_BVALID(s_axi_bvalid),
        .S_AXI_BREADY(s_axi_bready),
        .S_AXI_ARADDR(s_axi_araddr),
        .S_AXI_ARPROT(s_axi_arprot),
        .S_AXI_ARVALID(s_axi_arvalid),
        .S_AXI_ARREADY(s_axi_arready),
        .S_AXI_RDATA(s_axi_rdata),
        .S_AXI_RRESP(s_axi_rresp),
        .S_AXI_RVALID(s_axi_rvalid),
        .S_AXI_RREADY(s_axi_rready),
        
        // GPIO 控制
        .gpio_in(gpio_in),
        .gpio_out(gpio_out)
    );

endmodule

// =============================================================================
// YOLO 加速器控制模組
// =============================================================================

module yolo_accelerator_ctrl #(
    parameter integer C_S_AXI_DATA_WIDTH = 32,
    parameter integer C_S_AXI_ADDR_WIDTH = 12
)(
    input wire aclk,
    input wire aresetn,
    
    // AXI4-Lite Slave Interface
    input wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_awaddr,
    input wire [2:0] s_axi_awprot,
    input wire s_axi_awvalid,
    output reg s_axi_awready,
    input wire [C_S_AXI_DATA_WIDTH-1:0] s_axi_wdata,
    input wire [(C_S_AXI_DATA_WIDTH/8)-1:0] s_axi_wstrb,
    input wire s_axi_wvalid,
    output reg s_axi_wready,
    output reg [1:0] s_axi_bresp,
    output reg s_axi_bvalid,
    input wire s_axi_bready,
    input wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_araddr,
    input wire [2:0] s_axi_arprot,
    input wire s_axi_arvalid,
    output reg s_axi_arready,
    output reg [C_S_AXI_DATA_WIDTH-1:0] s_axi_rdata,
    output reg [1:0] s_axi_rresp,
    output reg s_axi_rvalid,
    input wire s_axi_rready,
    
    // DPU 控制介面
    output reg dpu_start,
    input wire dpu_done,
    input wire dpu_idle,
    output reg [31:0] dpu_input_addr,
    output reg [31:0] dpu_output_addr,
    output reg [15:0] dpu_input_width,
    output reg [15:0] dpu_input_height,
    
    // 中斷輸出
    output reg interrupt
);

    // 暫存器定義
    localparam [C_S_AXI_ADDR_WIDTH-1:0] 
        REG_CONTROL     = 12'h000,
        REG_STATUS      = 12'h004,
        REG_INPUT_ADDR  = 12'h008,
        REG_OUTPUT_ADDR = 12'h00C,
        REG_INPUT_WIDTH = 12'h010,
        REG_INPUT_HEIGHT= 12'h014,
        REG_INTERRUPT   = 12'h018;
    
    reg [31:0] reg_control;
    reg [31:0] reg_status;
    reg [31:0] reg_input_addr;
    reg [31:0] reg_output_addr;
    reg [15:0] reg_input_width;
    reg [15:0] reg_input_height;
    reg [31:0] reg_interrupt;
    
    // AXI 信號處理
    reg axi_awready_reg;
    reg axi_wready_reg;
    reg axi_bvalid_reg;
    reg axi_arready_reg;
    reg axi_rvalid_reg;
    
    always @(posedge aclk) begin
        if (!aresetn) begin
            s_axi_awready <= 0;
            s_axi_wready <= 0;
            s_axi_bvalid <= 0;
            s_axi_bresp <= 2'b00;
            s_axi_arready <= 0;
            s_axi_rvalid <= 0;
            s_axi_rresp <= 2'b00;
            s_axi_rdata <= 0;
            
            reg_control <= 0;
            reg_input_addr <= 0;
            reg_output_addr <= 0;
            reg_input_width <= 640;
            reg_input_height <= 480;
            reg_interrupt <= 0;
            
            dpu_start <= 0;
            interrupt <= 0;
        end else begin
            // 寫入操作
            if (s_axi_awvalid && s_axi_wvalid && !s_axi_awready) begin
                s_axi_awready <= 1;
                s_axi_wready <= 1;
                
                case (s_axi_awaddr)
                    REG_CONTROL: begin
                        if (s_axi_wstrb[0]) reg_control[7:0] <= s_axi_wdata[7:0];
                        if (s_axi_wstrb[1]) reg_control[15:8] <= s_axi_wdata[15:8];
                        if (s_axi_wstrb[2]) reg_control[23:16] <= s_axi_wdata[23:16];
                        if (s_axi_wstrb[3]) reg_control[31:24] <= s_axi_wdata[31:24];
                        if (s_axi_wdata[0]) dpu_start <= 1; // 啟動 DPU
                    end
                    REG_INPUT_ADDR: reg_input_addr <= s_axi_wdata;
                    REG_OUTPUT_ADDR: reg_output_addr <= s_axi_wdata;
                    REG_INPUT_WIDTH: reg_input_width <= s_axi_wdata[15:0];
                    REG_INPUT_HEIGHT: reg_input_height <= s_axi_wdata[15:0];
                    REG_INTERRUPT: reg_interrupt <= s_axi_wdata;
                endcase
            end else begin
                s_axi_awready <= 0;
                s_axi_wready <= 0;
                dpu_start <= 0;
            end
            
            // 寫入回應
            if (s_axi_awready && s_axi_wready && !s_axi_bvalid) begin
                s_axi_bvalid <= 1;
                s_axi_bresp <= 2'b00;
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 0;
            end
            
            // 讀取操作
            if (s_axi_arvalid && !s_axi_arready) begin
                s_axi_arready <= 1;
                s_axi_rvalid <= 1;
                s_axi_rresp <= 2'b00;
                
                case (s_axi_araddr)
                    REG_CONTROL: s_axi_rdata <= reg_control;
                    REG_STATUS: s_axi_rdata <= {30'h0, dpu_idle, dpu_done};
                    REG_INPUT_ADDR: s_axi_rdata <= reg_input_addr;
                    REG_OUTPUT_ADDR: s_axi_rdata <= reg_output_addr;
                    REG_INPUT_WIDTH: s_axi_rdata <= {16'h0, reg_input_width};
                    REG_INPUT_HEIGHT: s_axi_rdata <= {16'h0, reg_input_height};
                    REG_INTERRUPT: s_axi_rdata <= reg_interrupt;
                    default: s_axi_rdata <= 32'h0;
                endcase
            end else begin
                s_axi_arready <= 0;
            end
            
            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 0;
            end
            
            // 中斷產生
            if (dpu_done && reg_interrupt[0]) begin
                interrupt <= 1;
            end else if (reg_interrupt[31]) begin // 清除中斷
                interrupt <= 0;
                reg_interrupt[31] <= 0;
            end
        end
    end
    
    // 輸出信號連接
    assign dpu_input_addr = reg_input_addr;
    assign dpu_output_addr = reg_output_addr;
    assign dpu_input_width = reg_input_width;
    assign dpu_input_height = reg_input_height;
    
    // 狀態暫存器更新
    always @(posedge aclk) begin
        if (!aresetn) begin
            reg_status <= 0;
        end else begin
            reg_status[0] <= dpu_done;
            reg_status[1] <= dpu_idle;
        end
    end

endmodule
