# Vivado TCL 腳本 - 建立 KV260 專案
# create_project.tcl

# 設定專案
create_project kv260_yolo_riscv ./project -part xck26-sfvc784-2LV-c
set_property board_part xilinx.com:kv260_som:part0:1.3 [current_project]

# 加入 IP 目錄
set_property ip_repo_paths {./ip_repo} [current_project]
update_ip_catalog

# 建立 Block Design
create_bd_design "system"

# 加入 Zynq UltraScale+ MPSoC
startgroup
create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e:3.4 zynq_ultra_ps_e_0
endgroup

# 配置 Zynq PS
set_property -dict [list \
  CONFIG.PSU__USE__M_AXI_GP0 {1} \
  CONFIG.PSU__USE__M_AXI_GP2 {1} \
  CONFIG.PSU__USE__S_AXI_GP0 {1} \
  CONFIG.PSU__USE__IRQ0 {1} \
  CONFIG.PSU__USE__IRQ1 {1} \
  CONFIG.PSU__UART0__PERIPHERAL__ENABLE {1} \
  CONFIG.PSU__UART0__PERIPHERAL__IO {MIO 18 .. 19} \
  CONFIG.PSU__USB0__PERIPHERAL__ENABLE {1} \
  CONFIG.PSU__USB0__PERIPHERAL__IO {MIO 52 .. 63} \
  CONFIG.PSU__SD1__PERIPHERAL__ENABLE {1} \
  CONFIG.PSU__SD1__PERIPHERAL__IO {MIO 39 .. 51} \
  CONFIG.PSU__ENET3__PERIPHERAL__ENABLE {1} \
  CONFIG.PSU__ENET3__PERIPHERAL__IO {MIO 64 .. 75} \
] [get_bd_cells zynq_ultra_ps_e_0]

# 加入 DPU (Deep Learning Processing Unit)
startgroup
create_bd_cell -type ip -vlnv xilinx.com:ip:dpu:1.4 dpu_0
endgroup

# 配置 DPU
set_property -dict [list \
  CONFIG.DPU_NUM {1} \
  CONFIG.DPU_ARCH {4096} \
  CONFIG.DPU_RAM_USAGE {low} \
  CONFIG.DPU_CHN_AUG_ENA {true} \
  CONFIG.DPU_DWCV_ENA {true} \
] [get_bd_cells dpu_0]

# 加入 RISC-V Rocket Core (自定義 IP)
startgroup
create_bd_cell -type ip -vlnv user.org:user:rocket_core:1.0 rocket_core_0
endgroup

# 加入 Video Pipeline IP
startgroup
create_bd_cell -type ip -vlnv xilinx.com:ip:v_proc_ss:2.3 v_proc_ss_0
create_bd_cell -type ip -vlnv xilinx.com:ip:mipi_csi2_rx_subsystem:5.2 mipi_csi2_rx_subsystem_0
endgroup

# 加入 AXI Interconnect
startgroup
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_interconnect_0
set_property -dict [list CONFIG.NUM_MI {4}] [get_bd_cells axi_interconnect_0]
endgroup

# 加入時脈精靈
startgroup
create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wiz:6.0 clk_wiz_0
set_property -dict [list \
  CONFIG.PRIM_IN_FREQ.VALUE_SRC USER \
  CONFIG.PRIM_IN_FREQ {99.999001} \
  CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {100} \
  CONFIG.CLKOUT2_REQUESTED_OUT_FREQ {200} \
  CONFIG.CLKOUT3_REQUESTED_OUT_FREQ {300} \
  CONFIG.NUM_OUT_PORTS {3} \
] [get_bd_cells clk_wiz_0]
endgroup

# 連接 PS 和 PL
# 連接時脈
connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_clk0] [get_bd_pins clk_wiz_0/clk_in1]
connect_bd_net [get_bd_pins clk_wiz_0/clk_out1] [get_bd_pins axi_interconnect_0/ACLK]
connect_bd_net [get_bd_pins clk_wiz_0/clk_out2] [get_bd_pins dpu_0/m_axi_dpu_aclk]
connect_bd_net [get_bd_pins clk_wiz_0/clk_out3] [get_bd_pins rocket_core_0/clk]

# 連接 AXI 介面
connect_bd_intf_net [get_bd_intf_pins zynq_ultra_ps_e_0/M_AXI_HPM0_FPD] [get_bd_intf_pins axi_interconnect_0/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_interconnect_0/M00_AXI] [get_bd_intf_pins dpu_0/S_AXI_CONTROL]
connect_bd_intf_net [get_bd_intf_pins axi_interconnect_0/M01_AXI] [get_bd_intf_pins rocket_core_0/S_AXI_LITE]

# 連接中斷
connect_bd_net [get_bd_pins dpu_0/interrupt] [get_bd_pins zynq_ultra_ps_e_0/pl_ps_irq0]
connect_bd_net [get_bd_pins rocket_core_0/interrupt] [get_bd_pins zynq_ultra_ps_e_0/pl_ps_irq1]

# 加入約束檔案
add_files -fileset constrs_1 -norecurse ./constraints/kv260_pins.xdc

# 產生 HDL wrapper
make_wrapper -files [get_files ./project/kv260_yolo_riscv.srcs/sources_1/bd/system/system.bd] -top
add_files -norecurse ./project/kv260_yolo_riscv.gen/sources_1/bd/system/hdl/system_wrapper.v

# 設定 top module
set_property top system_wrapper [current_fileset]

# 產生 bitstream
launch_runs impl_1 -to_step write_bitstream
wait_on_run impl_1
