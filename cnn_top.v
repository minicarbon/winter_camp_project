`timescale 1ns/1ps

`include "global.h"
`include "calculate.h"
`include "ctrl.h"
`include "hg_acc_global.vh"

module cnn_top(
	/////////////////
	//global signal
	/////////////////
	input 									clk,
	input									top_rstn,
	
	////////////////
	//AXI_lite
	///////////////
	input [31:0]	cpu_start,
	input [31:0]	cpu_ins_a,
	input [31:0]	cpu_ins_b,
	input [31:0]	cpu_r_addr,
	input [31:0]	cpu_w_addr,
	input [31:0]	cpu_weight_addr,
	input [31:0]	cpu_data_dma2kernel_limit,
	input [31:0]	cpu_weight_dma2kernel_limit,
	output			cnn_over,
	
	/////////////
	//AXI master
	/////////////
	/*********** Axi Signal **********/
	output	wire	[`HG_ACC_AXI_AW_ID_W	-1 :0]  awid,
	output	wire	[`HG_ACC_AXI_AW_ADDR_W	-1 :0]  awaddr,
	output	wire	[`HG_ACC_AXI_AW_LEN_W	-1 :0]  awlen,
	output	wire	[`HG_ACC_AXI_AW_SIZE_W	-1 :0]  awsize,
	output	wire	[`HG_ACC_AXI_AW_BURS_W	-1 :0]  awburst,
	output	wire	[`HG_ACC_AXI_AW_LOCK_W	-1 :0]  awlock,
	output	wire	[`HG_ACC_AXI_AW_CACH_W	-1 :0]  awcache,
	output	wire	[`HG_ACC_AXI_AW_PORT_W	-1 :0]  awprot,
	output	wire	                                awvalid,
	input	wire	                                awready,
	output	wire	[`HG_ACC_AXI_W_ID_W		-1 :0]  wid,
	output	wire	[`HG_ACC_AXI_W_DATA_W	-1 :0]  wdata,
	output	wire	[`HG_ACC_AXI_W_STRB_W	-1 :0]  wstrb,
	output	wire	                                wlast,
	output	wire	                                wvalid,
	input	wire	                                wready,
	// input	wire	[`HG_ACC_AXI_B_ID_W		-1 :0]  bid,
	input  	wire	[`HG_ACC_AXI_B_RESP_W	-1 :0]  bresp,
	input	wire	                                bvalid,
	output	wire	                                bready,
	output 	wire	[`HG_ACC_AXI_AR_ID_W	-1 :0]  arid,
	output 	wire	[`HG_ACC_AXI_AR_ADDR_W	-1 :0]  araddr,
	output 	wire	[`HG_ACC_AXI_AR_LEN_W	-1 :0]  arlen,
	output 	wire	[`HG_ACC_AXI_AR_SIZE_W	-1 :0]  arsize,
	output 	wire	[`HG_ACC_AXI_AR_BURS_W	-1 :0]  arburst,
	output 	wire	[`HG_ACC_AXI_AR_LOCK_W	-1 :0]  arlock,
	output 	wire	[`HG_ACC_AXI_AR_CACH_W	-1 :0]  arcache,
	output 	wire	[`HG_ACC_AXI_AR_PROT_W	-1 :0]  arprot,
	output	wire	                                arvalid,
	input	wire	                                arready,
	// input  	wire	[`HG_ACC_AXI_R_ID_W		-1 :0]  rid,
	input  	wire	[`HG_ACC_AXI_R_DATA_W	-1 :0]  rdata,
	input	wire	[`HG_ACC_AXI_R_RESP_W	-1 :0]  rresp,
	input	wire	                                rlast,
	input	wire	                                rvalid,
	output	wire	                                rready
);

	/////////////////
	//ctrl bus
	/////////////////
	wire									start;
	//wire	[31:0]							cpu_ins_a;
	//wire	[31:0]							cpu_ins_b;
	wire									dma_start;
	wire									dma_tran_over;
	wire	[`BUF_MODE_W		-1	:0]		ctr2buf_mode;
	wire	[2:0]							ctr2wb_mode;
	wire	[2:0]							ctr2dpi_mode;
	wire	[`CAL_MODE_W		-1	:0]		ctr2cal_mode;
	wire	[`DPO_MODE_W		-1	:0]		ctr2dpo_mode;
	wire	[3:0]							ctr2dma_mode;

	wire									ctr2buf_vld;
	wire	[`BUF_INS_W			-1	:0]		ctr2buf_ins;
	wire									IB_rdy_ps;
	wire									ctr2wb_vld;
	wire	[`BUF_INS_W			-1	:0]		ctr2wb_ins;
	wire									WB_rdy_ps;

	/////////////////
	//buf bus
	/////////////////
	//wire	[`BUF_MODE_W		-1	:0]		ctr2buf_mode;
	//wire	[`BUF_INS_W			-1	:0]		ctr2buf_ins;
	wire	[`BUF_DMA_IN_DATA_W	-1	:0]		dma2buf_data;
	wire	[`BUF_DPO_IN_DATA_W	-1	:0]		dpo2buf_data;
	
	wire	[`DPI_INBUF_DATA_W	-1	:0]		inbuf2dpi_data;
	wire	[`CAL_OUTBUF_DATA_W	-1	:0]		outbuf2cal_data;
	wire	[`BUF_DMA_OUT_DATA_W-1	:0]		buf2dma_data;
	
	wire									IB_vld_ns;
	wire	[10:0]							IB_ins;
	wire									DI_rdy_ps;
	wire									EX_vld_ns;
	wire	[10:0]							EX_ins;
	wire									DO_vld_ns;
	wire	[12:0]							DO_ins_out;
	wire									dma2buf_vld;
	wire									buf2dma_vld;
	wire									dma2buf_rdy;
	
	/////////////////
	//weight buf bus
	/////////////////
	//wire	[`BUF_MODE_W		-1	:0]		ctr2wb_mode;
	wire	[`DPI_IN_WEIGHT_W	-1	:0]		dma2wb_data;
	wire	[`DPI_IN_WEIGHT_W	-1	:0]		wb2dpi_data;
	//pipeline
	//wire									ctr2wb_vld;
	//wire	[`BUF_INS_W			-1	:0]		ctr2wb_ins;
	//wire									WB_rdy_ps;
	wire									WB_vld_ns;
	wire	[10:0]							WB_ins;
	//wire									DI_rdy_ps;
	wire									dma2wb_vld;
	wire									wb2dma_rdy;
	
	/////////////////
	//dpi bus
	/////////////////
	//wire	[`DPI_MODE_W		-1	:0]		ctr2dpi_mode;
	//wire	[`DPI_INBUF_DATA_W	-1	:0]		inbuf2dpi_data;
	//wire	[`DPI_IN_WEIGHT_W	-1	:0]		wb2dpi_data;
	wire	[`DPI_IN_BIAS_W		-1	:0]		bb2dpi_data;
	wire	[`CAL_IN_DATA_W		-1	:0]		dpi2cal_data;
	wire	[`CAL_IN_WEIGHT_W	-1	:0]		dpi2cal_weight;
	wire	[`CAL_IN_BIAS_W		-1	:0]		dpi2cal_bias;
	//wire									IB_vld_ns;
	//wire	[9:0]							IB_ins;
	//wire									DI_rdy_ps;
	wire									DI_vld_ns;
	wire	[10:0]							DI_ins;
	wire									IR_rdy_ps;
	
	/////////////////
	//cal bus
	/////////////////
	//wire	[`CAL_MODE_W		-1	:0]		ctr2cal_mode;
	//wire	[`CAL_IN_DATA_W		-1	:0]		dpi2cal_data;
	//wire	[`CAL_IN_WEIGHT_W	-1	:0]		dpi2cal_weight;
	//wire	[`CAL_IN_BIAS_W		-1	:0]		dpi2cal_bias;
	//wire	[`CAL_OUTBUF_DATA_W	-1	:0]		outbuf2cal_data;
	wire 	[`CAL_OUT_DATA_W	-1	:0]		cal2dpo_data;
	//wire									DI_vld_ns;
	//wire	[9:0]							DI_ins;
	//wire									IR_rdy_ps;
	wire									OR_vld_ns;
	wire	[10:0]							OR_ins;
	wire									DO_rdy_ps;
	
	/////////////////
	//dpo bus
	/////////////////
	//wire	[`DPO_MODE_W		-1	:0]		ctr2dpo_mode;
	//wire	[`DPO_IN_DATA_W		-1	:0]		cal2dpo_data;
	//wire	[`DPO_OUTBUF_DATA_W	-1	:0]		dpo2buf_data;
	//pipeline
	//wire									OR_vld_ns;
	//wire	[9:0]							OR_ins;
	//wire									DO_rdy_ps;
	//wire									DO_vld_ns;
	wire	[14:0]							DO_ins;
	
	
	/////////////////
	//axi dma signal
	/////////////////
	wire    [`HG_ACC_AXI_AR_ADDR_W  -1 :0]  ar_addr;
    wire                                    ar_addr_en;
    wire                                    ar_addr_get;
    wire                                    r_data_ready;
    wire                                    r_data_valid;
    wire                                    r_data_end;
    wire    [`HG_ACC_AXI_R_DATA_W   -1 :0]  r_data;
    wire    [`HG_ACC_AXI_AW_ADDR_W  -1 :0]  aw_addr;
    wire                                    aw_addr_en;
    wire                                    aw_addr_get;
    wire    [`HG_ACC_AXI_W_DATA_W   -1 :0]  w_data;
    wire                                    w_data_valid;
    wire                                    w_data_ready;
    wire                                    w_data_end;
	
	//start_pluse
	reg		cpu_start_1d;
	//wire	start;

	reg rstn;
	wire rstn_asyn;
	reg cnn_over_1d;
	assign rstn_asyn = top_rstn && ~cnn_over_1d;
	
always @(posedge clk or negedge rstn) begin
	if(~rstn) begin
		cnn_over_1d <= 'b0;
	end else begin
		cnn_over_1d <= cnn_over;
	end
end

reg q1;

always @(posedge clk or negedge rstn_asyn)
	if(~rstn_asyn)
		{rstn,q1} <= 2'b0;
	else 
		{rstn,q1} <= {q1,1'b1};
	
always @(posedge clk or negedge rstn) begin
	if(~rstn) begin
		cpu_start_1d <= 'b0;
	end else begin
		cpu_start_1d <= cpu_start[0];
	end
end
assign start = ~cpu_start_1d && cpu_start[0];
	
	
	
ctrl ctrl(
	.clk				(clk),
	.rstn				(rstn),
	//input             
	.start				(start),
	.cpu_ins_a			(cpu_ins_a),
	.cpu_ins_b			(cpu_ins_b),
	.dma_start			(dma_start),
	.dma_tran_over		(dma_tran_over),
	.cnn_over			(cnn_over),
    //output            
	.ctr2buf_mode		(ctr2buf_mode),
	.ctr2wb_mode		(ctr2wb_mode),
	.ctr2dpi_mode		(ctr2dpi_mode),
	.ctr2cal_mode		(ctr2cal_mode),
	.ctr2dpo_mode		(ctr2dpo_mode),
	.ctr2dma_mode		(ctr2dma_mode),
	//pipeline          
	.ctr2buf_vld		(ctr2buf_vld),
	.ctr2buf_ins		(ctr2buf_ins),
	.IB_rdy_ps			(IB_rdy_ps), 
	.ctr2wb_vld			(ctr2wb_vld),
	.ctr2wb_ins			(ctr2wb_ins),
	.WB_rdy_ps			(WB_rdy_ps)
); 	
	
	
data_buffer data_buffer(
	.clk				(clk),
	.rstn				(rstn),
	//input             
	.ctr2buf_mode		(ctr2buf_mode),
	.dma2buf_data		(dma2buf_data),
	.dpo2buf_data		(dpo2buf_data),
    //output            
	.inbuf2dpi_data		(inbuf2dpi_data),
	.outbuf2cal_data	(outbuf2cal_data),
	.buf2dma_data		(buf2dma_data),
	//pipeline    
	.ctr2buf_vld		(ctr2buf_vld),
	.ctr2buf_ins		(ctr2buf_ins),
	.IB_rdy_ps			(IB_rdy_ps), 
	.IB_vld_ns			(IB_vld_ns),
	.IB_ins				(IB_ins),
	.DI_rdy_ps			(DI_rdy_ps),
	//psum_r pipeline   
	.EX_vld_ns			(EX_vld_ns),
	.EX_ins				(EX_ins),
	//psum_w pipeline   
	.DO_vld_ns			(DO_vld_ns),
	.DO_ins				(DO_ins),
	//dma controller singal
	.dma2buf_vld		(dma2buf_vld),
	.buf2dma_vld		(buf2dma_vld),
	.dma2buf_rdy		(dma2buf_rdy)
); 

weight_buffer weight_buffer(
	.clk				(clk),
	.rstn				(rstn),
	//input             
	.ctr2wb_mode		(ctr2wb_mode),
	.dma2wb_data		(dma2wb_data),
    //output            
	.wb2dpi_data		(wb2dpi_data),
	//pipeline    
	.ctr2wb_vld			(ctr2wb_vld),
	.ctr2wb_ins			(ctr2wb_ins),
	.WB_rdy_ps			(WB_rdy_ps), 
	.WB_vld_ns			(WB_vld_ns),
	.WB_ins				(WB_ins),
	.DI_rdy_ps			(DI_rdy_ps),
	//ddr pipeline   
	.dma2wb_vld			(dma2wb_vld),
	.wb2dma_rdy			(wb2dma_rdy)
); 

biases_buffer biases_buffer(
	.clk 									(clk),
	.rstn									(top_rstn),
	//input bus                             
	.ctr2bb_mode							(ctr2wb_mode),
	//output bus                            
	.bb2dpi_data							(bb2dpi_data),
	//pipeline                              
	.ctr2bb_vld								(ctr2wb_vld),
	.ctr2bb_ins								(ctr2wb_ins),
	.BB_rdy_ps								(),
	//bb2dpi                                
	.BB_vld_ns								(),
	.BB_ins									(),
	.DI_rdy_ps								(DI_rdy_ps)
);

//assign bb2dpi_data = 'b0;

dpi dpi(
	.clk				(clk),
	.rstn				(rstn),
	//input             
	.ctr2dpi_mode		(ctr2dpi_mode),
	.inbuf2dpi_data		(inbuf2dpi_data),
	//.outbuf2dpi_data	(outbuf2dpi_data),
	.wb2dpi_data		(wb2dpi_data),
	.bb2dpi_data		(bb2dpi_data),
    //output            
	.dpi2cal_data		(dpi2cal_data),
	.dpi2cal_weight		(dpi2cal_weight),
	.dpi2cal_bias		(dpi2cal_bias),
	//pipeline          
	.IB_vld_ns			(IB_vld_ns),
	.IB_ins				(IB_ins),
	.DI_rdy_ps			(DI_rdy_ps),
	.DI_vld_ns			(DI_vld_ns),
	.DI_ins				(DI_ins), 
	.IR_rdy_ps			(IR_rdy_ps)
); 

calculate calculate(
	.clk				(clk),
	.rstn				(rstn),
	//input             
	.ctr2cal_mode		(ctr2cal_mode),
	.dpi2cal_data		(dpi2cal_data),
	.dpi2cal_weight		(dpi2cal_weight),
	.dpi2cal_bias		(dpi2cal_bias),
	.outbuf2cal_data	(outbuf2cal_data),
    //output            
	.cal2dpo_data		(cal2dpo_data),
	//pipeline          
	.DI_vld_ns			(DI_vld_ns),
	.DI_ins				(DI_ins),
	.IR_rdy_ps			(IR_rdy_ps),
	.OR_vld_ns			(OR_vld_ns),
	.OR_ins				(OR_ins), 
	.DO_rdy_ps			(DO_rdy_ps),
	.EX_vld_ns			(EX_vld_ns),
	.EX_ins				(EX_ins)
); 

dpo dpo(
	.clk				(clk),
	.rstn				(rstn),
	//input             
	.ctr2dpo_mode		(ctr2dpo_mode),
	.cal2dpo_data		(cal2dpo_data),
    //output            
	.dpo2buf_data		(dpo2buf_data),
	//pipeline          
	.OR_vld_ns			(OR_vld_ns),
	.OR_ins				(OR_ins),
	.DO_rdy_ps			(DO_rdy_ps),
	.DO_vld_ns			(DO_vld_ns),
	.DO_ins_out			(DO_ins)
); 

 axi_interface axi_interface(
	/********** Global Signal ************/
	.clk				(clk),
	.rstn				(rstn),
	/********** SP Module Signal **********/
	.ar_addr			(ar_addr),
    .ar_addr_en			(ar_addr_en),
    .ar_addr_get		(ar_addr_get),
    .r_data_ready		(r_data_ready),
    .r_data_valid		(r_data_valid),
    .r_data_end			(r_data_end),
    .r_data				(r_data),
    .aw_addr			(aw_addr),
    .aw_addr_en			(aw_addr_en),
    .aw_addr_get		(aw_addr_get),
    .w_data				(w_data),
    .w_data_valid		(w_data_valid),
    .w_data_ready		(w_data_ready),
    .w_data_end			(w_data_end),
	/********** AXI Signal **********/
	.awid				(awid),
	.awaddr				(awaddr),
	.awlen				(awlen),
	.awsize				(awsize),
	.awburst			(awburst),
	.awlock				(awlock),
	.awcache			(awcache),
	.awprot				(awprot),
	.awvalid			(awvalid),
	.awready			(awready),
	.wid				(wid),
	.wdata				(wdata),
	.wstrb				(wstrb),
	.wlast				(wlast),
	.wvalid				(wvalid),
	.wready				(wready),
	.bresp				(bresp),
	.bvalid				(bvalid),
	.bready				(bready),
	.arid				(arid),
	.araddr				(araddr),
	.arlen				(arlen),
	.arsize				(arsize),
	.arburst			(arburst),
	.arlock				(arlock),
	.arcache			(arcache),
	.arprot				(arprot),
	.arvalid			(arvalid),
	.arready			(arready),
	.rdata				(rdata),
	.rresp				(rresp),
	.rlast				(rlast),
	.rvalid				(rvalid),
	.rready				(rready)
);

cnn_dma cnn_dma(
	/********** Global Signal ************/
	.clk				(clk),
	.rstn				(rstn),
	/********** 2axi-interface Module Signal **********/
	.ar_addr			(ar_addr),
    .ar_addr_en			(ar_addr_en),
    .ar_addr_get		(ar_addr_get),
    .r_data_ready		(r_data_ready),
    .r_data_valid		(r_data_valid),
    .r_data_end			(r_data_end),
    .r_data				(r_data),
    .aw_addr			(aw_addr),
    .aw_addr_en			(aw_addr_en),
    .aw_addr_get		(aw_addr_get),
    .w_data				(w_data),
    .w_data_valid		(w_data_valid),
    .w_data_ready		(w_data_ready),
    .w_data_end			(w_data_end),
	/********** controller Signal **********/
	.ctr2dma_mode				(ctr2dma_mode),
	.ddr_r_addr_init			(cpu_r_addr),
	.ddr_w_addr_init			(cpu_w_addr),
	.ddr_weight_addr_init		(cpu_weight_addr),
	.r_data_out2kernel_limit	(cpu_data_dma2kernel_limit),
	.r_weight_out2kernel_limit	(cpu_weight_dma2kernel_limit),
	.dma_start					(dma_start),
	.dma_tran_over				(dma_tran_over),
	
	/********** data buffer Signal **********/
	.dma2buf_data				(dma2buf_data),
	.dma2buf_vld				(dma2buf_vld),//
	.buf2dma_rdy				(1'b1),
	
	/********** weight buffer Signal **********/
	.dma2wb_data				(dma2wb_data),
	.dma2wb_vld					(dma2wb_vld),
	.wb2dma_rdy					(wb2dma_rdy),
	
	/********** out buffer Signal **********/
	.buf2dma_data				(buf2dma_data),
	.buf2dma_vld				(buf2dma_vld),//
	.dma2buf_rdy				(dma2buf_rdy)//

);















endmodule