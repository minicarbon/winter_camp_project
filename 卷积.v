/*CONV2D.v*/
module CONV2D
#(
	parameter DATA_WIDTH = 16,
	parameter FMAP_SIZE = 32,
	parameter KERNEL_SIZE = 5,
	parameter STRIDE = 1
)
(
	input clk, 
	input rst_n,
	input ena,
	input clear,
	input [DATA_WIDTH*KERNEL_SIZE-1:0]tap,//5*16-1,feature map列向量输入
	input [KERNEL_SIZE*KERNEL_SIZE*DATA_WIDTH-1:0]w,//卷积核输入5*5*16-1
	output [DATA_WIDTH-1:0]conv_out,//输出16位卷积结果
	output valid,
	output done
);
	
	function integer clogb2 (input integer bit_depth);//定义计算位宽函数，进行log2d的计算
	begin
        for(clogb2 = 0; bit_depth > 0; clogb2 = clogb2 + 1)
            bit_depth = bit_depth >> 1;
	end
	endfunction
	
	localparam CNT_BIT_NUM = clogb2((FMAP_SIZE * (FMAP_SIZE + 1)));//log2^32*32=10
	localparam CNT_LINE_BIT_NUM = clogb2(FMAP_SIZE);//log2^32=5
	
	reg[CNT_BIT_NUM-1:0] cnt;//像素点位置记录
	reg[CNT_LINE_BIT_NUM-1:0] cnt_col;//列计数
	reg[CNT_LINE_BIT_NUM-1:0] cnt_row;//行计数
	reg[CNT_LINE_BIT_NUM-1:0] cnt_stride_col;//用于stride间隔判断
	reg[CNT_LINE_BIT_NUM-1:0] cnt_stride_row;//用于stride间隔判断
	
	reg shift;
	reg sum_valid;
	wire stride_valid;
	wire shift_valid;
	wire win_valid;
	reg win_valid_s1;
	reg win_valid_s2;
	reg win_valid_s3;
	
	genvar i, j;
	
/*******************************************generate signals************************************/
	assign stride_valid = (cnt_stride_col == 0 & cnt_stride_row == 0) ? 1 : 0;
	assign shift_valid = (shift && (cnt_col < FMAP_SIZE - (KERNEL_SIZE - 1)));
	assign win_valid = (shift_valid && stride_valid);
	assign valid = win_valid_s3 & sum_valid;
	assign done = (ena && (cnt == FMAP_SIZE * KERNEL_SIZE + FMAP_SIZE * (FMAP_SIZE - KERNEL_SIZE + 1) + (2 + 1) - 1)) ? 1 : 0;
	assign conv_out = (valid) ? sum : 0;
	
reg [DATA_WIDTH-1:0] window_data[KERNEL_SIZE*KERNEL_SIZE-1:0];//列向量寄存器存储feature map 16位*25个数,,menroy	
/******************************************data window shift-in**********************************/
	//need two generate block as 'window_data' has a reg type, while 'tap' is wire type
	generate
		if(KERNEL_SIZE > 1) begin
			for(i = 0 ; i < KERNEL_SIZE; i = i + 1) begin
				for(j = 0; j < KERNEL_SIZE - 2; j = j + 1) begin
					always@(posedge clk)
						window_data[(i)*KERNEL_SIZE+j] <= window_data[i*KERNEL_SIZE+j+1];
													/*window_data寄存器组每行按元素左移*/
				end
			end
	
			if(KERNEL_SIZE > 1) begin
				for(i = 0 ; i < KERNEL_SIZE; i = i + 1) begin
					always@(posedge clk)
						window_data[(i)*KERNEL_SIZE+KERNEL_SIZE-2] <= tap[(i+1)*DATA_WIDTH-1-:DATA_WIDTH];/*同时行尾接收tap的输入数据，将整个window_data填满后就产生了一个有效窗口数据*/
				end
			end
		end
		else begin
			always@(posedge clk)
				window_data[0] <= tap[DATA_WIDTH-1:0];
		end
	endgenerate
	/*
	   0000*               000**
	   0000*               000**
	   0000*      _--->    000**
	   0000*               000**
	   0000*               000**
	*/
	
/******************************************adder tree**********************************/
	//partial sum adder tree
	//generated when KERNEL_SIZE > 1
	
	reg [DATA_WIDTH-1:0] product[KERNEL_SIZE*KERNEL_SIZE-1:0];//乘法寄存器，16位*25个数
	reg [DATA_WIDTH-1:0] partial_sum[KERNEL_SIZE-1:0];//部分和，16位*5个数=80,一维数组，名字后面是地址，product[1]表示地址
	reg [DATA_WIDTH-1:0] sum;//结果寄存器16位
	wire[DATA_WIDTH-1:0] partial_adder_out[KERNEL_SIZE*KERNEL_SIZE-1:0];//16位*25个数
	wire[DATA_WIDTH-1:0] sum_adder_out[KERNEL_SIZE-2:0];//16位*3个数
	generate
		if(KERNEL_SIZE > 1) begin
			for(i = 0; i < KERNEL_SIZE; i = i + 1) begin
				ADDER
				#(
					.DATA_WIDTH(DATA_WIDTH)
				)
				u_partial_adder_head
				(
					.ina(product[i*KERNEL_SIZE]),
					.inb(product[i*KERNEL_SIZE+1]),
					.out(partial_adder_out[i*KERNEL_SIZE])
				);
			end
		
			for(i = 0; i < KERNEL_SIZE; i = i + 1) begin
				for(j = 2; j < KERNEL_SIZE; j = j + 1) begin
					ADDER
					#(
						.DATA_WIDTH(DATA_WIDTH)
					)
					u_partial_adder_remain
					(
						.ina(partial_adder_out[i*KERNEL_SIZE+j-2]),
						.inb(product[i*KERNEL_SIZE+j]),
						.out(partial_adder_out[i*KERNEL_SIZE+j-1])
					);
				end
			end
		end
	endgenerate
		
	//conv sum adder tree
	//generated when KERNEL_SIZE > 1
	generate
		if(KERNEL_SIZE > 1) begin
			ADDER
			#(
				.DATA_WIDTH(DATA_WIDTH)
			)
			u_sum_adder_head
			(
				.ina(partial_sum[0]),
				.inb(partial_sum[1]),
				.out(sum_adder_out[0])
			);
	
			for(i = 2; i < KERNEL_SIZE; i = i + 1) begin
				ADDER
				#(
					.DATA_WIDTH(DATA_WIDTH)
				)
				u_sum_adder_remain
				(
					.ina(sum_adder_out[i-2]),
					.inb(partial_sum[i]),
					.out(sum_adder_out[i-1])
				);
			end
		end
	endgenerate
	
/*****************************************3-stage pipeline*********************************/
	//stage 1: generate multiplication product乘积
	generate 
		if(KERNEL_SIZE > 1) begin
			for(i = 0 ; i < KERNEL_SIZE; i = i + 1) begin
				for(j = 0 ; j < KERNEL_SIZE-1; j = j + 1) begin
					always@(posedge clk)
						product[i*KERNEL_SIZE+KERNEL_SIZE-1] <= w[((i*KERNEL_SIZE+KERNEL_SIZE-1)+1)*DATA_WIDTH-1-:DATA_WIDTH] * tap[(i+1)*DATA_WIDTH-1-:DATA_WIDTH];
				end
			end
	
			for(i = 0 ; i < KERNEL_SIZE; i = i + 1) begin
				for(j = 0 ; j < KERNEL_SIZE-1; j = j + 1) begin
					always@(posedge clk)
						product[i*KERNEL_SIZE+j] <= w[((i*KERNEL_SIZE+j)+1)*DATA_WIDTH-1-:DATA_WIDTH] * window_data[i*KERNEL_SIZE+j];
				end
			end
		end
		
		else begin
			always @(posedge clk)
				product[0] <= w[0] * tap[DATA_WIDTH-1:0];
		end
	endgenerate
	
	//stage 2: generate partial sum 
	generate
		if(KERNEL_SIZE > 1) begin
			for(i = 0; i < KERNEL_SIZE; i = i + 1) begin
				always @(posedge clk or negedge rst_n)
					if(!rst_n)
						partial_sum[i] <= 0;
					else
						partial_sum[i] <= partial_adder_out[i*KERNEL_SIZE+KERNEL_SIZE-2];
			end
		end
		else begin
			always @(posedge clk or negedge rst_n)
				if(!rst_n)
					partial_sum[0] <= 0;
				else
					partial_sum[0] <= product[0];
		end
	endgenerate
	
	//stage 3: generate conv sum 
	generate 
		if(KERNEL_SIZE > 1) begin
			always@(posedge clk or negedge rst_n)
				if(!rst_n)
					sum <= 0;
				else
					sum <= sum_adder_out[KERNEL_SIZE-2];
		end
		else begin
			always@(posedge clk or negedge rst_n)
				if(!rst_n)
					sum <= 0;
				else
					sum <= partial_sum[0];
		end
	endgenerate
			
			
/**********************************************generate control logic*****************************/	
	//global counter
	always @(posedge clk or negedge rst_n)
		if(!rst_n)
			cnt <= 0;
		else if(clear)
			cnt <= 0;
		else if(!ena)
			cnt <= cnt;
		else
			cnt <= cnt + 1;
	
	//valid data window detector
	always @(posedge clk or negedge rst_n)
		if(!rst_n)
			cnt_col <= 0;
		else if(clear)
			cnt_col <= 0;
		else if(!ena)
			cnt_col <= cnt_col;
		else if(shift) begin
			if(cnt_col == FMAP_SIZE - 1)
				cnt_col <= 0;
			else
				cnt_col <= cnt_col + 1;
		end
	
	//valid shift period detector
	always @(posedge clk or negedge rst_n)
		if(!rst_n)
			shift <= 0;
		else if(clear)
			shift <= 0;
		else if(!ena)
			shift <= 0;
		else if(cnt == FMAP_SIZE * KERNEL_SIZE + (KERNEL_SIZE - 1) - 1)
			shift <= 1'b1;
		else if(cnt == FMAP_SIZE * KERNEL_SIZE + FMAP_SIZE * (FMAP_SIZE - KERNEL_SIZE + 1) - 1)
			shift <= 1'b0;
    
	always @(posedge clk or negedge rst_n)
		if(!rst_n)
			cnt_stride_col <= 0;
		else if(clear)
			cnt_stride_col <= 0;
		else if(ena) begin
			if(shift) begin
				if(cnt_stride_col < STRIDE - 1)
					cnt_stride_col <= cnt_stride_col + 1;
				else
					cnt_stride_col = 0;
			end
		end
			
	always @(posedge clk or negedge rst_n)
		if(!rst_n)
			cnt_stride_row <= 0;
		else if(clear)
			cnt_stride_row <= 0;
		else if(ena) begin
			if(cnt_col == FMAP_SIZE - 1) begin
				if(cnt_stride_row < STRIDE - 1)
					cnt_stride_row <= cnt_stride_row + 1;
				else
					cnt_stride_row = 0;
			end
		end
	
	//valid sum period detector
	always @(posedge clk or negedge rst_n)
	    if(!rst_n)
	    	sum_valid <= 0;
		else if(clear)
	    	sum_valid <= 0;
		else if(!ena)
	    	sum_valid <= 0;
	    else if(cnt == FMAP_SIZE * KERNEL_SIZE + (KERNEL_SIZE - 1) + (1 + 2) - 1)
	    	sum_valid <= 1;
		else if(cnt == FMAP_SIZE * KERNEL_SIZE + FMAP_SIZE * (FMAP_SIZE - KERNEL_SIZE + 1) + (1 + 2) - 1)
	    	sum_valid <= 0;

	 //'conv sum valid' sig comes 2 clocks later than ‘window valid’ sig,
	 //insert 2 regs
	 always @(posedge clk or negedge rst_n)
	 	if(!rst_n) begin
	 		win_valid_s1 <= 0;
	 		win_valid_s2 <= 0;
	 		win_valid_s3 <= 0;
	 	end
	 	else if(clear) begin
	 		win_valid_s1 <= 0;
	 		win_valid_s2 <= 0;
	 		win_valid_s3 <= 0;
	 	end
	 	else if(ena) begin
	 		win_valid_s1 <= win_valid;
	 		win_valid_s2 <= win_valid_s1;
	 		win_valid_s3 <= win_valid_s2;
	 	end
	 	
endmodule

