#include "cnn.h"

float max(float a, float b){if(a>b) return a;return b;}

float active(float a){if(a<0) return 0.1*a;return a;}

float multadd(float a,float b,float c){return a+b*c;}

//int in_channel ;
//int in_width ;
//int pooling ;

void cnn(
	hls::stream<float> &strm_in,
	hls::stream<float> &strm_out,
	int in_channel_axi,
	int in_width_axi,
	int pooling_axi,
	int weight_num_axi,
	int input_size_axi
)
{
//#pragma HLS STREAM variable=in_channel off dim=1
//#pragma HLS STREAM variable=in_width off dim=1
//#pragma HLS STREAM variable=pooling off dim=1
#pragma HLS INTERFACE axis register both port=strm_out
#pragma HLS INTERFACE axis register both port=strm_in
#pragma HLS INTERFACE s_axilite register port=in_channel_axi bundle=BUS_A
#pragma HLS INTERFACE ap_stable port=in_channel_axi
#pragma HLS INTERFACE s_axilite register port=in_width_axi bundle=BUS_A
#pragma HLS INTERFACE ap_stable port=in_width_axi
#pragma HLS INTERFACE s_axilite register port=pooling_axi bundle=BUS_A
#pragma HLS INTERFACE ap_stable port=pooling_axi
#pragma HLS INTERFACE s_axilite register port=weight_num_axi bundle=BUS_A
#pragma HLS INTERFACE ap_stable port=weight_num_axi
#pragma HLS INTERFACE s_axilite register port=input_size_axi bundle=BUS_A
#pragma HLS INTERFACE ap_stable port=input_size_axi
#pragma HLS INTERFACE s_axilite register port=return bundle=BUS_A
//#pragma HLS INTERFACE ap_hs register port=return
#pragma HLS DATAFLOW

	//float biases;
	float weights[WEIGHTSIZE];
	#pragma HLS STREAM variable=weights depth=9220 dim=1
	
	float output_buf[208*208];
	#pragma HLS RESOURCE variable=output_buf core=RAM_2P_BRAM

	//int in_channel = 3;
	//int in_width = 208;
	//int pooling = 0;
	volatile int in_channel = in_channel_axi;
#pragma HLS STREAM variable=in_channel off dim=1
	volatile int in_width = in_width_axi;
#pragma HLS STREAM variable=in_width off dim=1
	volatile int pooling = pooling_axi;
#pragma HLS STREAM variable=pooling off dim=1
	
	volatile int weight_num = weight_num_axi;//9 * in_channel + 1;
#pragma HLS STREAM variable=weight_num off dim=1
	volatile int input_size = input_size_axi;//in_width*in_width*in_channel + weight_num;
#pragma HLS STREAM variable=input_size off dim=1
	
	hls::LineBuffer<3,210,float> line_buf;
	#pragma HLS RESOURCE variable=line_buf core=RAM_2P_BRAM
	#pragma HLS ARRAY_PARTITION variable=line_buf dim=1 complete
	hls::LineBuffer<1,104,float> pool_line_buf;
	#pragma HLS RESOURCE variable=pool_line_buf core=RAM_2P_BRAM

	hls::Window<3,3,float> con_win;
	hls::Window<3,3,float> weight_win;
	
	hls::stream<float> photo_in("photo_in");
	hls::stream<float> con_out("con_out");
	
	assert(in_channel < 1025);
	assert(weight_num < 9218);
	assert(in_width < 210);
	assert(input_size < 208*208*16+3+9*16);

	load_weight:for (int i=0;i<input_size;i++){
		float input_data;
		input_data = strm_in.read();
		if(i<weight_num)
			weights[i] = input_data;
		else if(i>=weight_num)
			photo_in.write(input_data);
	}

	for(int i=0;i<in_width;i++){
		for(int j=0;j<in_width;j++){
			output_buf[i*in_width+j]=0;
		}
	}

	float biases = weights[0];

	for(int ch=0;ch<in_channel;ch++){
		//load the weights window;
		load_window1:for(int i=0;i<3;i++){
			load_window2:for(int j=0;j<3;j++){
				weight_win.insert_pixel(weights[ch*9+i*3+j+1],i,j);
			}
		}

		//read and con the photo
		con_row:for(int row=0;row<in_width+1;row++){
			con_col:for(int col=0;col<in_width+1;col++){
				#pragma HLS PIPELINE II=1 rewind
				#pragma HLS DEPENDENCE variable=line_buf inter false
				#pragma HLS DEPENDENCE variable=output_buf inter false
				float data;
				if(row == in_width || col == in_width)
					data = 0;
				else
					data = photo_in.read();

				//line buffer shift and load
				line_buf.shift_pixels_up(col);
				line_buf.insert_bottom_row(data,col);

				//con_window
				con_win.shift_pixels_left();
				if(row<=1)
					con_win.insert_pixel(0,0,2);
				else
					con_win.insert_pixel(line_buf.getval(0,col),0,2);
				con_win.insert_pixel(line_buf.getval(1,col),1,2);
				con_win.insert_pixel(line_buf.getval(2,col),2,2);
	
				//convolution
				if(row>=1 && col>=1){

					float acc = 0;
					/*
					for(int i=0;i<3;i++){
						#pragma HLS UNROLL
						for(int j=0;j<3;j++){
							#pragma HLS UNROLL
							acc += weight_win.getval(i,j) * con_win.getval(i,j);
							//acc = multadd(acc,weight_win.getval(i,j),con_win.getval(i,j));
						}
					}*/

					float tmpp ;
					if(ch==0)
						tmpp = biases;
					else
						tmpp = output_buf[(row-1)*in_width+col-1];
					float tmp0 = weight_win.getval(0,0) * con_win.getval(0,0);
					float tmp1 = weight_win.getval(0,1) * con_win.getval(0,1);
					float tmp2 = weight_win.getval(0,2) * con_win.getval(0,2);
					float tmp3 = weight_win.getval(1,0) * con_win.getval(1,0);
					float tmp4 = weight_win.getval(1,1) * con_win.getval(1,1);
					float tmp5 = weight_win.getval(1,2) * con_win.getval(1,2);
					float tmp6 = weight_win.getval(2,0) * con_win.getval(2,0);
					float tmp7 = weight_win.getval(2,1) * con_win.getval(2,1);
					float tmp8 = weight_win.getval(2,2) * con_win.getval(2,2);
					float acc0 = tmp0+tmp1;
					float acc1 = tmp2+tmp3;
					float acc2 = tmp4+tmp5;
					float acc3 = tmp6+tmp7;
					float acc4 = tmpp+tmp8;
					float acc5 = acc0+acc1;
					float acc6 = acc2+acc3;
					float acc7 = acc4+acc5;
					float acc8 = acc6+acc7;
					//float acc = tmp0+tmp1+tmp2+tmp3+tmp4+tmp5+tmp6+tmp7+tmp8;*/
					output_buf[(row-1)*in_width+col-1] = acc8;
					//strm_out.write(acc);
					if(ch == in_channel-1){
						float out_data = acc8;//output_buf[(row-1)*in_width+col-1] ;//+ biases;
						con_out.write(out_data);
						//strm_out.write(out_data);
					}
				}
			}
		}
	}

	//pooling
	for(int row=0;row<in_width+(pooling==2?1:0);row++){
		for(int col=0;col<in_width+(pooling==2?1:0);col++){
			#pragma HLS PIPELINE
			#pragma HLS DEPENDENCE variable=pool_line_buf intra true
			#pragma HLS DEPENDENCE variable=pool_line_buf inter true
			//read the resule of convolution
			float pool_data;
			if(row==in_width || col==in_width)
				pool_data = 0;
			else
				pool_data = con_out.read();
			float out_data;
			
			//different type of pooling
			if(pooling==0)
				out_data = pool_data;
			else if(pooling==1){
				float pool_tmp;
				float pool_1_insert_data;
				pool_tmp = max(pool_line_buf.getval(0,col/2),pool_data);
				out_data = pool_tmp;
				pool_1_insert_data = (row%2==0 && col%2==0) ? pool_data : pool_tmp;
				pool_line_buf.insert_bottom_row(pool_1_insert_data,col/2);
			}
			else if(pooling==2){
				float pool_sp_data;
				if(row>0 && col>0)
					out_data = max(pool_line_buf.getval(0,col-1),pool_data);
				if(col>0){
					float pool_tmp1 = max(pool_sp_data,pool_data);
					pool_line_buf.insert_bottom_row(pool_tmp1,col-1);
				}
				if(row>0){
					float pool_tmp2 = max(pool_line_buf.getval(0,col),pool_data);
					pool_line_buf.insert_bottom_row(pool_tmp2,col);
				}
				pool_sp_data = pool_data;
			}
			
			//output
			if(pooling==0 || (pooling==1 && row%2==1 && col%2==1) || (pooling==2 && row>0 && col>0)){
				float real_out_data = active(out_data);
				strm_out.write(real_out_data);
			}
		}
	}


	return;
}

