#include "xparameters.h"
#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "ff.h"
#include "xdevcfg.h"
#include <stdlib.h>
#include <malloc.h>
#include "xaxidma.h"
#include "xaxivdma.h"
#include "xaxivdma_i.h"
#include "xcnn_hw.h"
#include "xcnn.h"
#include "xtime_l.h"
#include <math.h>
#include "xuartps.h"

#define VGA_VDMA_ID XPAR_AXIVDMA_0_DEVICE_ID
#define DISP_VTC_ID XPAR_VTC_0_DEVICE_ID
#define VID_VTC_ID XPAR_VTC_1_DEVICE_ID
#define VDMA_BASEADDR XPAR_AXI_VDMA_0_BASEADDR
#define CNN_BASEADDR XPAR_XCNN_ACCEL_RS_0_S_AXI_CONTROL_BUS_BASEADDR

# define PHOTO_PIXEL 307200

//////////////////////////////////////////////////////

#define VTC_BASEADDR XPAR_MIZ702_VTG_VGA_0_BASEADDR
//#define DDR_BASEADDR        0x00000000
#define DDR_BASEADDR        XPAR_PS7_DDR_0_S_AXI_BASEADDR
//#define UART_BASEADDR       0xe0001000
#define VDMA_BASEADDR       XPAR_AXI_VDMA_0_BASEADDR
#define H_STRIDE            640
#define H_ACTIVE            640
#define V_ACTIVE            480
#define pi					3.14159265358
#define COUNTS_PER_SECOND	(XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ)/64


#define VIDEO_LENGTH  (H_STRIDE*V_ACTIVE)
#define VIDEO_BASEADDR0 DDR_BASEADDR + 0x2000000
#define VIDEO_BASEADDR1 DDR_BASEADDR + 0x3000000
#define VIDEO_BASEADDR2 DDR_BASEADDR + 0x4000000

u32 *BufferPtr[3];

//unsigned int srcBuffer = (XPAR_PS7_DDR_0_S_AXI_BASEADDR  + 0x1000000);
//int run_triple_frame_buffer(XAxiVdma* InstancePtr, int DeviceId, int hsize,
//		int vsize, int buf_base_addr, int number_frame_count,
//		int enable_frm_cnt_intr);
//////////////////////////////////////////////////////////
static FATFS fatfs;
XAxiDma AxiDma;
XAxiVdma vdma;
XAxiVdma_DmaSetup vdmaConfigSet;
XCnn HlsCnn;
XCnn_Config *CnnPtr;
u8 frameBuf[3][640*480*4];
u8 *pFrames[3];

#define FILE "dog.bin"
#define FILE_WEIGHT "weights.bin"

int SD_Init()
{
    FRESULT rc;
    rc = f_mount(&fatfs,"",0);
    if(rc)
    {
        xil_printf("ERROR: f_mount returned %d\r\n",rc);
        return XST_FAILURE;
    }
    return XST_SUCCESS;
}

int SD_Transfer_read(char *FileName,u32 DestinationAddress,u32 ByteLength)
{
    FIL fil;
    FRESULT rc;
    UINT br;

    rc=f_open(&fil,FileName,FA_READ);
    if(rc)
    {
        xil_printf("ERROR:f_open returned %d\r\n",rc);
        return XST_FAILURE;
    }
    rc = f_lseek(&fil,0);
    if(rc)
    {
        xil_printf("ERROR:f_lseek returned %d\r\n",rc);
        return XST_FAILURE;
    }
    rc = f_read(&fil,(void*)DestinationAddress,ByteLength,&br);
    if(rc)
    {
        xil_printf("ERROR:f_read returned %d\r\n",rc);
        return XST_FAILURE;
    }
    rc = f_close(&fil);
    if(rc)
    {
        xil_printf("ERROR:f_close returned %d\r\n",rc);
        return XST_FAILURE;
    }
    return XST_SUCCESS;
}

int init_dma(){
	XAxiDma_Config *CfgPtr;
	int status;

	CfgPtr = XAxiDma_LookupConfig( (XPAR_AXIDMA_0_DEVICE_ID) );
	if(!CfgPtr){
		print("Error looking for AXI DMA config\n\r");
		return XST_FAILURE;
	}
	status = XAxiDma_CfgInitialize(&AxiDma,CfgPtr);
	if(status != XST_SUCCESS){
		print("Error initializing DMA\n\r");
		return XST_FAILURE;
	}
	//check for scatter gather mode
	if(XAxiDma_HasSg(&AxiDma)){
		print("Error DMA configured in SG mode\n\r");
		return XST_FAILURE;
	}
	//Disable interrupts, we use polling mode
	XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
	XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);

	// Reset DMA
	XAxiDma_Reset(&AxiDma);
	while (!XAxiDma_ResetIsDone(&AxiDma)) {}
	return XST_SUCCESS;
}

int dma_trans(UINTPTR TxBufferPtr, UINTPTR RxBufferPtr, int TxSize, int RxSize)
{
	int Status;

	if(RxSize != 0)
	{
		Status = XAxiDma_SimpleTransfer(&AxiDma,(UINTPTR) RxBufferPtr,
			RxSize, XAXIDMA_DEVICE_TO_DMA);

		if (Status != XST_SUCCESS) {
			xil_printf("RX Transfer faild%d\r\n",Status);
			return XST_FAILURE;
		}
	}

	if(TxSize != 0)
	{
		Status = XAxiDma_SimpleTransfer(&AxiDma,(UINTPTR) TxBufferPtr,
				TxSize, XAXIDMA_DMA_TO_DEVICE);
		if (Status != XST_SUCCESS) {
			xil_printf("TX Transfer faild%d\r\n",Status);
			return XST_FAILURE;
		}
	}

	return XST_SUCCESS;
}

void DemoInitialize()
{
	//VDMA configurateAXI VDMA0
	/****************WRETE DATA TO DDR**********************/
	Xil_Out32((VDMA_BASEADDR + 0x0AC), frameBuf[1]);	// start address
	Xil_Out32((VDMA_BASEADDR + 0x0B0), frameBuf[1]);	// start address
	Xil_Out32((VDMA_BASEADDR + 0x0B4), frameBuf[1]);	// start address
	Xil_Out32((VDMA_BASEADDR + 0x0A8), (H_STRIDE*4));		// h offset (640 * 4) bytes
	Xil_Out32((VDMA_BASEADDR + 0x0A4), (H_ACTIVE*4));		// h size (640 * 4) bytes
	Xil_Out32((VDMA_BASEADDR + 0x030), 0x00000003);		// enable circular mode
	Xil_Out32((VDMA_BASEADDR + 0x0A0), V_ACTIVE);		// v size (480)
	/*****************READ DATA FROM DDR**********************/
	Xil_Out32((VDMA_BASEADDR + 0x05c), frameBuf[2]); 	// start address
	Xil_Out32((VDMA_BASEADDR + 0x060), frameBuf[2]); 	// start address
	Xil_Out32((VDMA_BASEADDR + 0x064), frameBuf[2]); 	// start address
	Xil_Out32((VDMA_BASEADDR + 0x058), (H_STRIDE*4)); 		// h offset (640 * 4) bytes
	Xil_Out32((VDMA_BASEADDR + 0x054), (H_ACTIVE*4)); 		// h size (640 * 4) bytes
	Xil_Out32((VDMA_BASEADDR + 0x000), 0x00000003); 		// enable circular mode
	Xil_Out32((VDMA_BASEADDR + 0x050), V_ACTIVE); 			// v size (480)
	xil_printf(" End \n\r");

	xil_printf("vdma setting over\r\n");

}

int cnn_run(int in_channel,int in_width,int pooling) {
	// Look Up the device configuration
	CnnPtr = XCnn_LookupConfig(XPAR_XCNN_0_DEVICE_ID);
	if (!CnnPtr) {
		print("ERROR: Lookup of accelerator configuration failed.\n\r");
		return XST_FAILURE;
	}
	int status;
	// Initialize the Device
	status = XCnn_CfgInitialize(&HlsCnn, CnnPtr);
	if (status != XST_SUCCESS) {
		print("ERROR: Could not initialize accelerator.\n\r");
		return XST_FAILURE;
	}

1+9*in_channel+in_width*in_width*in_channel);
	// Start the device and read the results
	XCnn_Start(&HlsCnn);
	return XST_SUCCESS;
}

float sigmoid(float x){
	float out = 1.0/(1+expf(-x));
	return out;
}

int max(int a,int b){
	if(a<b)
		return b;
	else
		return a;
}

int min(int a,int b){
	if(a>b)
		return b;
	else
		return a;
}

float iou(int left_a,int right_a,int top_a,int bottom_a,
		int left_b,int right_b,int top_b,int bottom_b){
	int xA = max(left_a, left_b);
	int yA = max(top_a, top_b);
	int xB = min(right_a, right_b);
	int yB = min(bottom_a, bottom_b);

	//% Compute the area of intersection
	int intersection_area = (xB - xA + 1) * (yB - yA + 1);

	//% Compute the area of both rectangles
	int boxA_area = (right_a - left_a + 1) * (bottom_a - top_a + 1);
	int boxB_area = (right_b - left_b + 1) * (bottom_b - top_b + 1);

	//% Compute the IOU
	float iou_out = (intersection_area*1.0 / ((boxA_area + boxB_area - intersection_area)*1.0));
	return iou_out;
}

int main()
{
    init_platform();

	int i,j,k;
    xil_printf("Hello World\n\r");

    int rc;

    u32 len = PHOTO_PIXEL;

    rc = SD_Init();

    float* weightsFromFile = (float*)malloc(sizeof(float) * 63434868/4);
    xil_printf("point to weights = %d\n",(int)weightsFromFile);
	rc = SD_Transfer_read(FILE_WEIGHT,(u32)weightsFromFile,63434868);

    if(XST_SUCCESS != rc)
        xil_printf("fail to read SD Card~\r\n");
    else
        xil_printf("success to read SD Card~\r\n");
    xil_printf("SD Card Write and Read Success~\r\n");

	//initiali the vdma and show img
    DemoInitialize();

    for(int i=0;i<10000000;i++);

    while (XUartPs_IsReceiveData(XPAR_PS7_UART_1_BASEADDR))
    {
    	XUartPs_ReadReg(XPAR_PS7_UART_1_BASEADDR, XUARTPS_FIFO_OFFSET);
    }

    int reflush_uart = 0;
    char input_data;
while(1)
{
	LOOP:
	Xil_DCacheInvalidateRange((UINTPTR)frameBuf[1], 640*480*4);
	for (int i=0;i<640*480*4;i++){
		frameBuf[0][i] = frameBuf[1][i];
		//printf("%d = %d\n",i,*((u8*) (VIDEO_BASEADDR0+i)));
	}
	Xil_DCacheFlushRange((u32)frameBuf[0],640*480*4);


	if(XUartPs_IsReceiveData(XPAR_PS7_UART_1_BASEADDR))
	{
		input_data =  XUartPs_ReadReg(XPAR_PS7_UART_1_BASEADDR, XUARTPS_FIFO_OFFSET);
		xil_printf("%c\n", input_data);
	}

	if(input_data == 's')
	{
		while (XUartPs_IsReceiveData(XPAR_PS7_UART_1_BASEADDR))
		{
			XUartPs_ReadReg(XPAR_PS7_UART_1_BASEADDR, XUARTPS_FIFO_OFFSET);
		}
		reflush_uart = 1;
		XTime testBegin, testEnd;
		XTime time_1,time_2,time_3,time_4,time_5,time_6,time_7,time_8,time_9;
		XTime_GetTime(&testBegin);

		//CNN START

		float weights[1+1024*9];
		float* lay_1_in_0 = (float*)malloc(sizeof(float) * 208*208*3);
		float* lay_1_in_1 = (float*)malloc(sizeof(float) * 208*208*3);
		float* lay_1_in_2 = (float*)malloc(sizeof(float) * 208*208*3);
		float* lay_1_in_3 = (float*)malloc(sizeof(float) * 208*208*3);
		float* lay_1_out_0_0 = (float*)malloc(sizeof(float) * 104*104);
		float* lay_1_out_1_0 = (float*)malloc(sizeof(float) * 104*104);
		float* lay_1_out_2_0 = (float*)malloc(sizeof(float) * 104*104);
		float* lay_1_out_3_0 = (float*)malloc(sizeof(float) * 104*104);
		float* lay_1_out[16];
		for(int i=0;i<16;i++)
			lay_1_out[i] = (float*)malloc(sizeof(float) * 208*208);

		for (k=0;k<3;k++)
			for (j=0;j<208;j++)
				for (i=0;i<208;i++)
					lay_1_in_0[k*208*208 + j*208 +i] = (float)frameBuf[0][((j+32)*640 + (i+112))*4 + k+1] /255;
		for (k=0;k<3;k++)
			for (j=0;j<208;j++)
				for (i=0;i<208;i++)
					lay_1_in_1[k*208*208 + j*208 +i] = (float)frameBuf[0][((j+32)*640 + (i+112+208))*4 + k+1] /255;
		for (k=0;k<3;k++)
			for (j=0;j<208;j++)
				for (i=0;i<208;i++)
					lay_1_in_2[k*208*208 + j*208 +i] = (float)frameBuf[0][((j+32+208)*640 + (i+112))*4 + k+1] /255;
		for (k=0;k<3;k++)
			for (j=0;j<208;j++)
				for (i=0;i<208;i++)
					lay_1_in_3[k*208*208 + j*208 +i] = (float)frameBuf[0][((j+32+208)*640 + (i+112+208))*4 + k+1] /255;
		
		printf("lay_1_in_ready\n");

		int weights_offset=0;
		//lay_in_pro
		for(int o_ch=0;o_ch<16;o_ch++)
		{

			init_dma();
			cnn_run(3,208,1);
			dma_trans(0, (UINTPTR)lay_1_out_0_0, 0, 104*104*4);
			dma_trans((UINTPTR)&weightsFromFile[weights_offset], 0, 28*4, 0);
			while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {/* Wait */}
			dma_trans((UINTPTR)lay_1_in_0, 0, (208*208*3)*4, 0);
			while(!XCnn_IsDone(&HlsCnn)){};
			Xil_DCacheInvalidateRange((UINTPTR)lay_1_out_0_0, 104*104*4);

			init_dma();
			cnn_run(3,208,1);
			dma_trans(0, (UINTPTR)lay_1_out_1_0, 0, 104*104*4);
			dma_trans((UINTPTR)&weightsFromFile[weights_offset], 0, 28*4, 0);
			while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {/* Wait */}
			dma_trans((UINTPTR)lay_1_in_1, 0, (208*208*3)*4, 0);
			while(!XCnn_IsDone(&HlsCnn)){};
			Xil_DCacheInvalidateRange((UINTPTR)lay_1_out_1_0, 104*104*4);

			init_dma();
			cnn_run(3,208,1);
			dma_trans(0, (UINTPTR)lay_1_out_2_0, 0, 104*104*4);
			dma_trans((UINTPTR)&weightsFromFile[weights_offset], 0, 28*4, 0);
			while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {/* Wait */}
			dma_trans((UINTPTR)lay_1_in_2, 0, (208*208*3)*4, 0);
			while(!XCnn_IsDone(&HlsCnn)){};
			Xil_DCacheInvalidateRange((UINTPTR)lay_1_out_2_0, 104*104*4);

			init_dma();
			cnn_run(3,208,1);
			dma_trans(0, (UINTPTR)lay_1_out_3_0, 0, 104*104*4);
			dma_trans((UINTPTR)&weightsFromFile[weights_offset], 0, 28*4, 0);
			while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {/* Wait */}
			dma_trans((UINTPTR)lay_1_in_3, 0, (208*208*3)*4, 0);
			while(!XCnn_IsDone(&HlsCnn)){};
			Xil_DCacheInvalidateRange((UINTPTR)lay_1_out_3_0, 104*104*4);
			weights_offset += 28;

			for(int i=0;i<104;i++)
				for(int j=0;j<104;j++)
					lay_1_out[o_ch][i*208+j] = lay_1_out_0_0[i*104+j];
			for(int i=0;i<104;i++)
				for(int j=0;j<104;j++)
					lay_1_out[o_ch][i*208+j+104] = lay_1_out_1_0[i*104+j];
			for(int i=0;i<104;i++)
				for(int j=0;j<104;j++)
					lay_1_out[o_ch][(i+104)*208+j] = lay_1_out_2_0[i*104+j];
			for(int i=0;i<104;i++)
				for(int j=0;j<104;j++)
					lay_1_out[o_ch][(i+104)*208+j+104] = lay_1_out_3_0[i*104+j];
			Xil_DCacheFlushRange((u32)lay_1_out[o_ch], (208*208*4));
		}
		printf("cnn ok\n");
		//free(lay_1_in_0);free(lay_1_in_1);free(lay_1_in_2);free(lay_1_in_3);
		//free(lay_1_out_0_0);free(lay_1_out_1_0);free(lay_1_out_2_0);free(lay_1_out_3_0);


		printf("lay1_out complete\r\n");
		XTime_GetTime(&time_1);
		double diff = (double)(time_1 - testBegin) / COUNTS_PER_SECOND;
		printf("Elapsed time is %.5f seconds\n", diff);

		int in_channel ;
		int out_channel;
		int in_width ;
		int out_width ;
		int pool;
		int weight_num;

		//lay2
		float* lay_2_out[32];
		for(int i=0;i<32;i++)
			lay_2_out[i] = (float*)malloc(sizeof(float) * 104*104);
		in_channel = 16;
		out_channel = 32;
		in_width = 208;
		out_width = 104;
		pool = 1;
		weight_num = (1+in_channel*9);
		for(int o_ch=0;o_ch<out_channel;o_ch++)
		{
			init_dma();
			cnn_run(in_channel,in_width,pool);
			dma_trans(0, (UINTPTR)lay_2_out[o_ch], 0, out_width*out_width*4);
			dma_trans((UINTPTR)&weightsFromFile[weights_offset], 0, weight_num*4, 0);
			while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {}
			for(int in_ch=0;in_ch<in_channel;in_ch++){
				dma_trans((UINTPTR)lay_1_out[in_ch], 0, (in_width*in_width)*4, 0);
				if(in_ch<in_channel-1)
					while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {}
			}
			while(!XCnn_IsDone(&HlsCnn)){};
			weights_offset += weight_num;
		}
		for(int i=0;i<in_channel;i++)
			free(lay_1_out[i]);
		xil_printf("lay_2_out complete\r\n");
		XTime_GetTime(&time_2);
		diff = (double)(time_2 - time_1) / COUNTS_PER_SECOND;
		printf("Elapsed time is %.5f seconds\n", diff);

		//lay3
		float* lay_3_out[64];
		in_channel = 32;
		out_channel = 64;
		in_width = 104;
		out_width = 52;
		for(int i=0;i<out_channel;i++)
			lay_3_out[i] = (float*)malloc(sizeof(float) * out_width*out_width);
		pool = 1;
		weight_num = (1+in_channel*9);
		for(int o_ch=0;o_ch<out_channel;o_ch++)
		{
			init_dma();
			cnn_run(in_channel,in_width,pool);
			dma_trans(0, (UINTPTR)lay_3_out[o_ch], 0, out_width*out_width*4);
			dma_trans((UINTPTR)&weightsFromFile[weights_offset], 0, weight_num*4, 0);
			while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {}
			for(int in_ch=0;in_ch<in_channel;in_ch++){
				dma_trans((UINTPTR)lay_2_out[in_ch], 0, (in_width*in_width)*4, 0);
				if(in_ch<in_channel-1)
					while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {}
			}
			while(!XCnn_IsDone(&HlsCnn)){};
			weights_offset += weight_num;
		}
		for(int i=0;i<in_channel;i++)
			free(lay_2_out[i]);
		xil_printf("lay_3_out complete\r\n");
		XTime_GetTime(&time_3);
		diff = (double)(time_3 - time_2) / COUNTS_PER_SECOND;
		printf("Elapsed time is %.5f seconds\n", diff);

		//lay4
		float* lay_4_out[128];
		in_channel = 64;
		out_channel = 128;
		in_width = 52;
		out_width = 26;
		for(int i=0;i<out_channel;i++)
			lay_4_out[i] = (float*)malloc(sizeof(float) * out_width*out_width);
		pool = 1;
		weight_num = (1+in_channel*9);
		for(int o_ch=0;o_ch<out_channel;o_ch++)
		{
			init_dma();
			cnn_run(in_channel,in_width,pool);
			dma_trans(0, (UINTPTR)lay_4_out[o_ch], 0, out_width*out_width*4);
			dma_trans((UINTPTR)&weightsFromFile[weights_offset], 0, weight_num*4, 0);
			while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {}
			for(int in_ch=0;in_ch<in_channel;in_ch++){
				dma_trans((UINTPTR)lay_3_out[in_ch], 0, (in_width*in_width)*4, 0);
				if(in_ch<in_channel-1)
					while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {}
			}
			while(!XCnn_IsDone(&HlsCnn)){};
			weights_offset += weight_num;
		}
		for(int i=0;i<in_channel;i++)
			free(lay_3_out[i]);
		xil_printf("lay_4_out complete\r\n");
		XTime_GetTime(&time_4);
		diff = (double)(time_4 - time_3) / COUNTS_PER_SECOND;
		printf("Elapsed time is %.5f seconds\n", diff);

		//lay5
		float* lay_5_out[256];
		in_channel = 128;
		out_channel = 256;
		in_width = 26;
		out_width = 13;
		for(int i=0;i<out_channel;i++)
			lay_5_out[i] = (float*)malloc(sizeof(float) * out_width*out_width);
		pool = 1;
		weight_num = (1+in_channel*9);
		for(int o_ch=0;o_ch<out_channel;o_ch++)
		{
			init_dma();
			cnn_run(in_channel,in_width,pool);
			dma_trans(0, (UINTPTR)lay_5_out[o_ch], 0, out_width*out_width*4);
			dma_trans((UINTPTR)&weightsFromFile[weights_offset], 0, weight_num*4, 0);
			while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {}
			for(int in_ch=0;in_ch<in_channel;in_ch++){
				dma_trans((UINTPTR)lay_4_out[in_ch], 0, (in_width*in_width)*4, 0);
				if(in_ch<in_channel-1)
					while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {}
			}
			while(!XCnn_IsDone(&HlsCnn)){};
			weights_offset += weight_num;
		}
		for(int i=0;i<in_channel;i++)
			free(lay_4_out[i]);
		xil_printf("lay_5_out complete\r\n");
		XTime_GetTime(&time_5);
		diff = (double)(time_5 - time_4) / COUNTS_PER_SECOND;
		printf("Elapsed time is %.5f seconds\n", diff);

		//lay6
		float* lay_6_out[512];
		in_channel = 256;
		out_channel = 512;
		in_width = 13;
		out_width = 13;
		for(int i=0;i<out_channel;i++)
			lay_6_out[i] = (float*)malloc(sizeof(float) * out_width*out_width);
		pool = 2;
		weight_num = (1+in_channel*9);
		for(int o_ch=0;o_ch<out_channel;o_ch++)
		{
			init_dma();
			cnn_run(in_channel,in_width,pool);
			dma_trans(0, (UINTPTR)lay_6_out[o_ch], 0, out_width*out_width*4);
			dma_trans((UINTPTR)&weightsFromFile[weights_offset], 0, weight_num*4, 0);
			while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {}
			for(int in_ch=0;in_ch<in_channel;in_ch++){
				dma_trans((UINTPTR)lay_5_out[in_ch], 0, (in_width*in_width)*4, 0);
				if(in_ch<in_channel-1)
					while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {}
			}
			while(!XCnn_IsDone(&HlsCnn)){};
			weights_offset += weight_num;
		}
		for(int i=0;i<in_channel;i++)
			free(lay_5_out[i]);
		xil_printf("lay_6_out complete\r\n");
		XTime_GetTime(&time_6);
		diff = (double)(time_6 - time_5) / COUNTS_PER_SECOND;
		printf("Elapsed time is %.5f seconds\n", diff);

		//lay7
		float* lay_7_out[1024];
		in_channel = 512;
		out_channel = 1024;
		in_width = 13;
		out_width = 13;
		for(int i=0;i<out_channel;i++)
			lay_7_out[i] = (float*)malloc(sizeof(float) * out_width*out_width);
		pool = 0;
		weight_num = (1+in_channel*9);
		for(int o_ch=0;o_ch<out_channel;o_ch++)
		{
			init_dma();
			cnn_run(in_channel,in_width,pool);
			dma_trans(0, (UINTPTR)lay_7_out[o_ch], 0, out_width*out_width*4);
			dma_trans((UINTPTR)&weightsFromFile[weights_offset], 0, weight_num*4, 0);
			while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {}
			for(int in_ch=0;in_ch<in_channel;in_ch++){
				dma_trans((UINTPTR)lay_6_out[in_ch], 0, (in_width*in_width)*4, 0);
				if(in_ch<in_channel-1)
					while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {}
			}
			while(!XCnn_IsDone(&HlsCnn)){};
			weights_offset += weight_num;
		}
		for(int i=0;i<in_channel;i++)
			free(lay_6_out[i]);
		xil_printf("lay_7_out complete\r\n");
		XTime_GetTime(&time_7);
		diff = (double)(time_7 - time_6) / COUNTS_PER_SECOND;
		printf("Elapsed time is %.5f seconds\n", diff);

		//lay8
		float* lay_8_out[1024];
		in_channel = 1024;
		out_channel = 1024;
		in_width = 13;
		out_width = 13;
		for(int i=0;i<out_channel;i++)
			lay_8_out[i] = (float*)malloc(sizeof(float) * out_width*out_width);
		pool = 0;
		weight_num = (1+in_channel*9);
		for(int o_ch=0;o_ch<out_channel;o_ch++)
		{
			init_dma();
			cnn_run(in_channel,in_width,pool);
			dma_trans(0, (UINTPTR)lay_8_out[o_ch], 0, out_width*out_width*4);
			dma_trans((UINTPTR)&weightsFromFile[weights_offset], 0, weight_num*4, 0);
			while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {}
			for(int in_ch=0;in_ch<in_channel;in_ch++){
				dma_trans((UINTPTR)lay_7_out[in_ch], 0, (in_width*in_width)*4, 0);
				if(in_ch<in_channel-1)
					while (XAxiDma_Busy(&AxiDma,XAXIDMA_DMA_TO_DEVICE)) {}
			}
			while(!XCnn_IsDone(&HlsCnn)){};
			Xil_DCacheInvalidateRange((UINTPTR)lay_8_out[o_ch], 13*13*4);
			weights_offset += weight_num;
		}
		for(int i=0;i<in_channel;i++)
			free(lay_7_out[i]);
		xil_printf("lay_8_out complete\r\n");
		XTime_GetTime(&time_8);
		diff = (double)(time_8 - time_7) / COUNTS_PER_SECOND;
		printf("Elapsed time is %.5f seconds\n", diff);

		//lay9
		float lay_9_out[13*13][125];
		in_channel = 1024;
		weight_num = (1+in_channel);
		for(int o_ch=0;o_ch<125;o_ch++)
		{
			for (int i=0;i<weight_num;i++)
				weights[i] = weightsFromFile[i+weights_offset];
			weights_offset += weight_num;

			for(int pixel=0;pixel<13*13;pixel++)
			{
				lay_9_out[pixel][o_ch] = weights[0];
				for(int in_ch=0;in_ch<in_channel;in_ch++)
					lay_9_out[pixel][o_ch] += lay_8_out[in_ch][pixel] * weights[1+in_ch];
			}

		}

		for(int i=0;i<in_channel;i++){
			free(lay_8_out[i]);}
		xil_printf("lay_9_out complete\r\n");
		XTime_GetTime(&time_9);
		diff = (double)(time_9 - time_8) / COUNTS_PER_SECOND;
		printf("Elapsed time is %.5f seconds\n", diff);

		XTime_GetTime(&testEnd);
		diff = (double)(testEnd - testBegin) / COUNTS_PER_SECOND;
		printf("Elapsed time is %.2f seconds\n", diff);

		//for(int i=0;i<25;i++){
		//	printf("output_%d = %x = %f\n",i,*(int*)&lay_9_out[0][i],lay_9_out[0][i]);
		//}


		const float anchors[10] = {1.08,1.19,  3.42,4.41,  6.63,11.38,  9.42,5.11,  16.62,10.52};

		int box_num = 0;
		int box_info_left[100];
		int box_info_right[100];
		int box_info_top[100];
		int box_info_bottom[100];
		float box_info_c[100];
		int box_info_class[100];
		for(int row=0;row<13;row++)
			for(int col=0;col<13;col++)
				for(int box=0;box<5;box++)
				{
					float tx = lay_9_out[row*13+col][box*25+0];
					float ty = lay_9_out[row*13+col][box*25+1];
					float tw = lay_9_out[row*13+col][box*25+2];
					float th = lay_9_out[row*13+col][box*25+3];
					float tc = lay_9_out[row*13+col][box*25+4];
					float center_x = (col-1 + sigmoid(tx)+1) * 32.0;
					float center_y = (row-1 + sigmoid(ty)+1) * 32.0;
					float roi_w = expf(tw) * anchors[2*box] * 32.0;
					float roi_h = expf(th) * anchors[2*box+1] * 32.0;
					float final_confidence = sigmoid(tc);
					int left   = (int)(center_x - (roi_w/2.0)+112);
					int right  = (int)(center_x + (roi_w/2.0)+112);
					int top    = (int)(center_y - (roi_h/2.0)+32);
					int bottom = (int)(center_y + (roi_h/2.0)+32);
					//if(left<=0 || right>=639 || top<=0 || bottom>=479 )
					//	continue;

					float softmax_a=0;
					float class_predictions[20];
					for(int i=0;i<20;i++)
						softmax_a += expf(lay_9_out[row*13+col][box*25+5+i]);
					for(int i=0;i<20;i++)
						class_predictions[i] = expf(lay_9_out[row*13+col][box*25+5+i]) / softmax_a;
					int class=0;
					float max_class_c = class_predictions[0];
					for(int i=1;i<20;i++)
					{
						if(max_class_c<class_predictions[i]){
							max_class_c = class_predictions[i];
							class = i;
						}
					}

					if ((final_confidence * max_class_c) > 0.1 && !(left<=0 || right>=639 || top<=0 || bottom>=479 ))
					{
						//printf("max_class_c = %f\n",final_confidence *max_class_c);
						//printf("there is a box %d\n",box_num+1);
						box_info_left[box_num] = left;
						box_info_right[box_num] = right;
						box_info_top[box_num] = top;
						box_info_bottom[box_num] = bottom;
						box_info_c[box_num] = final_confidence * max_class_c;
						box_info_class[box_num] = class;
						box_num = box_num + 1;
					}
				}
		//printf("there box find out \n");
		for(int i=0;i<box_num-1;i++)
			for(int j=i;j<box_num;j++)
				if (box_info_c[i] < box_info_c[j]){
					int left_temp = box_info_left[i];
					box_info_left[i] = box_info_left[j];
					box_info_left[j] = left_temp;
					left_temp = box_info_right[i];
					box_info_right[i] = box_info_right[j];
					box_info_right[j] = left_temp;
					left_temp = box_info_top[i];
					box_info_top[i] = box_info_top[j];
					box_info_top[j] = left_temp;
					left_temp = box_info_bottom[i];
					box_info_bottom[i] = box_info_bottom[j];
					box_info_bottom[j] = left_temp;
					left_temp = box_info_class[i];
					box_info_class[i] = box_info_class[j];
					box_info_class[j] = left_temp;
					float c_temp = box_info_c[i];
					box_info_c[i] = box_info_c[j];
					box_info_c[j] = c_temp;
				}

		int nmsed_num = 0;
		int nms_info_left[20];
		int nms_info_right[20];
		int nms_info_top[20];
		int nms_info_bottom[20];
		int nms_info_class[20];
		float iou_threshold = 0.3;
		nms_info_left[0] = box_info_left[0];
		nms_info_right[0] = box_info_right[0];
		nms_info_top[0] = box_info_top[0];
		nms_info_bottom[0] = box_info_bottom[0];
		nms_info_class[0] = box_info_class[0];
		for (int i=1;i<box_num;i++){
			int to_delete = 0;
			for (int j=0;j<nmsed_num+1;j++){
				float iou_out = iou(box_info_left[i],box_info_right[i],box_info_top[i],box_info_bottom[i],
								nms_info_left[j],nms_info_right[j],nms_info_top[j],nms_info_bottom[j]);
				if (iou_out  > iou_threshold)
					to_delete = 1;
			}
			if (to_delete == 0){
				nmsed_num = nmsed_num + 1;
				nms_info_left[nmsed_num] = box_info_left[i];
				nms_info_right[nmsed_num] = box_info_right[i];
				nms_info_top[nmsed_num] = box_info_top[i];
				nms_info_bottom[nmsed_num] = box_info_bottom[i];
				nms_info_class[nmsed_num] = box_info_class[i];
			}
		}


		const u8 colors[20*3] = {254, 254, 254, 240, 212, 127,
					  226, 169, 0, 212, 127, 254,
					  198, 85, 127, 183, 42, 0,
					  169, 0, 254, 155, 42, 127,
					  141, 85, 0, 127, 254, 254,
					  113, 212, 127, 99, 169, 0,
					  85, 127, 254, 71, 85, 127,
					  56, 42, 0, 42, 0, 254,
					  28, 42, 127, 14, 85, 0,
					  0, 254, 254, 14, 212, 127};
		for(int i=0;i<nmsed_num+1;i++)
		{
			int left = nms_info_left[i];
			int right = nms_info_right[i];
			int top = nms_info_top[i];
			int bottom = nms_info_bottom[i];
			for(int j=top;j<=bottom;j++)
			{
				frameBuf[0][(j*640+left)*4+1] = (u8) colors[nms_info_class[i]*3];
				frameBuf[0][(j*640+left)*4+2] = (u8) colors[nms_info_class[i]*3+1];
				frameBuf[0][(j*640+left)*4+3] = (u8) colors[nms_info_class[i]*3+2];
				frameBuf[0][(j*640+right)*4+1] = (u8) colors[nms_info_class[i]*3];
				frameBuf[0][(j*640+right)*4+2] = (u8) colors[nms_info_class[i]*3+1];
				frameBuf[0][(j*640+right)*4+3] = (u8) colors[nms_info_class[i]*3+2];
			}
			for(int j=left;j<=right;j++)
			{
				frameBuf[0][(j+top*640)*4+1] = (u8) colors[nms_info_class[i]*3];
				frameBuf[0][(j+top*640)*4+2] = (u8) colors[nms_info_class[i]*3+1];
				frameBuf[0][(j+top*640)*4+3] = (u8) colors[nms_info_class[i]*3+2];
				frameBuf[0][(j+bottom*640)*4+1] = (u8) colors[nms_info_class[i]*3];
				frameBuf[0][(j+bottom*640)*4+2] = (u8) colors[nms_info_class[i]*3+1];
				frameBuf[0][(j+bottom*640)*4+3] = (u8) colors[nms_info_class[i]*3+2];
			}
			//printf("write box finish %d\n",i);
		}
		for(int i=0;i<640*480*4;i++)
			frameBuf[2][i] = frameBuf[0][i];
		Xil_DCacheFlushRange((u32)frameBuf[2], (640*480*4));
		while(!XUartPs_IsReceiveData(XPAR_PS7_UART_1_BASEADDR)){};
		while (XUartPs_IsReceiveData(XPAR_PS7_UART_1_BASEADDR))
		{
			XUartPs_ReadReg(XPAR_PS7_UART_1_BASEADDR, XUARTPS_FIFO_OFFSET);
		}
		if(!XUartPs_IsReceiveData(XPAR_PS7_UART_1_BASEADDR))
		{
			printf("no input\n");
			reflush_uart = 0;
			goto LOOP;
		}
		else
			printf("have input\n");

	}
	for(int i=0;i<640*480*4;i++)
		frameBuf[2][i] = frameBuf[0][i];

	//printf("next frame %d\n");

	Xil_DCacheFlushRange((u32)frameBuf[2], (640*480*4));

}
		//xil_printf("%d\r\n",photoFromFile[240*640*3]);
		//xil_printf("%d\r\n",lay_1_out[240*640*3]);


	while(1){};

    cleanup_platform();
    return 0;
}
