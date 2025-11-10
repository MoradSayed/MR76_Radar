#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"
#include "zuds.h"

#define msleep(ms) usleep((ms) * 1000)
#define min(a, b) (((a) < (b)) ? (a) : (b))

#define ZUDS_ERROR_OK                   0    // 没错误
#define ZUDS_ERROR_TIMEOUT              1    // 响应超时
#define ZUDS_ERROR_TRANSPORT            2    // 发送数据失败
#define ZUDS_ERROR_CANCEL               3    // 取消请求
#define ZUDS_ERROR_SUPPRESS_RESPONSE    4    // 抑制响应
#define ZUDS_ERROR_OTHTER               100

#define USBCAN_I 3  // USBCAN-I/I+ 3
#define USBCAN_II 4 // USBCAN-II/II+ 4
#define MAX_CHANNELS 2
#define RX_WAIT_TIME 100
#define RX_BUFF_SIZE 1000

int DevType = USBCAN_II;    // 设备类型号
int DevIdx = 0;             // 设备索引号
int CHANNEL_INDEX = 0;           // 通道号

// 接收线程上下文
typedef struct
{
    int DevType; // 设备类型
    int DevIdx;  // 设备索引
    int index;   // 通道号
    int total;   // 接收总数
    int stop;    // 线程结束标志
    ZUDS_HANDLE uds_handle; // uds句柄
} RX_CTX;

typedef struct
{
    int channel; // 作通道号
    int Extend_Flag; //扩展帧标志
    int trans_version;  //版本
} CHANNEL_PARAM;

//发送回调函数
static uint32 transmit_callback(void *ctx, const ZUDS_FRAME *frame, uint count)
{
    CHANNEL_PARAM *pchn_param = (CHANNEL_PARAM *)ctx;
    uint result = 0;

    // can
    VCI_CAN_OBJ *zcan_data = (VCI_CAN_OBJ *)malloc(sizeof(VCI_CAN_OBJ) * count);
    if (zcan_data == NULL)
    {
    	return 1; // 内存分配失败处理
    }

    memset(zcan_data, 0, sizeof(VCI_CAN_OBJ) * count);

    // 填充消息数据
    for (int i = 0; i < count; i++)
    {
        zcan_data[i].ID = frame[i].id;
        if (pchn_param->Extend_Flag)
        {
            zcan_data[i].ExternFlag = 1;
        }
        zcan_data[i].DataLen = frame[i].data_len;
        zcan_data[i].TimeFlag = 1;
        zcan_data[i].SendType = 0;
        zcan_data[i].RemoteFlag = 0;
        for (int j = 0; j < frame[i].data_len; j++)
        {
            zcan_data[i].Data[j] = frame[i].data[j];
        }
    }

    // 调用发送函数
    result = VCI_Transmit(USBCAN_II, DevIdx, pchn_param->channel, zcan_data, count);
    free(zcan_data);
    printf("result=%d count=%d \r\n", result, count);
    if (result == count)
        return 0;
    else
        return 1;
}


void *rx_thread(void *data)
{
    RX_CTX *ctx = (RX_CTX *)data;
    int DevType = ctx->DevType;
    int DevIdx  = ctx->DevIdx;
    int chn_idx = ctx->index;
    ZUDS_HANDLE UdsHandle = ctx->uds_handle;

    VCI_CAN_OBJ can[RX_BUFF_SIZE]; // 接收结构体
    ZUDS_FRAME uds_frame = {0};    // uds帧结构体
    int cnt = 0;                   // 接收数量
    int count = 0;                 // 缓冲区报文数量
    
    while (!ctx->stop)
    {
        memset(can, 0, sizeof(can));
        count = VCI_GetReceiveNum(DevType, DevIdx, ctx->index); // 获取缓冲区报文数量
        if (count > 0)
        {
            // printf("缓冲区报文数量: %d\n", count);
            int rcount = VCI_Receive(DevType, DevIdx, ctx->index, can, RX_BUFF_SIZE, RX_WAIT_TIME); // 读报文
            for (int i = 0; i < rcount; i++)
            {
            	memset(&uds_frame, 0, sizeof(uds_frame));
             	uds_frame.id = can[i].ID & 0x1fffffff; // 传入真实ID
            	uds_frame.extend = can[i].ExternFlag;
            	uds_frame.data_len = can[i].DataLen;
            	memcpy(uds_frame.data, can[i].Data, uds_frame.data_len);
            	ZUDS_OnReceive(UdsHandle, &uds_frame);
            	
                printf("[%d] %d ID: 0x%x ", can[i].TimeStamp, chn_idx, can[i].ID & 0x1fffffff);
                printf("%s ", can[i].ExternFlag ? "扩展帧" : "标准帧");
                if(can[i].RemoteFlag == 0){
                    printf(" Data: ");
                    for (int j = 0; j < can[i].DataLen; j++)
                        printf("%02x ", can[i].Data[j]);
                }
                else
                    printf(" 远程帧");
                printf("\n");
            }
        }
        msleep(10);
    }
    pthread_exit(0);
}

RX_CTX rx_ctx[MAX_CHANNELS];        // 接收线程上下文
pthread_t rx_threads[MAX_CHANNELS]; // 接收线程


void construct_can_frame(VCI_CAN_OBJ *can, UINT id)
{
    memset(can, 0, sizeof(VCI_CAN_OBJ));
    can->ID = id;        // id
    can->SendType = 0;   // 发送方式 0-正常, 1-单次, 2-自发自收
    can->RemoteFlag = 0; // 0-数据帧 1-远程帧
    can->ExternFlag = 0; // 0-数据帧 1-远程帧
    can->DataLen = 8;    // 数据长度 1~8
    for (int i = 0; i < can->DataLen; i++)
        can->Data[i] = i;
}


static int can_start(int DevType, int DevIdx, int chnIdx)
{
	 // 波特率这里的十六进制数字，可以由“zcanpro 波特率计算器”计算得出
	int Baud = 0x1c00;         // 波特率 0x1400-1M(75%), 0x1c00-500k(87.5%), 0x1c01-250k(87.5%), 0x1c03-125k(87.5%)
	VCI_INIT_CONFIG config;
        config.AccCode = 0;
        config.AccMask = 0xffffffff;
        config.Reserved = 0;
        config.Filter = 1;
        config.Timing0 = Baud & 0xff; // 0x00
        config.Timing1 = Baud >> 8;   // 0x1c
        config.Mode = 0;

        if (!VCI_InitCAN(DevType, DevIdx, chnIdx, &config))
        {
            printf("InitCAN(%d) fail\n", chnIdx);
            return 0;
        }
        printf("InitCAN(%d) success\n", chnIdx);

        if (!VCI_StartCAN(DevType, DevIdx, chnIdx))
        {
            printf("StartCAN(%d) fail\n", chnIdx);
            return 0;
        }
        printf("StartCAN(%d) success\n", chnIdx);

        rx_ctx[chnIdx].DevType = DevType;
        rx_ctx[chnIdx].DevIdx = DevIdx;
        rx_ctx[chnIdx].index = chnIdx;
        rx_ctx[chnIdx].total = 0;
        rx_ctx[chnIdx].stop = 0;
        pthread_create(&rx_threads[chnIdx], NULL, rx_thread, &rx_ctx[chnIdx]); // 创建接收线程
}

void Set_param(ZUDS_HANDLE puds_handle, CHANNEL_PARAM *pchn_param)
{
    ZUDS_ISO15765_PARAM param_15765 = {0};
    
    memset(&param_15765, 0, sizeof(param_15765));
    param_15765.version = pchn_param->trans_version;
    param_15765.max_data_len = 8;
    param_15765.local_st_min = 0;
    param_15765.block_size = 8;
    param_15765.fill_byte = 0;
    param_15765.frame_type = pchn_param->Extend_Flag; // 0-标准帧，1-扩展帧
    param_15765.is_modify_ecu_st_min = 0;             // 是否修改 ECU 的最小发送时间间隔参数
    param_15765.remote_st_min = 0;
    param_15765.fc_timeout = 70;                 // 等待流控超时时间，单位ms
    param_15765.fill_mode = 1;                   // 数据长度填充模式，0-不填充，1-小于8字节填充到8，大于8字节就近填充，2-填充至最大字节
    ZUDS_SetParam(puds_handle, 1, &param_15765); // 第二个参数为1对应15765结构体

    ZUDS_SESSION_PARAM param_seesion = {0};
    memset(&param_seesion, 0, sizeof(param_seesion));
    param_seesion.timeout = 2000;
    param_seesion.enhanced_timeout = 5000;
    ZUDS_SetParam(puds_handle, 0, &param_seesion); // 第二个参数为0对应ZUDS_SESSION_PARAM结构体

    printf("set_param completed");
    return;
}



int main(int argc, char *argv[])
{
    // 打开设备
    if (!VCI_OpenDevice(DevType, DevIdx, 0))
    {
        printf("Open device fail\n");
        return 0;
    }
    printf("Open device success\n");

    for (int i = 0; i < MAX_CHANNELS; i++)
    {
    	can_start(DevType, DevIdx, i);
    }
    
    // uds初始化;
    CHANNEL_PARAM chn_param = {0};
    ZUDS_HANDLE uds_handle = ZUDS_Init(0);
    printf("uds_handle is %d\r\n", uds_handle);

    chn_param.channel = CHANNEL_INDEX;
    chn_param.Extend_Flag = 0;   // 0-标准帧，1-扩展帧
    chn_param.trans_version = 0; // 0-2004版本，1-2016版本

    ZUDS_SetTransmitHandler(uds_handle, &chn_param, transmit_callback); // 设置发送回调,将uds句柄与通道句柄绑定

    Set_param(uds_handle, &chn_param);
    
    sleep(1);
    printf("request test\r\n");
    ZUDS_REQUEST request = {0};
    memset(&request, 0, sizeof(request));
    request.src_addr = 0x731;
    request.dst_addr = 0x7b1;
    request.sid = 0x10;
    request.param_len = 0x1; // 参数长度

    request.param = (u_int8_t *)malloc(sizeof(u_int8_t) * request.param_len);
    *(request.param) = 0x03;
    
    ZUDS_RESPONSE response = {0};
    memset(&response, 0, sizeof(response));
    ZUDS_Request(uds_handle, &request, &response);
    free(request.param);

    if (response.status == 0)
    {
        if (response.type == 0)
        {
            // 消极响应
            printf("消极响应：0x%02X 服务号：0x%02X  消极码：0x%02X\n",
                   response.negative.neg_code,
                   response.negative.sid,
                   response.negative.error_code);
        }
        if (response.type == 1)
        {
            // 积极响应
            printf("积极响应,响应id:0x%02X,参数长度:%d,参数内容：",
                   response.positive.sid,
                   response.positive.param_len);

            // 打印参数内容
            for (int i = 0; i < response.positive.param_len; i++)
            {
                printf("0x%02X ", response.positive.param[i]);
            }
            printf("\n");
        }
    }
    else if (response.status == 1)
    {
        printf("响应超时\n");
    }
    else if (response.status == 2)
    {
        printf("传输失败，请检查链路层，或请确认流控帧是否回复\n");
    }
    else if (response.status == 3)
    {
        printf("取消请求\n");
    }
    else if (response.status == 4)
    {
        printf("抑制响应\n");
    }
    else if (response.status == 5)
    {
        printf("忙碌中\n");
    }
    else if (response.status == 6)
    {
        printf("请求参数错误\n");
    }
    
    // 阻塞等待
    getchar();
    for (int i = 0; i < MAX_CHANNELS; i++)
    {
        rx_ctx[i].stop = 1;
        pthread_join(rx_threads[i], NULL);
    }

    // 复位通道
    for (int i = 0; i < MAX_CHANNELS; i++)
    {
        if (!VCI_ResetCAN(DevType, DevIdx, i))
            printf("ResetCAN(%d) fail\n", i);
        else
            printf("ResetCAN(%d) success!\n", i);
    }

    // 关闭设备
    if (!VCI_CloseDevice(DevType, DevIdx))
        printf("CloseDevice fail\n");
    else
        printf("CloseDevice success\n");
    return 0;
}
