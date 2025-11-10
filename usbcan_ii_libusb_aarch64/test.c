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

#define msleep(ms) usleep((ms) * 1000)
#define min(a, b) (((a) < (b)) ? (a) : (b))

#define USBCAN_I 3  // USBCAN-I/I+ 3
#define USBCAN_II 4 // USBCAN-II/II+ 4
#define MAX_CHANNELS 2
#define RX_WAIT_TIME 100
#define RX_BUFF_SIZE 1000

// 接收线程上下文
typedef struct
{
    int DevType; // 设备类型
    int DevIdx;  // 设备索引
    int index;   // 通道号
    int total;   // 接收总数
    int stop;    // 线程结束标志
} RX_CTX;

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

void *rx_thread(void *data)
{
    RX_CTX *ctx = (RX_CTX *)data;
    int DevType = ctx->DevType;
    int DevIdx  = ctx->DevIdx;
    int chn_idx = ctx->index;

    VCI_CAN_OBJ can[RX_BUFF_SIZE]; // 接收结构体
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

int main(int argc, char *argv[])
{
    // 波特率这里的十六进制数字，可以由“zcanpro 波特率计算器”计算得出
    int Baud = 0x1c00;         // 波特率 0x1400-1M(75%), 0x1c00-500k(87.5%), 0x1c01-250k(87.5%), 0x1c03-125k(87.5%)
    int DevType = USBCAN_II;    // 设备类型号
    int DevIdx = 0;             // 设备索引号

    RX_CTX rx_ctx[MAX_CHANNELS];        // 接收线程上下文
    pthread_t rx_threads[MAX_CHANNELS]; // 接收线程

    // 打开设备
    if (!VCI_OpenDevice(DevType, DevIdx, 0))
    {
        printf("Open device fail\n");
        return 0;
    }
    printf("Open device success\n");

    // 初始化，启动通道
    for (int i = 0; i < MAX_CHANNELS; i++)
    {
        VCI_INIT_CONFIG config;
        config.AccCode = 0;
        config.AccMask = 0xffffffff;
        config.Reserved = 0;
        config.Filter = 1;
        config.Timing0 = Baud & 0xff; // 0x00
        config.Timing1 = Baud >> 8;   // 0x1c
        config.Mode = 0;

        if (!VCI_InitCAN(DevType, DevIdx, i, &config))
        {
            printf("InitCAN(%d) fail\n", i);
            return 0;
        }
        printf("InitCAN(%d) success\n", i);

        if (!VCI_StartCAN(DevType, DevIdx, i))
        {
            printf("StartCAN(%d) fail\n", i);
            return 0;
        }
        printf("StartCAN(%d) success\n", i);

        rx_ctx[i].DevType = DevType;
        rx_ctx[i].DevIdx = DevIdx;
        rx_ctx[i].index = i;
        rx_ctx[i].total = 0;
        rx_ctx[i].stop = 0;
        pthread_create(&rx_threads[i], NULL, rx_thread, &rx_ctx[i]); // 创建接收线程
    }

    // 测试发送
    const int transmit_num = 10;       // 发送数量
    VCI_CAN_OBJ can_data[transmit_num]; // 帧结构体
    memset(can_data, 0, sizeof(can_data));
    for (int i = 0; i < transmit_num; ++i)
        construct_can_frame(&can_data[i], i);

    int test_num = VCI_Transmit(DevType, DevIdx, 0, can_data, transmit_num);
    printf("Transmit return = %d\n", test_num);

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
