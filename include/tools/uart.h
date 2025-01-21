#pragma once

// 串口相关的头文件
#include <iostream>
#include <stdlib.h> /*标准函数库定义*/
#include <unistd.h> /*Unix 标准函数定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <fcntl.h>   /*文件控制定义*/
#include <termios.h> /*PPSIX 终端控制定义*/
#include <errno.h>   /*错误号定义*/
#include <string>
#include <chrono>
// 宏定义
#define FALSE -1
#define TRUE 0

void LError(const char *message);

template <typename T>
unsigned char crc8withTable(T *addr, int len, const unsigned char *crc8table)
{
    unsigned char crc = 0x00; // 初始 CRC 值
    unsigned char data;

    // 将结构体转换为字节指针，逐字节计算
    unsigned char *bytePtr = (unsigned char *)addr;

    // 逐字节计算 CRC
    for (int i = 0; i < len; i++)
    {

        data = bytePtr[i];
        if (data == 0xA5)
            continue;
        crc ^= data;
        crc = crc8table[crc];
    }
    return crc ^ 0x00;
}

struct ReceivePacket
{
    uint8_t header;       // 针头0xA5
    uint8_t detect_color; // 0: red, 1: blue
    float initial_speed;  // 弹丸速度
    float q[4];           // 四元数
    uint8_t crc = 0;
    uint8_t tail=0xCC; // 帧尾
    bool check();
} __attribute__((packed));

struct SendPacket
{
    uint8_t header = 0xA5;
    uint8_t tracking; // 是否锁定
    uint8_t id;       // 锁定的是几号
    float pitch;
    float yaw;    // pitch，yaw角度
    uint8_t crc;  // crc校验
    uint8_t tail; // 帧尾
    SendPacket(uint8_t tracking, uint8_t id, float pitch, float yaw);
    SendPacket(){};
} __attribute__((packed));

class comm_service
{
public:
    comm_service();
    ~comm_service();

    /*@brief 打开串口
    @param port 串口号*/
    int CommOpen(const char *port);

    /*@brief 关闭串口*/
    void CommClose();

    /*@brief 串口初始化
    @param baudware 串口速度
    @param flow_ctrl 数据流控制（0不使用，1硬件控制，2软件控制）
    @param databits 数据位
    @param stopbits 停止位
    @param parity 校验类型 （N无校验位，E偶校验，O奇校验）*/
    int CommInit(int baudrate, int flow_ctrl, int databits, int stopbits, int parity);

    /*@brief 接收数据*
    @param rec_buf 接收数据流
    @return -1接受失败
    */
    int CommRecv(ReceivePacket &rcv_buf);

    int CommRecv(uint8_t *rcv_buf,float *q);

    /*@brief 发送函数
    @param 串口发送数据
    @param data_len 发送的长度*/
    int CommSend(const SendPacket *send_data);

    /*@brief 获取文件描述*/
    int CommGetFD() const;

    /*@brief 清除缓冲区*/
    void CommFlush(int flush);

private:
    int commSet(int baudrate, int flow_ctrl, int databits, int stopbits, int parity);

    /*@param 串口是否开启*/
    bool commIsOpen() const;

    int m_fd; // 文件描述句柄
};

// 查表法实现crc8的校验计算,多项式为0x07
const unsigned char crc8table[256] = {
    0, 7, 14, 9, 28, 27, 18, 21, 56, 63, 54, 49, 36, 35, 42, 45,
    112, 119, 126, 121, 108, 107, 98, 101, 72, 79, 70, 65, 84, 83, 90, 93,
    224, 231, 238, 233, 252, 251, 242, 245, 216, 223, 214, 209, 196, 195, 202, 205,
    144, 151, 158, 153, 140, 139, 130, 133, 168, 175, 166, 161, 180, 179, 186, 189,
    199, 192, 201, 206, 219, 220, 213, 210, 255, 248, 241, 246, 227, 228, 237, 234,
    183, 176, 185, 190, 171, 172, 165, 162, 143, 136, 129, 134, 147, 148, 157, 154,
    39, 32, 41, 46, 59, 60, 53, 50, 31, 24, 17, 22, 3, 4, 13, 10,
    87, 80, 89, 94, 75, 76, 69, 66, 111, 104, 97, 102, 115, 116, 125, 122,
    137, 142, 135, 128, 149, 146, 155, 156, 177, 182, 191, 184, 173, 170, 163, 164,
    249, 254, 247, 240, 229, 226, 235, 236, 193, 198, 207, 200, 221, 218, 211, 212,
    105, 110, 103, 96, 117, 114, 123, 124, 81, 86, 95, 88, 77, 74, 67, 68,
    25, 30, 23, 16, 5, 2, 11, 12, 33, 38, 47, 40, 61, 58, 51, 52,
    78, 73, 64, 71, 82, 85, 92, 91, 118, 113, 120, 127, 106, 109, 100, 99,
    62, 57, 48, 55, 34, 37, 44, 43, 6, 1, 8, 15, 26, 29, 20, 19,
    174, 169, 160, 167, 178, 181, 188, 187, 150, 145, 152, 159, 138, 141, 132, 131,
    222, 217, 208, 215, 194, 197, 204, 203, 230, 225, 232, 239, 250, 253, 244, 243};
