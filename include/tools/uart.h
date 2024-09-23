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
// 宏定义
#define FALSE -1
#define TRUE 0

void LError(const char *message);

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
    @param data_len 接收最大长度*/
    int CommRecv(char *rcv_buf, int data_len);

    /*@brief 发送函数
    @param 串口发送数据
    @param data_len 发送的长度*/
    int CommSend(char *send_buf, int data_len);

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