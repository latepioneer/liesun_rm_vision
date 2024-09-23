#include "uart.h"

void LError(const char *src)
{
    std::cout << src << std::endl;
}

comm_service::comm_service() : m_fd(-1) {}

comm_service::~comm_service() {}

int comm_service::CommOpen(const char *port)
{

    if (commIsOpen())
        return 1;

    m_fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (m_fd < 0)
    {
        LError("Can't Open Serial Port.");
        return -1;
    }
    // 恢复串口为阻塞状态
    if (fcntl(m_fd, F_SETFL, 0) < 0)
    {
        LError("fcntl failed.");
        return -1;
    }
    // 测试是否为终端设备
    if (0 == isatty(STDIN_FILENO))
    {
        LError("standard input is not a terminal device.");
        return -1;
    }
    return 1;
}

void comm_service::CommClose()
{
    if (!commIsOpen())
        return;
    close(m_fd);
}

int comm_service::commSet(int baudrate, int flow_ctrl, int databits, int stopbits, int parity)
{
    if (!commIsOpen())
        return -1;

    int i;
    int status;
    int speed_arr[] = {B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int name_arr[] = {115200, 19200, 9600, 4800, 2400, 1200, 300};

    struct termios options;

    /*  tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，
        该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.  */
    if (tcgetattr(m_fd, &options) != 0)
    {
        LError("commSet tcgetattr error.");
        return (FALSE);
    }
    bool isStandard = false;
    // 设置串口输入波特率和输出波特率
    for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
    {
        if (baudrate == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
            isStandard = true;
            break;
        }
    }

    // if (!isStandard)
    // {
    //     // 非标准波特率
    //     int res = serial_set_speci_baud(options, baudrate);
    //     if (res != 0)
    //     {
    //         LError("commSet serial_set_speci_baud error.");
    //         return (FALSE);
    //     }
    // }

    // 修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    // 修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    // 设置数据流控制
    switch (flow_ctrl)
    {
    case 0: // 不使用流控制
        options.c_cflag &= ~CRTSCTS;
        break;
    case 1: // 使用硬件流控制
        options.c_cflag |= CRTSCTS;
        break;
    case 2: // 使用软件流控制
        options.c_cflag |= IXON | IXOFF | IXANY;
        break;
    }

    // 设置数据位,屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 5:
        options.c_cflag |= CS5;
        break;
    case 6:
        options.c_cflag |= CS6;
        break;
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        LError("Unsupported data size.");
        return (FALSE);
    }

    // 设置校验位
    switch (parity)
    {
    case 'n':
    case 'N': // 无奇偶校验位。
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        // options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
        break;
    case 'o':
    case 'O': // 设置为奇校验
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E': // 设置为偶校验
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
        break;
    case 's':
    case 'S': // 设置为空格
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        LError("Unsupported parity.");
        return (FALSE);
    }

    // 解决二进制传输时，数据遇到0x0d , 0x11,0x13 会被丢掉的问题   J.  1027
    options.c_iflag &= ~(BRKINT | ICRNL | ISTRIP | IXON);

    // 设置停止位
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        LError("Unsupported stop bits.");
        return (FALSE);
    }

    // 修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // options.c_lflag &= ~(ISIG | ICANON);

    // 设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1;  /* 读取字符的最少个数为1 */

    // 如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(m_fd, TCIFLUSH);

    // 激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(m_fd, TCSANOW, &options) != 0)
    {
        LError("com set error!");
        return (FALSE);
    }
    return (TRUE);
}

int comm_service::CommInit(int speed, int flow_ctrl, int databits, int stopbits, int parity)
{
    if (!commIsOpen())
        return -1;
    // 设置串口数据帧格式
    return commSet(speed, flow_ctrl, databits, stopbits, parity);
}

int comm_service::CommRecv(char *rcv_buf, int max_len)
{
    if (!commIsOpen())
        return -1; // 如果串口未打开，则返回-1

    int total_len = 0; // 已接收的数据总长度
    int len = 0;       // 每次读取的数据长度
    unsigned char byte;

    while (total_len < max_len)
    {
        len = read(m_fd, &byte, 1); // 每次读取1个字节
        if (len > 0)
        {
            rcv_buf[total_len++] = byte; // 将读取的数据存入缓冲区
            if (byte == 0xCC)
            {
                // 如果检测到帧尾，则停止接收
                break;
            }
        }
        else if (len < 0)
        {
            return -1; // 读取错误，返回-1
        }
    }

    if (total_len >= max_len)
    {
        return -1; // 如果接收数据超出缓冲区大小，返回-1
    }

    return total_len; // 返回接收到的总字节数
}

int comm_service::CommSend(char *send_buf, int data_len)
{
    if (!commIsOpen())
        return -1;

    CommFlush(TCIFLUSH);
    int len = 0;
    len = write(m_fd, send_buf, data_len);
    if (len == data_len)
        return len;
    else
    {
        tcflush(m_fd, TCOFLUSH);
        return FALSE;
    }
}

void comm_service::CommFlush(int flush)
{
    if (!commIsOpen())
        return;

    tcflush(m_fd, flush);
}

int comm_service::CommGetFD() const
{
    return m_fd;
}

bool comm_service::commIsOpen() const
{
    return m_fd < 0 ? false : true;
}
