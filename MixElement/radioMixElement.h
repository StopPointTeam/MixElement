#ifndef RADIOMIXELEMENT_H
#define RADIOMIXELEMENT_H


#include <NeoHWSerial.h>

//注释以关闭调试功能
#define RADIO_DEBUG

#ifdef RADIO_DEBUG
#define NeoSerialDebug NeoSerial3
#endif

//默认与 HC-12 连接串口
#define RADIO_SERIAL_NUM NeoSerial2
//与 HC-12 串口通信波特率
#define RADIO_BAUD_RATE 9600

//通信包大小
#define FULL_MESSAGE_SIZE 50
//通信包最大有效信息大小
#define MAX_REAL_MESSAGE_SIZE 39
//通信包前段空字符填充长度
#define BLANK_CHAR_LENGTH 4
//通信包校验段字符串长度
#define CHECK_STR_LENGTH 2
//通信包标志字符
#define BLANK_CHAR '~'
#define CODE_CHAR '?'
#define START_CHAR '!'
#define CHECK_CHAR '@'
#define END_CHAR '#'
#define SUCCUESS_CHAR '$'
#define FAIL_CHAR '%'

//发送相关宏
#define FORCE_SEND 0 //强制发送
#define NO_FORCE_SEND 1 //非强制发送

#define DEFAULT_SEND_TIMES 1 //默认重复发送次数


//回调函数指针
typedef void (*HandleMessageFunction)(const char*);


class Radio
{
public:
    Radio();

    static void BeginTransmit(unsigned long baud_rate = RADIO_BAUD_RATE); //打开串口

    static void Send(const char* message, uint8_t send_type = NO_FORCE_SEND, uint8_t send_times = DEFAULT_SEND_TIMES); //发送

    static void SetHandleMessageFunction(HandleMessageFunction hm_func); //设置接收回调函数
    static void DisableReceiveInterrupt(); //禁用接收中断
    static void EnableReceiveInterrupt(); //恢复接收中断

private:
    static bool Receive(uint8_t ch, uint8_t status); //接收，使用中断触发

    static HandleMessageFunction hm_func; //接收回调函数

    static NeoHWSerial* NeoSerialX;
};


#endif