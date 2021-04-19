#include <Arduino.h>
#include <NeoHWSerial.h>

#include "radioMixElement.h"


//静态变量
HandleMessageFunction Radio::hm_func;
NeoHWSerial* Radio::NeoSerialX = &RADIO_SERIAL_NUM;


Radio::Radio()
{
    EnableReceiveInterrupt();
}


//打开串口
void Radio::BeginTransmit(unsigned long baud_rate)
{
    NeoSerialX->begin(baud_rate);
}


//发送
void Radio::Send(const char* message, uint8_t send_type, uint8_t send_times)
{
    //强制发送，禁用接收中断，防止发送被打断
    if (send_type == FORCE_SEND)
        DisableReceiveInterrupt();

    static unsigned char send_code = '0';

    //生成完整通信包
    char full_message[FULL_MESSAGE_SIZE];
    uint8_t message_length = strlen(message);
    char check_str[CHECK_STR_LENGTH + 1];

    message_length = message_length < MAX_REAL_MESSAGE_SIZE ? message_length : MAX_REAL_MESSAGE_SIZE;

    itoa(message_length, check_str, 10);

    for (uint8_t i = 0, j = 0, k = 0; i < FULL_MESSAGE_SIZE; i++)
    {
        if (i < BLANK_CHAR_LENGTH)
        {
            full_message[i] = BLANK_CHAR;
        }
        else if (i == BLANK_CHAR_LENGTH)
        {
            full_message[i] = CODE_CHAR;

            full_message[++i] = send_code++;

            if (send_code > '9')
                send_code = '0';

            full_message[++i] = START_CHAR;
        }
        else if (j < message_length && j < MAX_REAL_MESSAGE_SIZE)
        {
            full_message[i] = message[j];
            j++;
        }
        else if (j == message_length || j == MAX_REAL_MESSAGE_SIZE)
        {
            full_message[i] = CHECK_CHAR;
            j++;
        }
        else if (k < CHECK_STR_LENGTH)
        {
            full_message[i] = check_str[k];
            k++;
            
            if (check_str[k] == '\0')
                k = CHECK_STR_LENGTH;
        }
        else
        {
            full_message[i] = END_CHAR;
        }
    }

    //发送完整通信包
    for (uint8_t times = 0; times < send_times; times++)
    {
        NeoSerialX->print(full_message);
    }

    #ifdef RADIO_DEBUG
    NeoSerialDebug.print(F("#RADIO:  SEND: "));
    full_message[FULL_MESSAGE_SIZE - 1] = '\0';
    NeoSerialDebug.print(full_message);
    NeoSerialDebug.print(F(" TIMES: "));
    NeoSerialDebug.println(send_times);
    #endif

    if (send_type == FORCE_SEND)
        EnableReceiveInterrupt();
}


//设置接收回调函数
void Radio::SetHandleMessageFunction(HandleMessageFunction hm_func)
{
    Radio::hm_func = hm_func;
}


//禁用接收中断
void Radio::DisableReceiveInterrupt() //禁用接收中断
{
    NeoSerialX->detachInterrupt();
}


//恢复接收中断
void Radio::EnableReceiveInterrupt() //恢复接收中断
{
    NeoSerialX->attachInterrupt(Receive);
}


//接收，使用中断触发
bool Radio::Receive(uint8_t ch, uint8_t status)
{
    static bool is_code_record = false;
    static bool is_start_record = false;
    static bool is_check_record = false;
    static char message[FULL_MESSAGE_SIZE] = "";
    static char check_str[CHECK_STR_LENGTH + 1] = "";
    static unsigned char this_package_code = 0;
    static unsigned char last_package_code = 0;
    static uint8_t i = 0, j = 0;

    switch (ch)
    {
    case CODE_CHAR:
    {
        is_code_record = true;
        is_start_record = false;
        is_check_record = false;
        
        //重置字符串
        for (uint8_t k = 0; k < i; k++)
            message[k] = '\0';
        for (uint8_t k = 0; k < j; k++)
            check_str[k] = '\0';

        i = 0;
        j = 0;
    } break;

    case START_CHAR:
    {
        if (is_code_record == true)
        {
            is_code_record = false;
            is_start_record = true;
        }
    } break;

    case CHECK_CHAR:
    {
        if (is_start_record == true)
        {
            is_start_record = false;
            is_check_record = true;
        }
    } break;

    case END_CHAR:
    {
        if (is_check_record == true)
        {
            is_check_record = false;

            if (j <= CHECK_STR_LENGTH)
                check_str[j] = '\0';

            if (atoi(check_str) == strlen(message)) //位数校验成功
            {   
                if (this_package_code != last_package_code)
                {
                    #ifdef RADIO_DEBUG
                    NeoSerialDebug.print(F("#RADIO:  RECEIVE: "));
                    NeoSerialDebug.println(message);
                    #endif
                    
                    hm_func(message);

                    last_package_code = this_package_code;
                }
            }
            else //位数校验失败
            {                
                #ifdef RADIO_DEBUG
                NeoSerialDebug.println(F("#RADIO:  RECEIVE CHECK FAIL!"));
                #endif
            }
        }
    } break;

    default:
    {
        if (is_code_record == true)
        {            
            this_package_code = ch;
        }
        else if (is_start_record == true)
        {
            if (i < FULL_MESSAGE_SIZE)
                message[i] = ch;
            i++;
        }
        else if (is_check_record == true)
        {
            if (j <= CHECK_STR_LENGTH)
                check_str[j] = ch;
            j++;
        }
    }
    }

    return false;
}