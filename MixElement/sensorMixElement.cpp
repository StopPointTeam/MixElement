#include <Arduino.h>

#include "sensorMixElement.h"

#ifdef SENSOR_DEBUG
#include <NeoHWSerial.h>
#endif


//静态变量
int Sensor::gray_1_gate = DEFAULT_GRAY_1_GATE;


Sensor::Sensor()
{
    pinMode(BUTTON_1_OUT, INPUT_PULLUP);
    pinMode(BUTTON_2_OUT, INPUT_PULLUP);
    pinMode(BUTTON_3_OUT, INPUT_PULLUP);

    pinMode(BUTTON_2_VCC, OUTPUT);
    digitalWrite(BUTTON_2_VCC, HIGH);
}


//设置灰度传感器临界值
void Sensor::SetGrayGate(uint8_t gray_sensor_num, int gate)
{
    switch (gray_sensor_num)
    {
    case GRAY_1: gray_1_gate = gate;
    }
}


bool Sensor::IsWhite(uint8_t gray_sensor_num)
{
    uint8_t gray_out_pin;
    int gray_val;
    int gray_gate;
    
    switch (gray_sensor_num)
    {
    case GRAY_1: gray_out_pin = GRAY_1_OUT; gray_gate = gray_1_gate;
    }

    gray_val = analogRead(gray_out_pin);

    #ifdef SENSOR_DEBUG
    //调试输出灰度值
    switch (gray_sensor_num)
    {
    case GRAY_1: NeoSerialDebug.print(F("#SENSOR: GRAY_1 and gate_val: "));
    }

    NeoSerialDebug.print(gray_val);
    NeoSerialDebug.print(F(" "));
    NeoSerialDebug.println(gray_gate);
    #endif

    if (gray_val > gray_gate)
        return true;
    else return false;
}


//碰撞传感器（开关）判断是否闭合
bool Sensor::IsPushed(uint8_t button_num)
{
    uint8_t button_out_pin;
    int button_val;

    if (button_num == BUTTON_1)
        button_out_pin = BUTTON_1_OUT;
    else if (button_num == BUTTON_2)
        button_out_pin = BUTTON_2_OUT;
    else button_out_pin = BUTTON_3_OUT;

    button_val = digitalRead(button_out_pin);

    #ifdef SENSOR_DEBUG
    //调试输出按钮状态
    if (button_num == BUTTON_1)
        NeoSerialDebug.print(F("#SENSOR: BUTTON_1: "));
    else if (button_num == BUTTON_2)
        NeoSerialDebug.print(F("#SENSOR: BUTTON_2: "));
    else NeoSerialDebug.print(F("#SENSOR: BUTTON_3: "));

    if (button_val == HIGH)
        NeoSerialDebug.println(F("pushed"));
    else NeoSerialDebug.println(F("released"));
    #endif    

    if (button_val == HIGH)
        return true;
    else return false;
}