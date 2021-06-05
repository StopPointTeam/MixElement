#ifndef SENSORMIXELEMENT_H
#define SENSORMIXELEMENT_H


//注释以关闭调试功能
#define SENSOR_DEBUG

#ifdef SENSOR_DEBUG
#define NeoSerialDebug NeoSerial
#endif

//灰度传感器 OUT 接口定义
#define GRAY_1_OUT A5

//灰度传感器临界值
#define DEFAULT_GRAY_1_GATE 500

//灰度传感器标识定义
#define GRAY_1 0

//碰撞传感器 OUT 接口定义
#define BUTTON_1_OUT A0
#define BUTTON_2_OUT A1
#define BUTTON_3_OUT A4

//碰撞传感器 VCC 接口定义
#define BUTTON_2_VCC A2

//碰撞传感器标识定义
#define BUTTON_1 0
#define BUTTON_2 1
#define BUTTON_3 2


class Sensor
{
public:
    Sensor();

    //设置灰度传感器临界值
    static void SetGrayGate(uint8_t gray_sensor_num, int gate);

    //灰度传感器判断下方是否为白色
    static bool IsWhite(uint8_t gray_sensor_num);

    //碰撞传感器（开关）判断是否闭合
    static bool IsPushed(uint8_t button_num);

private:
    //灰度传感器临界值
    static int gray_1_gate;
};  


#endif