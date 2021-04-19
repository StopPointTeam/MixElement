#ifndef SENSORMIXELEMENT_H
#define SENSORMIXELEMENT_H


//注释以关闭调试功能
#define SENSOR_DEBUG

#ifdef SENSOR_DEBUG
#define NeoSerialDebug NeoSerial3
#endif

//灰度传感器 OUT 接口定义
#define GRAY_1_OUT A0
#define GRAY_2_OUT A1
#define GRAY_3_OUT A2
#define GRAY_4_OUT A3
#define GRAY_5_OUT A4
#define GRAY_6_OUT A5
#define GRAY_7_OUT A6

//灰度传感器 VCC 接口定义
#define GRAY_1_VCC 48
#define GRAY_2_VCC 49
#define GRAY_3_VCC 50
#define GRAY_4_VCC 51
#define GRAY_5_VCC 52
#define GRAY_6_VCC 53
#define GRAY_7_VCC 47

//灰度传感器临界值
#define DEFAULT_GRAY_1_GATE 900
#define DEFAULT_GRAY_2_GATE 900
#define DEFAULT_GRAY_3_GATE 850
#define DEFAULT_GRAY_4_GATE 850
#define DEFAULT_GRAY_5_GATE 900
#define DEFAULT_GRAY_6_GATE 880
#define DEFAULT_GRAY_7_GATE 690

//灰度传感器闪烁时间
#define GRAY_FLASH_TIME 200

//灰度传感器标识定义
#define GRAY_1 0
#define GRAY_2 1
#define GRAY_3 2
#define GRAY_4 3
#define GRAY_5 4
#define GRAY_6 5
#define GRAY_7 6

//碰撞传感器 OUT 接口定义
#define BUTTON_1_OUT 2
#define BUTTON_2_OUT 3

//碰撞传感器 VCC 接口定义
#define BUTTON_1_VCC 45
#define BUTTON_2_VCC 46

//碰撞传感器标识定义
#define BUTTON_1 0
#define BUTTON_2 1


class Sensor
{
public:
    Sensor();

    //设置灰度传感器临界值
    static void SetGrayGate(uint8_t gray_sensor_num, int gate);

    //使灰度传感器闪烁
    static void FlashGraySensor(uint8_t gray_sensor_num);

    //灰度传感器判断下方是否为白色
    static bool IsWhite(uint8_t gray_sensor_num);

    //灰度传感器灰度值偏离比例，即 (gray_gate - gray_val) / gray_gate
    static float GrayDeviationRate(uint8_t gray_sensor_num);

    //碰撞传感器（开关）判断是否闭合
    static bool IsPushed(uint8_t button_num);

private:
    //灰度传感器临界值
    static int gray_1_gate;
    static int gray_2_gate;
    static int gray_3_gate;
    static int gray_4_gate;
    static int gray_5_gate;
    static int gray_6_gate;
    static int gray_7_gate;
};  


#endif