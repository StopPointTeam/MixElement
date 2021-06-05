#include <Arduino.h>
#include <NeoHWSerial.h>

#include "moveMixElement.h" //运动库
#include "sensorMixElement.h" //传感器库


Move move; //轮胎运动实例
Sensor sensor; //传感器实例


//****************************************可调参数****************************************
//从触碰白线到开始转向延时
#define BEFORE_FORTURN_DELAY 175

//开始转到结束转的延时
#define AFTER_FORTURN_DELAY 270
//***************************************************************************************


//****************************************全局变量****************************************
//GND Pins
const uint8_t gnd_pins[1] = {A3};
//***************************************************************************************


//****************************************自定函数****************************************
//设置接口低电平作为额外地
void SetGNDPins(void);
//***************************************************************************************


//****************************************调试相关****************************************
//注释以关闭调试功能
#define TAICHI_DEBUG

#ifdef TAICHI_DEBUG

#define NeoSerialDebug NeoSerial
#define DEBUG_BAUT_RATE 115200

#endif
//***************************************************************************************


void setup()
{
    uint8_t mode;
    
    NeoSerialDebug.begin(DEBUG_BAUT_RATE);
    
    SetGNDPins();

    // while (1)
    // {
    //     sensor.IsWhite(GRAY_1);
    // }

    while (1)
    {
        if (sensor.IsPushed(BUTTON_1))
        {
            mode = 0;
            break;
        }
        else if (sensor.IsPushed(BUTTON_2))
        {
            mode = 1;
            break;
        }
    }

    if (mode == 0)
    {
        uint8_t white_times = 0;

        move.Forward();

        while (white_times < 3)
        {
            if (sensor.IsWhite(GRAY_1))
            {
                white_times++;
                delay(200);
            }
        }

        delay(BEFORE_FORTURN_DELAY);
        move.ForRightward();
        delay(AFTER_FORTURN_DELAY);

        move.Stop();
        
        //white_times = 0;
        move.Forward();

        long start_time = millis();

        while (1)
        {
            if (sensor.IsPushed(BUTTON_3) || millis() - start_time > 10000)
            {
                move.Stop();
                while (1) {}
            }
        }

        // while (white_times < 5)
        // {
        //     if (sensor.IsWhite(GRAY_1))
        //     {
        //         white_times++;
        //         delay(200);
        //     }
        // }

        // move.Stop();
    }
    else
    {
        uint8_t white_times = 0;

        move.Forward();

        while (white_times < 3)
        {
            if (sensor.IsWhite(GRAY_1))
            {
                white_times++;
                delay(200);
            }
        }

        delay(BEFORE_FORTURN_DELAY);

        // move.Stop();
        // while (1) {}

        move.ForRightward();

        while (1)
        {
            move.ForRightward();
            delay(5000);
            move.Stop();
            delay(5000);
        }
    }
}


void loop()
{

}


//设置接口低电平作为额外地
void SetGNDPins(void)
{
    uint8_t pin_num = sizeof(gnd_pins) / sizeof(uint8_t);

    for (uint8_t i = 0; i < pin_num; i++)
    {
        pinMode(gnd_pins[i], OUTPUT);
        digitalWrite(gnd_pins[i], LOW);
    }
}