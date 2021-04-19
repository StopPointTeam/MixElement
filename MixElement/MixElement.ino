#include <Arduino.h>
#include <EEPROM.h>
#include <NeoHWSerial.h>

#include "routeMixElement.h" //路径库
#include "moveMixElement.h" //轮胎运动库
#include "sensorMixElement.h" //传感器库
#include "radioMixElement.h" //通信库


Route route; //路径实例
Move move; //轮胎运动实例
Sensor sensor; //传感器实例
Radio radio; //通讯实例


//****************************************可调参数****************************************
//EEPROM 写入位置
#define EEPROM_ADDRESS 0x0

//（前进转）从触碰白线到开始转向延时
#define BEFORE_FORTURN_DELAY 670
//（后退转）从触碰白线到开始转向延时
#define BEFORE_BACKTURN_DELAY 500

//开始转到开始检测是否摆正的延时
#define BEFORE_CHECK_STRAIGHT_DELAY 700
//***************************************************************************************


//****************************************全局变量****************************************
//GND Pins
const uint8_t gnd_pins[8] = {12, 13, 32, 33, 34, 35, 36, 37};

//EEPROM 储存的调试数据
struct StroageInfo
{
    int delay_time_after_turn;
} stroage_info;

//转向开始到结束的时间
int delay_time_after_turn;

//从 route 实例获取的点
Point passed_point;
Point next_point;
Point next_next_point;

#define FRONT_NEXT 0
#define BACK_NEXT 1
//下一点朝向
uint8_t next_position = FRONT_NEXT;

//底盘是否携带环
bool is_carry = false;
//***************************************************************************************


//****************************************自定函数****************************************
//设置接口低电平作为额外地
void SetGNDPins(void);

//从 EEPROM 读取数据
void ReadFromEEPROM(void);

//在开始运行前依次检测各灰度传感器下方黑白是否正常，若不正常，异常传感器闪烁，程序不继续进行
void CheckGrayStatus(void);

//从 route 实例获取点
void GetPointFromRoute(void);

//计算方向，同时更改完成转向后相对下一点的朝向
uint8_t CalcDirection(void);

//由灰度比计算修正减速比
float CalcFixSpeedRate(float gray_deviation_rate);

#define FRONT_END 0
#define BACK_END 1

#define ENABLE_FIX 0
#define DISABLE_FIX 1
//沿线直行，在触发条件后离开函数但不停止
void LineForward(uint8_t end_position, uint8_t type = ENABLE_FIX, float speed_rate = 1.0);

//沿线后退，在触发条件后离开函数但不停止
void LineBackward(uint8_t end_position, uint8_t type = ENABLE_FIX, float speed_rate = 1.0);

//直行或后退或转向，完成后离开函数但不停止。会自动跳过无需前往的释放点
void TurnDirection(float speed_rate = 1.0);

//通讯消息处理函数
void HandleMessage(const char* message);
//***************************************************************************************


//****************************************调试相关****************************************
//注释以关闭调试功能
#define MIXELE_DEBUG

#ifdef MIXELE_DEBUG

#include "MemoryUsage.h"

#define NeoSerialDebug NeoSerial3
#define DEBUG_BAUT_RATE 115200

int loop_time = 0;

//错误消息函数，用于在出现致命错误后结束程序
void DebugCanNotContinue(const char* message)
{
    move.Stop();
    
    NeoSerialDebug.print(F("#MIXELE: CAN NOT CONTINUE WHEN ")); NeoSerialDebug.println(message);
    NeoSerialDebug.print(F("#MIXELE: loop_time: ")); NeoSerialDebug.println(loop_time);
    NeoSerialDebug.print(F("#MIXELE: pass: [")); NeoSerialDebug.print(passed_point.x); NeoSerialDebug.print(F(", ")); NeoSerialDebug.print(passed_point.y); NeoSerialDebug.print(F("]"));
    NeoSerialDebug.print(F(" TYPE: ")); NeoSerialDebug.println((int)passed_point.type);
    NeoSerialDebug.print(F("#MIXELE: next: [")); NeoSerialDebug.print(next_point.x); NeoSerialDebug.print(F(", ")); NeoSerialDebug.print(next_point.y); NeoSerialDebug.print(F("]"));
    NeoSerialDebug.print(F(" next_position: ")); NeoSerialDebug.print((int)next_position);
    NeoSerialDebug.print(F(" TYPE: ")); NeoSerialDebug.println((int)next_point.type);
    SRamDisplay();

    while (1) {}
}
#endif
//***************************************************************************************


void setup()
{
    #ifdef MIXELE_DEBUG
    NeoSerialDebug.begin(DEBUG_BAUT_RATE);
    NeoSerialDebug.println(F("#MIXELE: ======================setup()====================="));
    SRamDisplay();
    #endif

    SetGNDPins();

    move.Stop();

    radio.SetHandleMessageFunction(HandleMessage);
    radio.BeginTransmit();

    //从 EEPROM 读取数据
    ReadEEPROM();

    //在开始运行前依次检测各灰度传感器下方黑白是否正常
    CheckGrayStatus();

    //前往 x, 0
    //沿线直行，到后端传感器接触下一条线离开函数
    LineForward(BACK_END);

    //已越过 x, 0 正式进入循环
}


void loop()
{
    //从 route 实例获取点
    GetPointFromRoute();
    
    #ifdef MIXELE_DEBUG
    loop_time++;
    NeoSerialDebug.println(F("#MIXELE: ====================New loop()===================="));
    NeoSerialDebug.print(F("#MIXELE: loop_time: ")); NeoSerialDebug.println(loop_time);
    NeoSerialDebug.print(F("#MIXELE: pass: [")); NeoSerialDebug.print(passed_point.x); NeoSerialDebug.print(F(", ")); NeoSerialDebug.print(passed_point.y); NeoSerialDebug.print(F("]"));
    NeoSerialDebug.print(F(" TYPE: ")); NeoSerialDebug.println((int)passed_point.type);
    NeoSerialDebug.print(F("#MIXELE: next: [")); NeoSerialDebug.print(next_point.x); NeoSerialDebug.print(F(", ")); NeoSerialDebug.print(next_point.y); NeoSerialDebug.print(F("]"));
    NeoSerialDebug.print(F(" next_position: ")); NeoSerialDebug.print((int)next_position);
    NeoSerialDebug.print(F(" TYPE: ")); NeoSerialDebug.println((int)next_point.type);
    SRamDisplay();
    #endif

    //情况一：刚完整经过普通点，下一个点为普通点或携带点
    if (passed_point.type == NORMAL_POINT && (next_point.type == NORMAL_POINT || next_point.type == CARRY_POINT))   
    {
        if (next_position == FRONT_NEXT)
        {
            //沿线直行，到前端传感器接触下一条线离开函数
            LineForward(FRONT_END);

            //若下一点为携带点
            if (next_point.type == CARRY_POINT)
            {
                //底盘携带
                is_carry = true;

                //越过点成为普通点
                route.SetNextPointType(NORMAL_POINT);
            }
        }
        else
        {
            //沿线后退，到后端传感器接触下一条线离开函数
            LineBackward(BACK_END);
        }

        //继续直行或后退或转向
        TurnDirection();
    }
    //情况二：刚完整经过普通点，下一个点为释放点（从底盘）
    else if (passed_point.type == NORMAL_POINT && next_point.type == GETOUT_POINT)
    {
        //沿线直行，到前端传感器接触下一条线离开函数
        LineForward(FRONT_END);
        
        //沿线直行，到后端传感器接触下一条线离开函数
        LineForward(BACK_END);

        //停止前进
        move.Stop();

        //下一点朝向为后
        next_position = BACK_NEXT;
    }
    //情况三：刚完整经过释放点（从底盘），下一个点为普通点
    else if (passed_point.type == GETOUT_POINT && next_point.type == NORMAL_POINT)
    {
        //沿线后退，到前端传感器接触线离开函数
        LineBackward(FRONT_END, DISABLE_FIX);

        //底盘携带清空
        is_carry = false;
        
        //沿线后退，到后端传感器接触线离开函数
        LineBackward(BACK_END);

        //继续后退或转向
        TurnDirection();
    }
    //出现错误
    else 
    {
        move.Stop();

        #ifdef MIXELE_DEBUG
        DebugCanNotContinue("CHOOSE LOOP");
        #endif
    }

    //更新位置，继续循环
    route.UpdatePosition();

    #ifdef MIXELE_DEBUG
    NeoSerialDebug.println(F("#MIXELE: ====================End loop()===================="));
    #endif
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


//从 EEPROM 读取数据
void ReadEEPROM(void)
{
    //从 EEPROM 读取调试数据
    EEPROM.get(EEPROM_ADDRESS, stroage_info);

    //转向时间
    delay_time_after_turn = stroage_info.delay_time_after_turn;

    #ifdef MIXELE_DEBUG
    NeoSerialDebug.println(F("#MIXELE: Data based on EEPROM: "));
    NeoSerialDebug.print(F("#MIXELE: delay_time_after_turn: ")); NeoSerialDebug.println(delay_time_after_turn);
    #endif
}


//在开始运行前依次检测各灰度传感器下方黑白是否正常，若不正常，异常传感器闪烁，程序不继续进行
void CheckGrayStatus(void)
{
    //若正常，1 2 5 6 号传感器检测到黑色，3 4 号传感器检测到白色
    bool is_status_right = false;

    while (!is_status_right)
    {
        if (sensor.IsWhite(GRAY_1))
            sensor.FlashGraySensor(GRAY_1);
        else if (sensor.IsWhite(GRAY_2))
            sensor.FlashGraySensor(GRAY_2);
        else if (!sensor.IsWhite(GRAY_3))
            sensor.FlashGraySensor(GRAY_3);
        else if (!sensor.IsWhite(GRAY_4))
            sensor.FlashGraySensor(GRAY_4);
        else if (sensor.IsWhite(GRAY_5))
            sensor.FlashGraySensor(GRAY_5);
        else if (sensor.IsWhite(GRAY_6))
            sensor.FlashGraySensor(GRAY_6);
        else is_status_right = true;
    }

    #ifdef MIXELE_DEBUG
    NeoSerialDebug.println(F("#MIXELE: Gray Sensor Status OK!"));
    #endif
}


//从 route 实例获取点
void GetPointFromRoute(void)
{
    passed_point = route.GetPassedPoint();
    next_point = route.GetNextPoint();
    next_next_point = route.GetNextNextPoint();
}


//计算方向，同时更改完成转向后相对下一点的朝向
uint8_t CalcDirection(void)
{
    //计算第三点与第一点的相对坐标 rx0, ry0
    int8_t rx0 = next_next_point.x - passed_point.x;
    int8_t ry0 = next_next_point.y - passed_point.y;

    //计算当前小车朝向的方向向量 vx, vy
    int8_t vx = next_point.x - passed_point.x;
    int8_t vy = next_point.y - passed_point.y;

    //坐标旋转变换
    int8_t rx, ry;
    if (vx == 0 && vy == 1)
    {
        rx = rx0;
        ry = ry0;
    }
    else if (vx == -1 && vy == 0)
    {
        rx = ry0;
        ry = -rx0;
    }
    else if (vx == 0 && vy == -1)
    {
        rx = -rx0;
        ry = -ry0;
    }
    else if (vx == 1 && vy == 0)
    {
        rx = -ry0;
        ry = rx0;
    }
    #ifdef MIXELE_DEBUG
    else DebugCanNotContinue("CALC DIRECTION <1>"); //调试用
    #endif

    //判断行进方向
    if (rx == 0 && ry == 2)
    {
        if (next_position == FRONT_NEXT)
        {
            return FORWARD;
        }
        else
        {
            return BACKWARD;
        }
    }
    else if (rx == 0 && ry == 0)
    {
        if (next_position == FRONT_NEXT)
        {
            next_position = BACK_NEXT;
            return BACKWARD;
        }
        else
        {
            next_position = FRONT_NEXT;
            return FORWARD;
        }
    }
    else if (rx == -1 && ry == 1)
    {
        if (next_position == FRONT_NEXT)
        {
            return FORLEFTWARD;
        }
        else
        {
            next_position = FRONT_NEXT;
            return BACKLEFTWARD;
        }
    }
    else if (rx == 1 && ry == 1)
    {
        if (next_position == FRONT_NEXT)
        {
            return FORRIGHTWARD;
        }
        else
        {
            next_position = FRONT_NEXT;
            return BACKRIGHTWARD;
        }
    }
    #ifdef MIXELE_DEBUG
    else DebugCanNotContinue("CALC DIRECTION <2>"); //调试用
    #endif
}


//由灰度比计算修正减速比
float CalcFixSpeedRate(float gray_deviation_rate)
{
    return -50.0 * pow((gray_deviation_rate - 1.0), 2.0) + 1.0;
}


//沿线直行，在触发条件后离开函数但不停止
void LineForward(uint8_t end_position, uint8_t type, float speed_rate)
{
    #ifdef MIXELE_DEBUG
    //调试输出沿线直行状态
    NeoSerialDebug.print(F("#MIXELE: Line Forward"));
    NeoSerialDebug.print(F(" end_position: "));
    NeoSerialDebug.println((int)end_position);
    #endif

    move.Forward(speed_rate);

    //记录开始时间
    unsigned long begin_time = millis();
    //记录灰度传感器匹配情况
    bool gray_match_a = false;
    bool gray_match_b = false;

    while (1)
    {
        if (type == ENABLE_FIX)
        {
            if (!sensor.IsWhite(GRAY_3) && sensor.IsWhite(GRAY_4)) //左侧越线
            {
                move.ForRightward(speed_rate, CalcFixSpeedRate(sensor.GrayDeviationRate(GRAY_3)));
            }
            else if (sensor.IsWhite(GRAY_3) && !sensor.IsWhite(GRAY_4)) //右侧越线
            {
                move.ForLeftward(speed_rate, CalcFixSpeedRate(sensor.GrayDeviationRate(GRAY_4)));
            }
            else if (sensor.IsWhite(GRAY_3) && sensor.IsWhite(GRAY_4)) //在白线上
            {
                move.Forward(speed_rate);
            }
            else //均不符合
            {
                //move.Backward(LINE_FIND_SPEED_RATE);
            }
        }

        if (end_position == FRONT_END) //前端接触线离开函数
        {
            if (sensor.IsWhite(GRAY_1))
                gray_match_a = true;
            
            if (sensor.IsWhite(GRAY_2))
                gray_match_b = true;
        }
        else if (end_position == BACK_END) //后端接触线离开函数
        {
            if (sensor.IsWhite(GRAY_5))
                gray_match_a = true;
            
            if (sensor.IsWhite(GRAY_6))
                gray_match_b = true;
        }

        //对应前端接触线离开函数和后端接触线离开函数的情况
        if (gray_match_a && gray_match_b)
            break;
    }

    #ifdef MIXELE_DEBUG
    //调试输出沿线直行结束
    NeoSerialDebug.println(F("#MIXELE: End Line Forward"));
    #endif
}


//沿线后退，在触发条件后离开函数但不停止
void LineBackward(uint8_t end_position, uint8_t type, float speed_rate)
{
    #ifdef MIXELE_DEBUG
    //调试输出沿线后退状态
    NeoSerialDebug.print(F("#MIXELE: Line Backward"));
    NeoSerialDebug.print(F(" end_position: "));
    NeoSerialDebug.println((int)end_position);
    #endif

    //记录灰度传感器匹配情况
    bool gray_match_a = false;
    bool gray_match_b = false;

    move.Backward(speed_rate);

    while (1)
    {
        if (type == ENABLE_FIX)
        {
            if (!sensor.IsWhite(GRAY_3) && sensor.IsWhite(GRAY_4)) //左侧越线
            {
                move.BackRightward(speed_rate, CalcFixSpeedRate(sensor.GrayDeviationRate(GRAY_3)));
            }
            else if (sensor.IsWhite(GRAY_3) && !sensor.IsWhite(GRAY_4)) //右侧越线
            {
                move.BackLeftward(speed_rate, CalcFixSpeedRate(sensor.GrayDeviationRate(GRAY_4)));
            }
            else if (sensor.IsWhite(GRAY_3) && sensor.IsWhite(GRAY_4)) //在白线上
            {
                move.Backward(speed_rate);
            }
            else //均不符合
            {
                //move.Forward(LINE_FIND_SPEED_RATE);
            }
        }

        if (end_position == FRONT_END) //前端接触线离开函数
        {
            if (sensor.IsWhite(GRAY_1))
                gray_match_a = true;
            
            if (sensor.IsWhite(GRAY_2))
                gray_match_b = true;
        }
        else //后端接触线离开函数
        {
            if (sensor.IsWhite(GRAY_5))
                gray_match_a = true;
            
            if (sensor.IsWhite(GRAY_6))
                gray_match_b = true;
        }

        //对应前端接触线离开函数和后端接触线离开函数的情况
        if (gray_match_a && gray_match_b)
            break;
    }

    #ifdef MIXELE_DEBUG
    //调试输出沿线后退结束
    NeoSerialDebug.println(F("#MIXELE: End Line Backward"));
    #endif
}


//直行或后退或转向，完成后离开函数但不停止。会自动跳过无需前往的释放点
void TurnDirection(float speed_rate)
{    
    //若下下点为底盘释放点，判断是否需要跳过
    if (next_next_point.type == GETOUT_POINT && !is_carry)
    {
        route.ChangeRoute(JUMP_DEAD_ROAD);
        GetPointFromRoute();

        #ifdef MIXELE_DEBUG
        //调试输出直行或后退或转向状态
        NeoSerialDebug.println(F("#MIXELE: JUMP THE GETOUT POINT FOR STATUS REASON"));
        #endif
    }
    
    uint8_t direction = CalcDirection();
    
    #ifdef MIXELE_DEBUG
    //调试输出直行或后退或转向状态
    NeoSerialDebug.print(F("#MIXELE: Turn Direction"));
    NeoSerialDebug.print(F(" direction: "));
    NeoSerialDebug.println((int)direction);
    #endif
    
    if (direction == FORWARD) //继续直行
    {
        //沿线直行，到后端传感器接触线离开函数
        LineForward(BACK_END, speed_rate);
    }
    else if (direction == BACKWARD) //继续后退
    {
        //沿线后退，到前端传感器接触线离开函数
        LineBackward(FRONT_END, speed_rate);
    }
    else //继续转向
    {
        //等待小车旋转中心与十字中心重合
        if (direction == FORLEFTWARD || direction == FORRIGHTWARD)
            delay(BEFORE_FORTURN_DELAY);
        else delay(BEFORE_BACKTURN_DELAY);

        //旋转
        move.MoveDirection(direction, speed_rate);

        //基于时间判定结束
        delay(delay_time_after_turn);
    }

    #ifdef MIXELE_DEBUG
    //调试输出直行或后退或转向结束
    NeoSerialDebug.println(F("#MIXELE: End Turn Direction"));
    #endif
}


//通讯消息处理函数
void HandleMessage(const char* message)
{
    radio.Send("Get the message: ");
    radio.Send(message);
}