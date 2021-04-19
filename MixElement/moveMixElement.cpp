#include <Arduino.h>

#include "moveMixElement.h"

#ifdef MOVE_DEBUG
#include <NeoHWSerial.h>
#endif


//静态变量
float Move::global_speed_rate = DEFAULT_GLOBAL_SPEED_RATE; //全局速度比率
uint8_t Move::current_direction = STOP; //当前运动状态
float Move::current_speed_rate = 0.0; //当前运动速度比率
float Move::current_turn_speed_rate; //当前转向时一侧减速的比率


Move::Move() {}


Move::Move(float global_speed_rate)
{
    Move::global_speed_rate = global_speed_rate;
}


//设置全局速度比率
void Move::SetGlobalSpeedRate(float global_speed_rate)
{
    Move::global_speed_rate = global_speed_rate;
}


//获取当前运动方向
uint8_t Move::GetCurrentMove(void)
{
    return current_direction;
}


//获取当前运动速度比率
float Move::GetCurrentSpeedRate(void)
{
    return current_speed_rate;
}


//获取当前转向时一侧减速的比率
float Move::GetCurrentTurnSpeedRate(void)
{
    return current_turn_speed_rate;
}


//控制某个轮子转动
void Move::Wheel(uint8_t wheel, uint8_t rotation, float speed_rate)
{
    uint8_t pin_in1, pin_in2, pin_enx;

    switch (wheel)
    {
    case LEFT_A_WHEEL:
    {
        pin_in1 = LEFT_L298N_IN1;
        pin_in2 = LEFT_L298N_IN2;
        pin_enx = LEFT_L298N_ENA;
    } break;

    case LEFT_B_WHEEL:
    {
        pin_in1 = LEFT_L298N_IN3;
        pin_in2 = LEFT_L298N_IN4;
        pin_enx = LEFT_L298N_ENB;
    } break; 

    case RIGHT_A_WHEEL:
    {
        pin_in1 = RIGHT_L298N_IN1;
        pin_in2 = RIGHT_L298N_IN2;
        pin_enx = RIGHT_L298N_ENA;
    } break;

    case RIGHT_B_WHEEL:
    {
        pin_in1 = RIGHT_L298N_IN3;
        pin_in2 = RIGHT_L298N_IN4;
        pin_enx = RIGHT_L298N_ENB;
    }
    }

    if (rotation == STOP_ROTATION) //停止转动
    {
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, LOW);
        return; //结束函数
    }
    
    analogWrite(pin_enx, abs(speed_rate) * global_speed_rate * 255.0); //设置 PWM 波，即转速

    if ((rotation == FORWARD_ROTATION && speed_rate >= 0) || (rotation == BACKWARD_ROTATION && speed_rate < 0)) //向前转动
    {
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, HIGH);
    }
    else //向后转动
    {
        digitalWrite(pin_in1, HIGH);
        digitalWrite(pin_in2, LOW);
    }
}


//前进
void Move::Forward(float speed_rate)
{
    Wheel(LEFT_A_WHEEL, FORWARD_ROTATION, speed_rate);
    Wheel(LEFT_B_WHEEL, FORWARD_ROTATION, speed_rate);
    Wheel(RIGHT_A_WHEEL, FORWARD_ROTATION, speed_rate);
    Wheel(RIGHT_B_WHEEL, FORWARD_ROTATION, speed_rate);

    current_direction = FORWARD;
    current_speed_rate = speed_rate;
    current_turn_speed_rate = 1.0;

    #ifdef MOVE_DEBUG
    //调试输出前进状态
    NeoSerialDebug.print(F("#MOVE:   Move Forward"));
    NeoSerialDebug.print(F(" speed_rate: ")); NeoSerialDebug.println(speed_rate);
    #endif
}


//后退
void Move::Backward(float speed_rate)
{
    Wheel(LEFT_A_WHEEL, BACKWARD_ROTATION, speed_rate);
    Wheel(LEFT_B_WHEEL, BACKWARD_ROTATION, speed_rate);
    Wheel(RIGHT_A_WHEEL, BACKWARD_ROTATION, speed_rate);
    Wheel(RIGHT_B_WHEEL, BACKWARD_ROTATION, speed_rate);

    current_direction = BACKWARD;    
    current_speed_rate = speed_rate;
    current_turn_speed_rate = 1.0;

    #ifdef MOVE_DEBUG
    //调试输出后退状态
    NeoSerialDebug.print(F("#MOVE:   Move Backward"));
    NeoSerialDebug.print(F(" speed_rate: ")); NeoSerialDebug.println(speed_rate);
    #endif
}


//向前左转
void Move::ForLeftward(float speed_rate, float turn_speed_rate)
{
    Wheel(LEFT_A_WHEEL, FORWARD_ROTATION, turn_speed_rate * speed_rate);
    Wheel(LEFT_B_WHEEL, FORWARD_ROTATION, turn_speed_rate * speed_rate);
    Wheel(RIGHT_A_WHEEL, FORWARD_ROTATION, speed_rate);
    Wheel(RIGHT_B_WHEEL, FORWARD_ROTATION, speed_rate);

    current_direction = FORLEFTWARD;
    current_speed_rate = speed_rate;
    current_turn_speed_rate = turn_speed_rate;

    #ifdef MOVE_DEBUG
    //调试输出向前左转状态
    NeoSerialDebug.print(F("#MOVE:   Move ForLeftward"));
    NeoSerialDebug.print(F(" speed_rate: ")); NeoSerialDebug.print(speed_rate); NeoSerialDebug.print(F(" turn_speed_rate: ")); NeoSerialDebug.println(turn_speed_rate);
    #endif
}


//向前右转
void Move::ForRightward(float speed_rate, float turn_speed_rate)
{
    Wheel(LEFT_A_WHEEL, FORWARD_ROTATION, speed_rate);
    Wheel(LEFT_B_WHEEL, FORWARD_ROTATION, speed_rate);
    Wheel(RIGHT_A_WHEEL, FORWARD_ROTATION, turn_speed_rate * speed_rate);
    Wheel(RIGHT_B_WHEEL, FORWARD_ROTATION, turn_speed_rate * speed_rate);

    current_direction = FORRIGHTWARD;   
    current_speed_rate = speed_rate;
    current_turn_speed_rate = turn_speed_rate;

    #ifdef MOVE_DEBUG
    //调试输出向前右转状态
    NeoSerialDebug.print(F("#MOVE:   Move ForRightward"));
    NeoSerialDebug.print(F(" speed_rate: ")); NeoSerialDebug.print(speed_rate); NeoSerialDebug.print(F(" turn_speed_rate: ")); NeoSerialDebug.println(turn_speed_rate);
    #endif
}


//向后左转
void Move::BackLeftward(float speed_rate, float turn_speed_rate)
{
    Wheel(LEFT_A_WHEEL, BACKWARD_ROTATION, turn_speed_rate * speed_rate);
    Wheel(LEFT_B_WHEEL, BACKWARD_ROTATION, turn_speed_rate * speed_rate);
    Wheel(RIGHT_A_WHEEL, BACKWARD_ROTATION, speed_rate);
    Wheel(RIGHT_B_WHEEL, BACKWARD_ROTATION, speed_rate);

    current_direction = BACKLEFTWARD;    
    current_speed_rate = speed_rate;
    current_turn_speed_rate = turn_speed_rate;

    #ifdef MOVE_DEBUG
    //调试输出向后左转状态
    NeoSerialDebug.print(F("#MOVE:   Move BackLeftward"));
    NeoSerialDebug.print(F(" speed_rate: ")); NeoSerialDebug.print(speed_rate); NeoSerialDebug.print(F(" turn_speed_rate: ")); NeoSerialDebug.println(turn_speed_rate);
    #endif
}


//向后右转
void Move::BackRightward(float speed_rate, float turn_speed_rate)
{
    Wheel(LEFT_A_WHEEL, BACKWARD_ROTATION, speed_rate);
    Wheel(LEFT_B_WHEEL, BACKWARD_ROTATION, speed_rate);
    Wheel(RIGHT_A_WHEEL, BACKWARD_ROTATION, turn_speed_rate * speed_rate);
    Wheel(RIGHT_B_WHEEL, BACKWARD_ROTATION, turn_speed_rate * speed_rate);

    current_direction = BACKRIGHTWARD;     
    current_speed_rate = speed_rate;
    current_turn_speed_rate = turn_speed_rate;

    #ifdef MOVE_DEBUG
    //调试输出向后右转状态
    NeoSerialDebug.print(F("#MOVE:   Move BackRightward"));
    NeoSerialDebug.print(F(" speed_rate: ")); NeoSerialDebug.print(speed_rate); NeoSerialDebug.print(F(" turn_speed_rate: ")); NeoSerialDebug.println(turn_speed_rate);
    #endif
}


//制动
void Move::Stop(void)
{
    Wheel(LEFT_A_WHEEL, STOP_ROTATION);
    Wheel(LEFT_B_WHEEL, STOP_ROTATION);
    Wheel(RIGHT_A_WHEEL, STOP_ROTATION);
    Wheel(RIGHT_B_WHEEL, STOP_ROTATION);  

    current_direction = STOP;
    current_speed_rate = 0;
    current_turn_speed_rate = 1.0;

    #ifdef MOVE_DEBUG
    //调试输出制动状态
    NeoSerialDebug.println(F("#MOVE:   Move Stop"));
    #endif
}


//向某方向运动
void Move::MoveDirection(uint8_t direction, float speed_rate, float turn_speed_rate)
{
    switch (direction)
    {
    case FORWARD: Forward(speed_rate); break;
    case BACKWARD: Backward(speed_rate); break;
    case FORLEFTWARD: ForLeftward(speed_rate, turn_speed_rate); break;
    case FORRIGHTWARD: ForRightward(speed_rate, turn_speed_rate); break;
    case BACKLEFTWARD: BackLeftward(speed_rate, turn_speed_rate); break;
    case BACKRIGHTWARD: BackRightward(speed_rate, turn_speed_rate); break;
    case STOP: Stop();
    }
}