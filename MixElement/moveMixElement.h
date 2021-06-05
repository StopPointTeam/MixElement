#ifndef MOVEMIXELEMENT_H
#define MOVEMIXELEMENT_H


//轮胎定义
#define LEFT_A_WHEEL 0
#define LEFT_B_WHEEL 1
#define RIGHT_A_WHEEL 2
#define RIGHT_B_WHEEL 3

//轮胎旋转方向定义，包括停止
#define FORWARD_ROTATION 0
#define BACKWARD_ROTATION 1
#define STOP_ROTATION 6

//运动方向、状态定义，与轮胎旋转方向定义兼容
#define FORWARD 0
#define BACKWARD 1
#define FORLEFTWARD 2
#define FORRIGHTWARD 3
#define BACKLEFTWARD 4
#define BACKRIGHTWARD 5
#define STOP 6

//默认全局速度比率
#define DEFAULT_GLOBAL_SPEED_RATE 1.0

//默认转向时一侧减速的比率
#define DEFAULT_TRUN_SPEED_RATE -1.0

//左侧 L298N 接口定义
#define LEFT_L298N_IN1 7
#define LEFT_L298N_IN2 6
#define LEFT_L298N_IN3 8
#define LEFT_L298N_IN4 9
#define LEFT_L298N_ENA 4
#define LEFT_L298N_ENB 5

//右侧 L298N 接口定义
#define RIGHT_L298N_IN1 13
#define RIGHT_L298N_IN2 12
#define RIGHT_L298N_IN3 11
#define RIGHT_L298N_IN4 10
#define RIGHT_L298N_ENA 2
#define RIGHT_L298N_ENB 3


class Move
{
public:
    Move();
    Move(float global_speed_rate);

    static void SetGlobalSpeedRate(float global_speed_rate); //设置全局速度比率

    static uint8_t GetCurrentMove(void); //获取当前运动方向
    static float GetCurrentSpeedRate(void); //获取当前运动速度比率
    static float GetCurrentTurnSpeedRate(void); //获取当前转向时一侧减速的比率

    static void Wheel(uint8_t wheel, uint8_t rotation, float speed_rate = 1.0); //控制某个轮子转动

    static void Forward(float speed_rate = 1.0); //前进
    static void Backward(float speed_rate = 1.0); //后退
    static void ForLeftward(float speed_rate = 1.0, float turn_speed_rate = DEFAULT_TRUN_SPEED_RATE); //向前左转
    static void ForRightward(float speed_rate = 1.0, float turn_speed_rate = DEFAULT_TRUN_SPEED_RATE); //向前右转
    static void BackLeftward(float speed_rate = 1.0, float turn_speed_rate = DEFAULT_TRUN_SPEED_RATE); //向后左转
    static void BackRightward(float speed_rate = 1.0, float turn_speed_rate = DEFAULT_TRUN_SPEED_RATE); //向后右转
    static void Stop(void); //制动

    static void MoveDirection(uint8_t direction, float speed_rate = 1.0, float turn_speed_rate = DEFAULT_TRUN_SPEED_RATE); //向某方向运动

private:
    static float global_speed_rate; //全局速度比率
    static uint8_t current_direction; //当前运动状态
    static float current_speed_rate; //当前运动速度比率
    static float current_turn_speed_rate; //当前转向时一侧减速的比率
};


#endif