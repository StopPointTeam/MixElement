#ifndef ROUTEMIXELEMENT_H
#define ROUTEMIXELEMENT_H


#include <Arduino.h>

//注释以关闭调试功能
#define ROUTE_DEBUG

#ifdef ROUTE_DEBUG
#define NeoSerialDebug NeoSerial3
#endif

//坐标点操作定义
#define NORMAL_POINT 0 //普通点
#define CARRY_POINT 3 //携带点（从底盘）
#define GETOUT_POINT 4 //释放点（从底盘）

//坐标点数组定义
#define X 0
#define Y 1
#define TYPE 2

//更改路径选项
#define JUMP_DEAD_ROAD 0


//点结构
struct Point
{
    int8_t x;
    int8_t y;
    uint8_t type;

    Point* next;
};


class Route
{
public:
    Route();

    static Point GetPassedPoint(void);
    static Point GetNextPoint(void);
    static Point GetNextNextPoint(void);

    static void SetPassedPoinType(uint8_t type);
    static void SetNextPointType(uint8_t type);
    static void SetNextNextPointType(uint8_t type);

    static void UpdatePosition(void); //更新位置

    static void ChangeRoute(uint8_t choice); //更改路径

private:
    static void SetPointX(Point* point, int8_t x); //设置点的 x 值
    static void SetPointY(Point* point, int8_t y); //设置点的 y 值
    static void SetPointType(Point* point, uint8_t type); //设置点的 type 值
    static void SetPointNext(Point* point, Point* next); //设置点的 next 值

    static Point* CreatePoint(int8_t x, int8_t y, uint8_t type); //新建点

    static void Connect(Point* point_a, Point* point_b); //连接 point_a point_b

    static Point* AddPoint(Point* front_point, int8_t x, int8_t y, uint8_t type, Point* next_point = NULL); //在 front_point 和 next_point 间插入新的点

    static void InitBaseRoute(void); //生成基本路径点链

    static Point* head_point; //头部点
    static Point* passed_point; //经过的点
};


#endif