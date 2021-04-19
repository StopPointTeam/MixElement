#include <Arduino.h>

#include "routeMixElement.h"


#ifdef ROUTE_DEBUG
#include <NeoHWSerial.h>
#endif


//静态变量
Point* Route::head_point = NULL;
Point* Route::passed_point = NULL;


Route::Route() 
{
    //生成基本路径点链
    InitBaseRoute();

    //设置 passed_point = head_point
    passed_point = head_point;
}


Point Route::GetPassedPoint(void)
{
    return *passed_point;
}


Point Route::GetNextPoint(void)
{
    return *passed_point->next;
}


Point Route::GetNextNextPoint(void)
{
    return *passed_point->next->next;
}


void Route::SetPassedPoinType(uint8_t type)
{
    SetPointType(passed_point, type);
}


void Route::SetNextPointType(uint8_t type)
{
    SetPointType(passed_point->next, type);
}


void Route::SetNextNextPointType(uint8_t type)
{
    SetPointType(passed_point->next->next, type);
}


//更新位置
void Route::UpdatePosition(void)
{
    passed_point = passed_point->next;

    #ifdef ROUTE_DEBUG
    NeoSerialDebug.println(F("#ROUTE:  UPDATE POSITION!"));
    #endif
}


//更改路径
void Route::ChangeRoute(uint8_t choice)
{
    switch (choice)
    {
    case JUMP_DEAD_ROAD:
    {
        Connect(passed_point, passed_point->next->next->next);
    }
    }
}


//设置点的 x 值
void Route::SetPointX(Point* point, int8_t x)
{
    point->x = x;
}


//设置点的 y 值
void Route::SetPointY(Point* point, int8_t y)
{
    point->y = y;
}


//设置点的 type 值
void Route::SetPointType(Point* point, uint8_t type)
{
    point->type = type;
}


//设置点的 next 值
void Route::SetPointNext(Point* point, Point* next)
{
    point->next = next;
}


//新建点
Point* Route::CreatePoint(int8_t x, int8_t y, uint8_t type)
{
    Point* point = new Point;

    SetPointX(point, x);
    SetPointY(point, y);
    SetPointType(point, type);
    SetPointNext(point, NULL);

    return point;
}


//连接 point_a 和 point_b，使 point_a->next == point_b
//在主点链存在的情况下，point_a 和 point_b 必须属于主点链，否则可能出现不可预料的结果。函数不对 point_a 和 point_b 是否属于主点链进行检查
//在主点链不存在的情况下，point_a 和 point_b 必须属于单向点链，否则可能出现不可预料的结果。函数不对 point_a 和 point_b 是否属于单向点链进行检查
//若主点链为单向，point_a 在 point_b 之前，则 point_a 和 point_b 之间的点将被删除
//若主点链为环形，head_point 不在 point_a point_b 之间，则 point_a 和 point_b 之间的点将被删除
//若主点链为单向，point_a 在 point_b 之后，则 point_b 之前和 point_a 之后的点将被删除，point_b 成为 head_point，形成环形点链
//若主点链为环形，head_point 在 point_b point_a 之间，则 point_b point_a 之间的点将被删除，point_b 成为 head_point
//若主点链为单向，point_b head_point 均不为空，但 point_a 为空，则从 head_point 开始删除，并将 point_b 作为 head_point
//若主点链为环形，point_b head_point 均不为空，但 point_a 为空，则从 head_point 开始删除，并将 point_b 作为 head_point，形成单向点链
//若 point_a point_b 均不为空，但 head_point 为空（主点链不存在），则 point_a 成为 head_point，point_a 之前的点和 point_b 之前、之后的点不会被删除，point_a 之后的点会被删除
//若 point_b 不为空，point_a head_point 均为空（主点链不存在），则 point_b 成为 head_point，point_b 之后的点不会被删除
//若主点链为单向，point_a head_point 均不为空，但 point_b 为空，则从 point_a 的下一点开始删除，point_a 作为末尾点
//若主点链为环形，point_a head_point 均不为空，但 point_b 为空，则从 point_a 的下一点开始删除，point_a 作为末尾点，形成单向点链
//若 head_point 不为空，point_a point_b 均为空，则删除整个点链
//若 point_a point_b head_point 均为空，不进行操作
void Route::Connect(Point* point_a, Point* point_b)
{
    if (point_b) //point_b 不为空
    {
        if (head_point) //head_point 不为空
        {
            Point* temp_point;
            Point* delete_point;

            if (point_a) //point_a 不为空，则从 point_a 的下一点开始删除，并连接 point_a 和 point_b
            {
                temp_point = point_a->next;
                point_a->next = point_b;
            }
            else //point_a 为空，则从 head_point 开始删除，并将 point_b 作为 head_point
            {
                temp_point = point_b;

                while (!temp_point)
                {
                    if (temp_point->next == head_point) //为环形链表，将尾部 next 置空，使其变为单向链表
                        temp_point->next = NULL;

                    temp_point = temp_point->next;
                }

                temp_point = head_point;
                head_point = point_b;
            }

            while (temp_point != point_b)
            {
                if (!temp_point) //point_a 在 point_b 之后，则从 head_point 开始删除，并将 point_b 作为 head_point
                {
                    temp_point = head_point;
                    head_point = point_b;
                    
                    continue;
                }
                else if (temp_point == head_point) //主点链为环，且 head_point 在 point_b point_a 之间，则将 point_b 作为 head_point
                {
                    head_point = point_b;
                }

                delete_point = temp_point;
                temp_point = temp_point->next;
                delete delete_point;
            }
        }
        else if (point_a) //head_point 为空，point_a 不为空，则 point_a 作为 head_point，删除 point_a 之后的点
        {
            Point* temp_point;
            Point* delete_point;

            temp_point = point_a->next;
            point_a->next = point_b;
            head_point = point_a;

            while (temp_point && temp_point != point_a)
            {
                delete_point = temp_point;
                temp_point = temp_point->next;
                delete delete_point;
            }
        }
        else //head_point 为空，point_a 为空，则 point_b 作为 head_point
        {
            head_point = point_b;
        }
    }
    else //point_b 为空
    {
        if (head_point) //head_point 不为空
        {
            Point* temp_point;
            Point* delete_point;
            Point* former_head_point = head_point;
            
            if (point_a) //point_a 不为空，则从 point_a 的下一点开始删除，point_a 作为末尾点
            {
                temp_point = point_a->next;
                point_a->next = NULL;
            }
            else //point_a 为空，则删除整个点链
            {
                temp_point = head_point;
                head_point = NULL;
            }

            while (temp_point)
            {
                delete_point = temp_point;
                temp_point = temp_point->next;
                delete delete_point;

                if (temp_point == former_head_point) //为环形链表，将尾部 next 置空，使其变为单向链表
                    temp_point->next = NULL;
            }
        }
        else if (point_a) //head_point 为空，point_a 不为空，则 point_a 作为 head_point，删除point_a 之后的点
        {
            Point* temp_point;
            Point* delete_point;
            
            temp_point = point_a->next;
            point_a->next = NULL;
            head_point = point_a;

            while (temp_point)
            {
                delete_point = temp_point;
                temp_point = temp_point->next;
                delete delete_point;
            }
        }

        //head_point 为空，point_a 为空，无需进行操作
    }
}


//在 front_point 和 next_point 间插入新的点
//front_point 和 next_point 必须属于主点链
//将首先连接 front_point 和 next_point，具体规则参见 Connect 函数
//若 front_point 为空，则新建的 point 为初始点
//若 next_point 为空，则新建的 point 为末尾点
//若 front_point 和 next_point 均为空，则删除整个点链，新建的 point 为初始点
Point* Route::AddPoint(Point* front_point, int8_t x, int8_t y, uint8_t type, Point* next_point)
{
    Point* point = CreatePoint(x, y, type);

    //连接 front_point 和 next_point
    Connect(front_point, next_point);
    
    if (front_point)
        front_point->next = point;
    else head_point = point; //若 front_point 为空，则新建的 point 为初始点

    point->next = next_point;

    return point;
}


//生成基本路径点链
void Route::InitBaseRoute(void)
{    
    //基本路径数组
    const static int8_t base_route [][3] = 
    {
        {0, 0, NORMAL_POINT},
        {0, 1, NORMAL_POINT},
        {0, 2, NORMAL_POINT},
        {-1, 2, NORMAL_POINT},
        {-1, 1, NORMAL_POINT},
        {0, 1, NORMAL_POINT},
        {0, 0, NORMAL_POINT},
        {-1, 0, NORMAL_POINT},
        {0, 0, NORMAL_POINT},
        {0, 1, NORMAL_POINT},
        {0, 2, NORMAL_POINT},
        {-1, 2, NORMAL_POINT},
        {-1, 1, NORMAL_POINT},
        {0, 1, NORMAL_POINT},
        {0, 0, NORMAL_POINT},
        {-1, 0, NORMAL_POINT},
        {0, 0, NORMAL_POINT},
        {0, 1, NORMAL_POINT},
        {0, 2, NORMAL_POINT},
        {-1, 2, NORMAL_POINT},
        {-1, 1, NORMAL_POINT},
        {0, 1, NORMAL_POINT},
        {0, 0, NORMAL_POINT},
        {-1, 0, NORMAL_POINT}
    };

    //计算数组元素个数
    int route_amount = sizeof(base_route) / sizeof(base_route[0]);

    //生成基本路径点链
    Point* temp_point = NULL;
    for (int i = 0; i < route_amount; i++)
        temp_point = AddPoint(temp_point, base_route[i][X], base_route[i][Y], base_route[i][TYPE]);

    //生成环形点链
    Connect(temp_point, head_point);
}