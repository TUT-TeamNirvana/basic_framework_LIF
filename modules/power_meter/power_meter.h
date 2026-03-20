#ifndef __POWER_METER_H
#define __POWER_METER_H

#include "bsp_iic.h"
#include "INA226.h"

typedef struct{
    float voltage;
    float current;
    float power;
}power_meter_data_t;
//包含power_meter.h文件后在application层对应的任务线程直接调用函数即可
void power_meter_init(void);
//包含power_meter.h文件后在application层定义power_meter_data_t结构体指针，对应的任务线程直接调用函数赋值指针即可
power_meter_data_t *get_power_meter_data(void);

#endif

