# Power Meter 模块使用说明

## 模块概述

本模块用于读取基于INA226芯片的底盘功率计数据,提供实时功率监测功能。

## 硬件连接

| INA226引脚 | STM32连接      |
| ---------- | -------------- |
| VCC        | 3.3V           |
| GND        | GND            |
| SCL        | I2C_SCL(PB8)   |
| SDA        | I2C_SDA(PB9)   |
| ALERT      | 可配置中断引脚  |

## 快速使用

### 1. 模块初始化

```c
PowerMeter_Init();  // 在系统初始化时调用
```

### 2. 数据读取

```c
// 获取最新数据指针
const PowerData_t* pdata = PowerMeter_GetData();

// 数据结构包含:
typedef struct {
    float bus_voltage;   // 总线电压(V)
    float current;       // 电流(A) 
    float power;         // 功率(W)
    uint32_t timestamp;  // 时间戳(ms)
} PowerData_t;
```

## API说明

### 初始化函数

`void PowerMeter_Init(void)`

- 功能: 初始化I2C通信和INA226配置
- 参数: 无
- 注意: 需在main函数初始化阶段调用

### 数据更新函数

`void PowerMeter_Update(void)`

- 功能: 执行新的数据采集(需周期性调用)
- 参数: 无

### 数据获取函数

`const PowerData_t* PowerMeter_GetData(void)`

- 功能: 获取最新功率数据指针
- 返回: 指向最新PowerData_t数据的常量指针

## 示例代码

```c
// 在任务循环中调用
while(1) {
    PowerMeter_Update(); // 先更新数据
    const PowerData_t* pdata = PowerMeter_GetData();
    printf("总线电压: %.2fV\n", pdata->bus_voltage);
    printf("当前功率: %.2fW\n", pdata->power);
    osDelay(100);
}
```

## 注意事项

1. 确保I2C总线已正确初始化
2. 采样率配置为默认1kHz,如需修改请调整INA226_CONFIG宏定义
3. 功率计算精度依赖分流电阻阻值(默认为0.002Ω)
4. 建议数据读取间隔不小于10ms
