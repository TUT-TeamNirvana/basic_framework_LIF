#include "power_meter.h"

static power_meter_data_t data;

void power_meter_init(void){
    INA226_SetConfig(0x4127);
    INA226_SetCalibrationReg(0x3200);
}

power_meter_data_t *get_power_meter_data(void){
    static float v;
    static float i;
    static float p;
    v=INA226_GetBusV();
    i=INA226_GetCurrent();
    p=INA226_GetPower();
    data.voltage=v;
    data.current=i;
    data.power=p;
    return &data;
}
