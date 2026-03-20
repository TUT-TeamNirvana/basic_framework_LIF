#include "bsp_log.h"

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"

char printf_buf[256]={0};//用于rtt终端printf输出的缓存区
void BSPLogInit()
{
    SEGGER_RTT_Init();
}
//不能打印浮点类型的数据
int PrintLog(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int n = SEGGER_RTT_vprintf(BUFFER_INDEX, fmt, &args); // 一次可以开启多个buffer(多个终端),我们只用一个
    va_end(args);
    return n;
}
//通过这个函数将浮点类型的数据转换为字符串类型
void Float2Str(char *str, float va)
{
    int flag = va < 0;
    int head = (int)va;
    int point = (int)((va - head) * 1000);
    head = abs(head);
    point = abs(point);
    if (flag)
        sprintf(str, "-%d.%d", head, point);
    else
        sprintf(str, "%d.%d", head, point);
}

//rtt波形显示函数
/**
  * @brief          RTT波形打印(传入指针)，格式为富莱安H7-tool显示格式，参数为浮点数指针
  * @param[in]      num_args : 参数数目
  * @retval         none
  */
typedef float float32_t;
void RTT_PrintWave(int num_args, ...) {
    float32_t *param_point[num_args];
    int i;
    char buf[256];
    va_list arg;
    va_start(arg, num_args);
    int len = 0;
    for (i = 0; i < num_args; i++) {
        param_point[i] = va_arg(arg, float32_t *);

        if (i == (num_args - 1)) {
            len += sprintf((buf + len), "%.3f\r\n", *(float *) param_point[i]);
        } else {
            len += sprintf((buf + len), "%.3f,", *(float *) param_point[i]);
        }
    }
    va_end(arg);
    SEGGER_RTT_SetTerminal(0);
    SEGGER_RTT_WriteString(0, buf);
}

/**
  * @brief          RTT波形打印(非指针)，格式为富莱安H7-tool显示格式，参数为整形转浮点数非指针
  * @param[in]      num_args : 参数数目
  * @retval         none
  */
void RTT_PrintWave_np(int num_args, ...) {
    float32_t param_point[num_args];
    int i;
    char buf[256];
    va_list arg;
    va_start(arg, num_args);
    int len = 0;
    for (i = 0; i < num_args; i++) {
        param_point[i] = va_arg(arg, double);

        if (i == (num_args - 1)) {
            len += sprintf((buf + len), "%.3f\r\n", (float) param_point[i]);
        } else {
            len += sprintf((buf + len), "%.3f,", (float) param_point[i]);
        }
    }
    va_end(arg);
    SEGGER_RTT_SetTerminal(0);
    SEGGER_RTT_WriteString(0, buf);
}