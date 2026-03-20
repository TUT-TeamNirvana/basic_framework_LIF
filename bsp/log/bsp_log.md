# bsp_log

<p align='right'>neozng1@hnu.edu.cn</p>

## 使用说明

bsp_log是基于segger RTT实现的日志打印模块。

推荐使用`bsp_log.h`中提供了三级日志：

```c
#define LOGINFO(format,...)
#define LOGWARNING(format,...)
#define LOGERROR(format,...)
```

分别用于输出不同等级的日志。注意RTT不支持直接使用`%f`进行浮点格式化,要使用`void Float2Str(char *str, float va);`转化成字符串之后再发送。

**若想启用RTT，必须通过`launch.json`的`debug-jlink`启动调试（不论使用什么调试器）。** 按照`VSCode+Ozone环境配置`完成配置之后的cmsis dap和daplink是可以支持Jlink全家桶的。

另外，若你使用的是cmsis-dap和daplink，**请在 *jlink* 调试任务启动之后再打开`log`任务。**（均在项目文件夹下的.vsocde/task.json中，有注释自行查看）。否则可能出线RTT viewer无法连接客户端的情况。

在ozone中查看log输出，直接打开console调试任务台和terminal调试中断便可看到调试输出。

> 由于ozone版本的原因，可能出现日志不换行或没有颜色。

## 自定义输出

你也可以自定义输出格式，详见Segger RTT的文档。

```c
int printf_log(const char *fmt, ...);
void Float2Str(char *str, float va); // 输出浮点需要先用此函数进行转换
```

调用第一个函数，可以通过jlink或dap-link向调试器连接的上位机发送信息，格式和printf相同，示例如下：

```c
printf_log("Hello World!\n");
printf_log("Motor %d met some problem, error code %d!\n",3,1);
```

第二个函数可以将浮点类型转换成字符串以方便发送：

```c
    float a = -45.121;
    float b = 3.1415926f;
    char printf_buf[10];//这个变量已经在bsp_log.c中定义是全局变量不需要每次使用的时候在函数中声明本质上是储存字符串的内存缓存区
    Float2Str(printf_buf,a);
    PrintLog("a=%s\n",printf_buf);//默认显示在终端0上
    //或者用以下方式
    SEGGER_RTT_SetTerminal(1);//设置显示的终端
    sprintf(printf_buf,"absolute=%f,absolute_set=%f\r\n",a,b);
    SEGGER_RTT_WriteString(0, printf_buf);
```
## H7TOOL RTT使用说明
1、配置好wifi的账号密码
2、USB接线查看H7的网关和电脑的网关是不是一样的，需要保证H7的网关和电脑在一个网段。
3、选择wifi通信模式确保pc端的tool的ip地址和H7的ip地址在同一个网段。
4、TOOL的IP网段必须和电脑在一个网段。而且电脑不能打开虚拟网卡（关闭科学上网）。
5、重新打开上位机链接。
## rtt波形显示使用示例
```c
    float a = 0.145f;
    float b = -1.256f;
    RTT_PrintWave_np(2,a,b);//非指针参数

    float *c = a;
    float *d = b;
    RTT_PrintWave(2,c,d);//指针参数
    //或者RTT_PrintWave(2,&a,&b);
```
在任何一个文件使用关于rtt的函数都需要包含`bsp_log.h`头文件。