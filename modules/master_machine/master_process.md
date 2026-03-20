# master_process

- 模块功能：
  - 负责与视觉模块进行通信，支持 UART 和 VCP 两种模式。
  - 提供数据打包、解包、发送和接收功能。
  - 包含离线回调函数以处理视觉模块离线情况。
```
master_process.c
├── 头文件包含（条件编译）
├── 静态变量定义
│   ├── 公共变量：send_frame, recv_frame, vision_daemon
│   ├── UART 专用：vision_usart
│   └── VCP 专用：vis_recv_buff
├── 宏定义
├── 公共函数
│   ├── pack_send_frame()      - 打包帧（添加\n）
│   ├── unpack_recv_frame()    - 解包帧（不检查\n）
│   └── VisionOfflineCallback() - 离线回调
├── UART 模式专用
│   ├── VisionRxCallback()     - 串口接收回调
│   ├── VisionInit()           - 初始化（UART）
│   └── VisionSend()           - 发送（UART）
├── VCP 模式专用
│   ├── DecodeVision()         - USB 接收回调
│   ├── VisionInit()           - 初始化（VCP）
│   └── VisionSend()           - 发送（VCP）
└── 公共接口
└── VisionGetRecvData()    - 获取接收数据
```
- 电控 → 视觉（发送路径）：
```
RobotCMDTask (200Hz)
  ↓
VisionSend(&vision_send)
  ↓
pack_send_frame()  // 添加帧头、CRC16、帧尾\n
  ↓
USBTransmit(send_buff, tx_len)
  ↓
CDC_Transmit_FS(buffer, len)  // usbd_cdc_if.c L286
  ↓
USBD_CDC_SetTxBuffer() + USBD_CDC_TransmitPacket()
  ↓
USB IN 端点 → PC 视觉程序
```

- 视觉 → 电控（接收路径）：
```
PC 视觉程序 → USB OUT 端点
  ↓
CDC_Receive_FS()  // usbd_cdc_if.c L262
  ↓
UserRxBufferFS[]  // 数据存入接收缓冲区
  ↓
rx_cbk(*Len)  // 调用回调函数 DecodeVision
  ↓
unpack_recv_frame()  // 解包、验证 CRC16
  ↓
recv_frame  // 存储到全局变量
```