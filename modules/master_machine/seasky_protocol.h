#ifndef __SEASKY_PROTOCOL_H
#define __SEASKY_PROTOCOL_H

#include <stdio.h>
#include <stdint.h>
// #include "master_process.h"

#define PROTOCOL_CMD_ID 0XA5
#define OFFSET_BYTE 8 // 出数据段外，其他部分所占字节数

typedef struct
{
	struct
	{
		uint8_t sof;
		uint16_t data_length;
		uint8_t crc_check; // 帧头CRC校验
	} header;			   // 数据帧头
	uint16_t cmd_id;	   // 数据ID
	uint16_t frame_tail;   // 帧尾CRC校验
} protocol_rm_struct;




#endif
