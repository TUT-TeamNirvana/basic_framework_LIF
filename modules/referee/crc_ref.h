#ifndef __CRC_H_
#define __CRC_H_

// 裁判系统官方CRC校验,LUT和module/algorithms中的不同,后续需要统一实现crc,提供8/16/32的支持

#include <stdint.h>

#define TRUE 1
#define FALSE 0
// CRC8
void append_crc8_check_sum(uint8_t *pchMessage, uint16_t dwLength);
uint8_t verify_crc8_check_sum(uint8_t *pchMessage, uint16_t dwLength);
uint8_t get_crc8_check_sum(uint8_t *pchMessage, uint16_t dwLength, uint8_t ucCRC8);

// CRC16
void append_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength);
uint8_t verify_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength);
uint16_t get_crc16_check_sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

#endif
