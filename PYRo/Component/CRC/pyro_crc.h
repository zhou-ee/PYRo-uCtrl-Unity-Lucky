#ifndef __PYRO_CORE_PROTOCOL_H__
#define __PYRO_CORE_PROTOCOL_H__

#include <stdint.h>

// 原有 CRC16 声明
extern uint8_t verify_crc16_check_sum(uint8_t const* p_msg, uint16_t len);

// 新增 CRC8 声明，保持命名风格一致
extern uint8_t verify_crc8_check_sum(uint8_t const* p_msg, uint16_t len);

#endif