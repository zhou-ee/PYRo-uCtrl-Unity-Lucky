#ifndef __CONFIG_H__
#define __CONFIG_H__
#include <cstdint>
#include "BMI088_driver.h"

constexpr float PITCH_MIN_RAD = -0.5f; // Pitch 轴最小角度 (rad)
constexpr float PITCH_MAX_RAD = 0.0f;  // Pitch 轴最大角度 (rad)

constexpr float YAW_OFFSET_RAD = -1.99801016f;

#endif
