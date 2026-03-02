#ifndef __CONFIG_H__
#define __CONFIG_H__
#include <cstdint>

constexpr float TRACK_SPACING         = 0.456f; // 履带中心距 (m)
constexpr float MEC_WHEELBASE         = 0.421f; // 麦轮轴距
constexpr float MEC_FRONT_TRACK_WIDTH = 0.41f;  // 麦轮前轮距
constexpr float MEC_REAR_TRACK_WIDTH  = 0.41f;  // 麦轮后轮距
constexpr float WHEEL_RADIUS          = 0.076f; // 轮子半径 (m)
constexpr float TRACK_RADIUS          = 0.025f; // 履带驱动半径 (m)
constexpr float LEG_MIN_POS           = 0.15f;  // 腿部最小位置 (rad)
constexpr float LEG_MAX_POS           = 1.65f;  // 腿部最大位置
constexpr float LEG_POS_BUFFER_RAD    = 0.1f;   // 腿部位置缓冲 (rad)
constexpr float LEG_MAX_TORQUE        = 27.0f;  // 腿部最大输出扭矩 (N*m)
constexpr float LEG_K_WALL            = 300.0f; // 虚拟墙弹性系数 (N*m/rad)
constexpr float LEG_D_WALL            = 20.0f;  // 虚拟墙阻尼系数 (N*m*s/rad)
constexpr float YAW_OFFSET_RAD        = 0.796136022f;
constexpr uint32_t JX_POLY_DEGREE     = 7; // JX 多项式拟合阶数
constexpr uint32_t JY_POLY_DEGREE     = 5; // JY 多项式拟合阶数
constexpr uint32_t XB_POLY_DEGREE     = 6; // XB 多项式拟合阶数
constexpr uint32_t YB_POLY_DEGREE     = 4; // YB 多项式拟合阶数
constexpr float JX_POLY_COEF[JX_POLY_DEGREE + 1] = {
    -0.0742, 0.3986, -0.8740, 0.9990,
    -0.6561, 0.2485, -0.0279, -0.0072}; // JX多项式拟合系数，单位m/rad
constexpr float JY_POLY_COEF[JY_POLY_DEGREE + 1] = {
    -0.0194, 0.1161,  -0.2662,
    0.3140,  -0.2409, 0.2048}; // JY多项式拟合系数，单位m/rad
constexpr float XB_POLY_COEF[XB_POLY_DEGREE + 1] = {
    -0.0086, 0.0379,  -0.0700, 0.0618,
    -0.0154, -0.0062, -0.0011}; // XB多项式拟合系数，单位m
constexpr float YB_POLY_COEF[YB_POLY_DEGREE + 1] = {
    -0.0056, 0.0311, -0.0793, 0.1957, 0.0745}; // YB多项式拟合系数，单位m
constexpr float MASS         = 15.2f;          // 机器人质量 (kg)
constexpr float GRAVITY      = 9.81f;          // 重力加速度 (m/s^2)
constexpr float DIST_FRONT   = 0.2295f;        // 质心到前轴距离 (m)
constexpr float DIST_HIP     = 0.193f;         // 质心到髋关节的水平距离
constexpr float H_COG        = 0.15f;          // 重心垂直高度
constexpr float H_HIP_OFFSET = 0.074f;         // 髋关节与前轮轴心的垂直落差
inline float LEFT_LEG_OFFSET_RAD =
    1.3483417f; // 左腿位置偏移 (rad)，正值表示向前偏移
inline float RIGHT_LEG_OFFSET_RAD =
    1.13099241f; // 右腿位置偏移 (rad)，正值表示向前偏移

#endif
