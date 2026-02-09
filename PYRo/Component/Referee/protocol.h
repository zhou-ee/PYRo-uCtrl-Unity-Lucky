#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include <cstdint>
#include <cstddef>


namespace pyro
{

constexpr uint8_t HEADER_SOF    = 0xA5;
constexpr size_t FRAME_MAX_SIZE = 256;
constexpr size_t HEADER_SIZE    = 5; // sizeof(FrameHeader)
constexpr size_t CMD_SIZE       = 2;
constexpr size_t CRC16_SIZE     = 2;
constexpr size_t HEADER_CRC_LEN = HEADER_SIZE + CRC16_SIZE;
constexpr size_t HEADER_CRC_CMDID_LEN =
    HEADER_SIZE + CRC16_SIZE + sizeof(uint16_t);
constexpr size_t HEADER_CMDID_LEN = HEADER_SIZE + sizeof(uint16_t);

#pragma pack(push, 1)

// 命令码 (enum class)
enum class CmdId : uint16_t
{
    // --- Rx: Server -> Robot ---
    GAME_STATE          = 0x0001, // 比赛状态数据
    GAME_RESULT         = 0x0002, // 比赛结果数据
    GAME_ROBOT_HP       = 0x0003, // 机器人血量数据
    FIELD_EVENTS        = 0x0101, // 场地事件数据
    REFEREE_WARNING     = 0x0104, // 裁判警告数据
    DART_INFO           = 0x0105, // 飞镖发射相关数据

    // Rx: 自主决策
    SENTRY_CMD          = 0x0120, // [V1.2] 哨兵自主决策指令
    RADAR_CMD           = 0x0121, // [V1.2] 雷达自主决策指令

    ROBOT_STATE         = 0x0201, // 机器人性能体系数据
    POWER_HEAT_DATA     = 0x0202, // 实时功率热量数据
    ROBOT_POS           = 0x0203, // 机器人位置
    BUFF_MUSK           = 0x0204, // 机器人增益
    AERIAL_ENERGY       = 0x0205, // 空中支援时间数据
    ROBOT_HURT          = 0x0206, // 伤害状态
    SHOOT_DATA          = 0x0207, // 实时射击信息
    BULLET_REMAINING    = 0x0208, // 子弹剩余发送数
    ROBOT_RFID          = 0x0209, // 机器人RFID状态
    DART_CLIENT_CMD     = 0x020A, // 飞镖选手端指令数据
    GROUND_ROBOT_POS    = 0x020B, // 地面机器人位置数据
    RADAR_MARK          = 0x020C, // 雷达标记进度数据
    SENTRY_INFO         = 0x020D, // 哨兵自主决策信息同步
    RADAR_INFO          = 0x020E, // 雷达自主决策信息同步

    // --- Tx/Rx: 交互 ---
    STUDENT_INTERACTIVE = 0x0301, // 机器人交互数据
    CUSTOM_CONTROLLER   = 0x0302, // [图传] 自定义控制器与机器人交互
    TINY_MAP_INTERACT   = 0x0303, // 选手端小地图交互数据
    MAP_RECEIVE_RADAR   = 0x0305, // 选手端小地图接收雷达数据
    CUSTOM_CLIENT_DATA  = 0x0306, // 自定义控制器与选手端交互数据
    MAP_RECEIVE_PATH    = 0x0307, // 选手端小地图接收路径数据
    MAP_RECEIVE_ROBOT   = 0x0308, // 选手端小地图接收机器人数据 (自定义UI消息)
};

// 交互子命令码范围 (enum class)
enum class InteractionSubCmd : uint16_t
{
    // 客户端 UI 绘制 (Server -> Client)
    UI_CMD_START     = 0x0100,
    UI_CMD_END       = 0x01FF,

    // 机器人间通信 (Robot -> Robot)
    ROBOT_COMM_START = 0x0200,
    ROBOT_COMM_END   = 0x02FF,
};

// 帧头结构
struct FrameHeader
{
    uint8_t sof;
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
};

/* ----------------- Structure Definitions ----------------- */

// 0x0001
struct GameStatus
{
    uint8_t game_type     : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t sync_timestamp;
};

// 0x0002
struct GameResult
{
    uint8_t winner;
};

// 0x0003
struct GameRobotHP
{
    uint16_t robot_1_hp;
    uint16_t robot_2_hp;
    uint16_t robot_3_hp;
    uint16_t robot_4_hp;
    uint16_t reserved_1;
    uint16_t robot_7_hp;
    uint16_t outpost_hp;
    uint16_t base_hp;
};

// 0x0101
struct EventData
{
    uint32_t event_type;
};

// 0x0104
struct RefereeWarning
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
};

// 0x0105
struct DartInfo
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
};

// 0x0201
struct RobotStatus
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_hp;
    uint16_t maximum_hp;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output  : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
};

// 0x0202
struct PowerHeatData
{
    uint16_t reserved_1;
    uint16_t reserved_2;
    float reserved_3;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
};

// 0x0203
struct RobotPos
{
    float x;
    float y;
    float angle;
    uint32_t reserved;
};

// 0x0204
struct BuffInfo
{
    uint8_t recovery_buff;
    uint16_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
};

// 0x0206
struct HurtData
{
    uint8_t armor_id            : 4;
    uint8_t hp_deduction_reason : 4;
};

// 0x0207
struct ShootData
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
};

// 0x0208
struct ProjectileAllowance
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
};

// 0x0209
struct RfidStatus
{
    uint32_t rfid_status;
    uint8_t rfid_status_2;
};

// 0x020A
struct DartClientCmd
{
    uint8_t dart_launch_opening_status;
    uint8_t reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
};

// 0x020B
struct GroundRobotPosition
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved_1;
    float reserved_2;
};

// 0x020C
struct RadarMarkData
{
    uint16_t mark_progress;
};

// 0x020D
struct SentryInfo
{
    uint32_t sentry_info;
    uint16_t sentry_info_2;
};

// 0x020E
struct RadarInfo
{
    uint8_t radar_info;
};

// 0x0303
struct MapCommand
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint16_t cmd_source;
};

// -----------------------------------------------------------
// TX Structs
// -----------------------------------------------------------

// 0x0301 Header
struct InteractionHeader
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
};

// 0x0301 Payload Wrapper
struct RobotInteractionData
{
    InteractionHeader header;
    uint8_t user_data[112];
};

// 0x0308
struct CustomInfo
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
};

// Main Data Holder
struct RefereeData
{
    GameStatus game_status;
    GameResult game_result;
    GameRobotHP game_robot_hp;
    EventData field_event;
    RefereeWarning referee_warning;
    DartInfo dart_info;

    RobotStatus robot_status;
    PowerHeatData power_heat;
    RobotPos robot_pos;
    BuffInfo buff;
    HurtData hurt;
    ShootData shoot;
    ProjectileAllowance allowance;
    RfidStatus rfid;
    DartClientCmd dart_client_cmd;
    GroundRobotPosition ground_robot_pos;
    RadarMarkData radar_mark;
    SentryInfo sentry_info;
    RadarInfo radar_info;

    MapCommand map_command;
    RobotInteractionData robot_interaction;
};

#pragma pack(pop)

} // namespace pyro


#endif // ROBOMASTER_PROTOCOL_H