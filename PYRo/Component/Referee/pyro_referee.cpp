/**
 * @file pyro_referee.cpp
 * @brief RoboMaster Referee System Driver (Modern C++ Style Implementation)
 */

#include "pyro_referee.h"
#include "pyro_crc.h"
#include "pyro_dwt_drv.h"

#include <cstring> // for memcpy, strlen

namespace pyro
{


// ==========================================================================
// Task Implementation
// ==========================================================================

void referee_system::referee_task::init()
{
    // Task-specific initialization if needed
}

void referee_system::referee_task::run_loop()
{
    while (true)
    {
        if (_parent)
        {
            _parent->unpack_fifo_data();

            // 2s timeout check
            if (dwt_drv_t::get_timeline_ms() - _parent->_last_update_time >
                2000)
            {
                _parent->_is_online = false;
            }
        }
        vTaskDelay(1);
    }
}

// ==========================================================================
// Driver Singleton & Constructor
// ==========================================================================

referee_system *referee_system::get_instance()
{
    static referee_system instance(
        uart_drv_t::get_instance(uart_drv_t::which_uart::uart1));
    return &instance;
}

referee_system::referee_system(uart_drv_t *uart_handle)
    : _uart(uart_handle), _task(nullptr), _data{}, _unpack_obj{}, _send_seq(0),
      _robot_id(0), _is_online(false), _last_update_time(0)
{
    fifo_s_init(&_fifo, _fifo_buf, FIFO_BUF_LEN);
    memset(_tx_buffer, 0, MAX_TX_FRAME_LEN);

    _task = new referee_task(this);
}

// ==========================================================================
// Init
// ==========================================================================

void referee_system::init(const std::initializer_list<CmdId> listening_ids)
{
    if (!_uart)
        return;

    _enabled_ids.reset();
    for (auto id : listening_ids)
    {
        const uint16_t id_val = static_cast<uint16_t>(id);
        if (id_val < MAX_CMD_ID_COUNT)
        {
            _enabled_ids.set(id_val);
        }
    }

    _uart->add_rx_event_callback(
        [this](uint8_t *p, const uint16_t size,
               const BaseType_t task_woken) -> bool
        { return this->rx_callback(p, size, task_woken); },
        reinterpret_cast<uint32_t>(this));
    _uart->enable_rx_dma();

    if (_task)
        _task->start();
}

void referee_system::init()
{
    if (!_uart)
        return;

    _enabled_ids.set(); // Enable all

    _uart->add_rx_event_callback(
        [this](uint8_t *p, const uint16_t size,
               const BaseType_t task_woken) -> bool
        { return this->rx_callback(p, size, task_woken); },
        reinterpret_cast<uint32_t>(this));
    _uart->enable_rx_dma();

    if (_task)
        _task->start();
}

// ==========================================================================
// Logic
// ==========================================================================

uint16_t referee_system::get_client_id() const
{
    return _robot_id + 0x0100;
}

bool referee_system::send_packet(CmdId cmd_id, const void *data,
                                 uint16_t const len)
{
    // C++ Cast: static_cast for enum to int
    const uint16_t cmd_val         = static_cast<uint16_t>(cmd_id);

    const uint16_t frame_total_len = HEADER_CMDID_LEN + len + CRC16_SIZE;
    if (frame_total_len > MAX_TX_FRAME_LEN)
        return false;
    if (!_uart)
        return false;

    // C++ Cast: reinterpret_cast for byte buffer manipulation
    auto *p_header        = reinterpret_cast<FrameHeader *>(_tx_buffer);

    p_header->sof         = HEADER_SOF;
    p_header->data_length = len;
    p_header->seq         = _send_seq++;
    p_header->crc8        = 0;

    append_crc8_check_sum(reinterpret_cast<uint8_t *>(p_header), HEADER_SIZE);

    // Pointer arithmetic handled with proper casting
    uint8_t *p_cmd_start = _tx_buffer + HEADER_SIZE;
    auto *p_cmd_id       = reinterpret_cast<uint16_t *>(p_cmd_start);
    *p_cmd_id            = cmd_val;

    if (len > 0 && data != nullptr)
    {
        memcpy(_tx_buffer + HEADER_CMDID_LEN, data, len);
    }

    append_crc16_check_sum(_tx_buffer, frame_total_len);
    return (_uart->write(_tx_buffer, frame_total_len) == PYRO_OK);
}

bool referee_system::_send_interaction_packet_base(const uint16_t sub_cmd_id,
                                                   const uint16_t receiver_id,
                                                   const void *data,
                                                   const uint16_t len)
{
    // Payload max check: 128 - 9 (Frame overhead) - 6 (Interact Header) = 113
    // Safety buffer used: 112
    if (len + sizeof(InteractionHeader) > 119)
        return false;

    uint8_t buffer[128]; // Use stack buffer
    auto *p_header        = reinterpret_cast<InteractionHeader *>(buffer);

    p_header->data_cmd_id = sub_cmd_id;
    p_header->sender_id   = _robot_id;
    p_header->receiver_id = receiver_id;

    if (len > 0 && data != nullptr)
    {
        memcpy(buffer + sizeof(InteractionHeader), data, len);
    }

    return send_packet(CmdId::STUDENT_INTERACTIVE, buffer,
                       sizeof(InteractionHeader) + len);
}

bool referee_system::send_robot_interaction(const uint16_t receiver_id,
                                            const uint16_t sub_cmd_id,
                                            const void *data,
                                            const uint16_t len)
{
    if (_robot_id == 0)
        return false;

    const bool is_my_team_red = (_robot_id < 100);
    const bool is_target_red  = (receiver_id < 100);

    if (is_my_team_red != is_target_red)
        return false;

    return _send_interaction_packet_base(sub_cmd_id, receiver_id, data, len);
}

bool referee_system::send_ui_interaction(const uint16_t sub_cmd_id,
                                         const void *data, const uint16_t len)
{
    return _send_interaction_packet_base(sub_cmd_id, get_client_id(), data,
                                         len);
}

bool referee_system::send_custom_info(const char *message)
{
    if (_robot_id == 0)
        return false;

    CustomInfo custom_data;
    custom_data.sender_id   = _robot_id;
    custom_data.receiver_id = get_client_id();

    memset(custom_data.user_data, 0, sizeof(custom_data.user_data));

    size_t str_len = std::strlen(message);
    if (str_len > 30)
        str_len = 30;

    memcpy(custom_data.user_data, message, str_len);

    return send_packet(CmdId::MAP_RECEIVE_ROBOT, &custom_data,
                       sizeof(CustomInfo));
}

// ==========================================================================
// RX Implementation
// ==========================================================================

bool referee_system::rx_callback(uint8_t *p, const uint16_t size,
                                 BaseType_t xHigherPriorityTaskWoken)
{
    // FIFO expects char*
    fifo_s_puts(&_fifo, reinterpret_cast<char *>(p), size);
    return true;
}

void referee_system::unpack_fifo_data()
{
    uint8_t byte = 0;

    // Use local references for cleaner code
    auto &step   = _unpack_obj.step;
    auto &idx    = _unpack_obj.index;
    auto &packet = _unpack_obj.protocol_packet;
    auto &len    = _unpack_obj.data_len;

    while (fifo_s_used(&_fifo))
    {
        byte = static_cast<uint8_t>(fifo_s_get(&_fifo));

        switch (step)
        {
            case unpack_step::HEADER_SOF:
                if (byte == HEADER_SOF)
                {
                    step          = unpack_step::LENGTH_LOW;
                    idx           = 0;
                    packet[idx++] = byte;
                }
                else
                {
                    idx = 0;
                }
                break;

            case unpack_step::LENGTH_LOW:
                len           = byte;
                packet[idx++] = byte;
                step          = unpack_step::LENGTH_HIGH;
                break;

            case unpack_step::LENGTH_HIGH:
                len |= (static_cast<uint16_t>(byte) << 8);
                packet[idx++] = byte;

                if (len < (FRAME_MAX_SIZE - HEADER_CRC_CMDID_LEN))
                    step = unpack_step::FRAME_SEQ;
                else
                {
                    step = unpack_step::HEADER_SOF;
                    idx  = 0;
                }
                break;

            case unpack_step::FRAME_SEQ:
                packet[idx++] = byte;
                step          = unpack_step::HEADER_CRC8;
                break;

            case unpack_step::HEADER_CRC8:
                packet[idx++] = byte;

                if (idx == HEADER_SIZE)
                {
                    if (verify_crc8_check_sum(packet, HEADER_SIZE))
                        step = unpack_step::DATA_CRC16;
                    else
                    {
                        step = unpack_step::HEADER_SOF;
                        idx  = 0;
                    }
                }
                break;

            case unpack_step::DATA_CRC16:
                if (idx < (HEADER_CRC_CMDID_LEN + len))
                {
                    packet[idx++] = byte;
                }

                if (idx >= (HEADER_CRC_CMDID_LEN + len))
                {
                    step = unpack_step::HEADER_SOF;
                    idx  = 0; // Prepare for next frame

                    if (verify_crc16_check_sum(packet,
                                               HEADER_CRC_CMDID_LEN + len))
                    {
                        solve_data(packet);
                    }
                }
                break;
        }
    }
}

void referee_system::solve_data(const uint8_t *frame)
{
    auto *p_header = reinterpret_cast<const FrameHeader *>(frame);
    const uint16_t data_length =
        p_header->data_length; // 获取协议发来的实际长度

    uint16_t cmd_id_val = 0;
    uint8_t index       = HEADER_SIZE;

    // Extract CMD_ID
    memcpy(&cmd_id_val, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    _is_online        = true;
    _last_update_time = dwt_drv_t::get_timeline_ms();

    // White-list check
    if (cmd_id_val >= MAX_CMD_ID_COUNT || !_enabled_ids.test(cmd_id_val))
        return;

    // Convert integer to Enum Class for switch-case
    auto cmd = static_cast<CmdId>(cmd_id_val);

    switch (cmd)
    {
        case CmdId::GAME_STATE:
            safe_copy(_data.game_status, frame + index, data_length);
            break;
        case CmdId::GAME_RESULT:
            safe_copy(_data.game_result, frame + index, data_length);
            break;
        case CmdId::GAME_ROBOT_HP:
            safe_copy(_data.game_robot_hp, frame + index, data_length);
            break;
        case CmdId::FIELD_EVENTS:
            safe_copy(_data.field_event, frame + index, data_length);
            break;
        case CmdId::REFEREE_WARNING:
            safe_copy(_data.referee_warning, frame + index, data_length);
            break;
        case CmdId::DART_INFO:
            safe_copy(_data.dart_info, frame + index, data_length);
            break;
        case CmdId::ROBOT_STATE:
            safe_copy(_data.robot_status, frame + index, data_length);
            break;
        case CmdId::POWER_HEAT_DATA:
            safe_copy(_data.power_heat, frame + index, data_length);
            break;
        case CmdId::ROBOT_POS:
            safe_copy(_data.robot_pos, frame + index, data_length);
            break;
        case CmdId::BUFF_MUSK:
            safe_copy(_data.buff, frame + index, data_length);
            break;
        case CmdId::ROBOT_HURT:
            safe_copy(_data.hurt, frame + index, data_length);
            break;
        case CmdId::SHOOT_DATA:
            safe_copy(_data.shoot, frame + index, data_length);
            break;
        case CmdId::BULLET_REMAINING:
            safe_copy(_data.allowance, frame + index, data_length);
            break;
        case CmdId::ROBOT_RFID:
            safe_copy(_data.rfid, frame + index, data_length);
            break;
        case CmdId::DART_CLIENT_CMD:
            safe_copy(_data.dart_client_cmd, frame + index, data_length);
            break;
        case CmdId::GROUND_ROBOT_POS:
            safe_copy(_data.ground_robot_pos, frame + index, data_length);
            break;
        case CmdId::RADAR_MARK:
            safe_copy(_data.radar_mark, frame + index, data_length);
            break;
        case CmdId::SENTRY_INFO:
            safe_copy(_data.sentry_info, frame + index, data_length);
            break;
        case CmdId::RADAR_INFO:
            safe_copy(_data.radar_info, frame + index, data_length);
            break;
        case CmdId::TINY_MAP_INTERACT:
            safe_copy(_data.map_command, frame + index, data_length);
            break;
        case CmdId::STUDENT_INTERACTIVE:
            safe_copy(_data.robot_interaction, frame + index, data_length);
            break;

        default:
            break;
    }
}

} // namespace pyro