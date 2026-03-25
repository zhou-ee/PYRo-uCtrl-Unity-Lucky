#ifndef __PYRO_RC_CONTROLS_H__
#define __PYRO_RC_CONTROLS_H__

#include <cstdint>
#include "freertos.h"
#include "task.h"

namespace pyro {

/* ===================================================================
 * 1. 拨杆/挡位开关类 (Tiny Switch) - 内存占用: 1 Byte
 * =================================================================== */
enum class sw_pos_t : uint8_t { UNKNOWN = 0, UP = 1, MID = 2, DOWN = 3 };
enum class sw_event_t : uint8_t { NONE = 0, TO_UP, TO_MID, TO_DOWN };

class tiny_switch_t {
public:
    sw_pos_t current_pos : 4;
    sw_pos_t last_pos    : 4;
    
    tiny_switch_t() : current_pos(sw_pos_t::UNKNOWN), last_pos(sw_pos_t::UNKNOWN) {}

    void update(sw_pos_t new_pos) {
        if (new_pos != current_pos) {
            last_pos = current_pos;
            current_pos = new_pos;
            
            if (current_pos == sw_pos_t::UP) dispatch(sw_event_t::TO_UP);
            else if (current_pos == sw_pos_t::MID) dispatch(sw_event_t::TO_MID);
            else if (current_pos == sw_pos_t::DOWN) dispatch(sw_event_t::TO_DOWN);
        }
    }
private:
    void dispatch(sw_event_t ev); 
};

/* ===================================================================
 * 2. 独立按键类 (Tiny Button) - 内存占用: 4 Bytes
 * =================================================================== */
enum class btn_event_t : uint8_t {
    NONE = 0, 
    PRESS_DOWN, 
    PRESS_UP, 
    SINGLE_CLICK, 
    DOUBLE_CLICK, 
    MULTI_CLICK, 
    LONG_PRESS_START
};

class tiny_button_t {
public:
    uint16_t ticks = 0;             
    uint8_t  state          : 3;    
    uint8_t  debounce_cnt   : 3;    
    uint8_t  current_level  : 1;    
    uint8_t  active_level   : 1;    
    uint8_t  event          : 3;    
    uint8_t  repeat_cnt     : 3;    // 连击计数器 (最高支持7连击)
    uint8_t  cfg_multi_clk  : 1;    // 开启多击判定
    uint8_t  cfg_long_press : 1;    // 开启长按判定

    void init(bool active_high = true, bool enable_multi = false, bool enable_long = false) {
        active_level = active_high;
        cfg_multi_clk = enable_multi;
        cfg_long_press = enable_long;
        state = 0; ticks = 0; event = 0; repeat_cnt = 0; debounce_cnt = 0;
        current_level = !active_high;
    }

    void update(bool level) {
        event = static_cast<uint8_t>(btn_event_t::NONE);
        
        // 1 帧(14ms)物理消抖
        if (level != current_level) {
            if (++debounce_cnt >= 1) { 
                current_level = level;
                debounce_cnt = 0;
            }
        } else { debounce_cnt = 0; }

        switch (state) {
            case 0: // 【空闲态】
                if (current_level == active_level) {
                    dispatch(btn_event_t::PRESS_DOWN);
                    ticks = 0; 
                    repeat_cnt = 1; 
                    state = 1; 
                }
                break;
                
            case 1: // 【按下态】
                if (current_level != active_level) { 
                    dispatch(btn_event_t::PRESS_UP);
                    ticks = 0;
                    if (cfg_multi_clk) state = 2; 
                    else { 
                        dispatch(btn_event_t::SINGLE_CLICK); 
                        state = 0; 
                    }
                } else if (cfg_long_press && (++ticks >= 30)) { // 30帧 = 420ms
                    dispatch(btn_event_t::LONG_PRESS_START);
                    state = 3; 
                }
                break;
                
            case 2: // 【等待确认连击态】
                if (current_level == active_level) { 
                    dispatch(btn_event_t::PRESS_DOWN);
                    if (repeat_cnt < 7) repeat_cnt++; 
                    ticks = 0;
                    state = 1; // 跳回状态1，继续判定后续是长按还是松开
                } else if (++ticks >= 18) { // 18帧 = 252ms，超时确认
                    if (repeat_cnt == 1) dispatch(btn_event_t::SINGLE_CLICK);
                    else if (repeat_cnt == 2) dispatch(btn_event_t::DOUBLE_CLICK);
                    else dispatch(btn_event_t::MULTI_CLICK); 
                    state = 0;
                }
                break;
                
            case 3: // 【长按保持态】
                if (current_level != active_level) { 
                    dispatch(btn_event_t::PRESS_UP);
                    state = 0; 
                }
                break;
        }
    }
private:
    void dispatch(btn_event_t ev);
};

/* ===================================================================
 * 3. 泛型事件登记册 (RC Broker)
 * =================================================================== */
template <typename TargetType, typename EventType>
class rc_broker_t {
private:
    static constexpr uint8_t MAX_SUBS = 32;

    struct sub_info_t {
        TargetType* target_ptr;
        EventType target_ev;
        TaskHandle_t task;
        uint32_t bit;
    };

    // 【核心修复】利用 C++17 的 inline static 特性，直接在类内完成静态数组成员的初始化！
    inline static sub_info_t _subs[MAX_SUBS] = {};

public:
    static void subscribe(TargetType* target, EventType ev, TaskHandle_t task, uint32_t bit) {
        for (int i = 0; i < MAX_SUBS; ++i) {
            if (_subs[i].target_ptr == nullptr) {
                _subs[i] = {target, ev, task, bit};
                break;
            }
        }
    }

    static void publish(TargetType* target, EventType ev) {
        for (int i = 0; i < MAX_SUBS; ++i) {
            if (_subs[i].target_ptr == target && _subs[i].target_ev == ev) {
                xTaskNotify(_subs[i].task, _subs[i].bit, eSetBits);
            }
        }
    }
};

// 【核心修复】彻底删除之前在类外面写的那个 template <typename T, typename E> ... 变量实体定义

// 别名方便使用
using btn_broker = rc_broker_t<tiny_button_t, btn_event_t>;
using sw_broker  = rc_broker_t<tiny_switch_t, sw_event_t>;

// 派发函数实现
inline void tiny_button_t::dispatch(btn_event_t ev) { btn_broker::publish(this, ev); }
inline void tiny_switch_t::dispatch(sw_event_t ev)  { sw_broker::publish(this, ev); }

} // namespace pyro
#endif