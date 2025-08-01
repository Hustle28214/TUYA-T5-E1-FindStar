# 基于本项目开展罗盘界面的开发

1. config文件夹

TUYA_T5AI_BOARD_EYES.config:

```yaml
CONFIG_PROJECT_VERSION="1.0.1"
CONFIG_TUYA_PRODUCT_KEY="8ixyalzpn0jrun9y"
CONFIG_ENABLE_CHAT_DISPLAY=y
CONFIG_ENABLE_GUI_EYES=y
CONFIG_BOARD_CHOICE_T5AI=y
CONFIG_TUYA_T5AI_BOARD_EX_MODULE_EYES=y
CONFIG_MEM_SIZE=51200
CONFIG_MEMP_NUM_UDP_PCB=10
CONFIG_MEMP_NUM_TCP_SEG=80
CONFIG_PBUF_LINK_ENCAPSULATION_HLEN=96
CONFIG_TCP_SND_BUF=32768
CONFIG_TCP_SND_QUEUELEN=44
CONFIG_MEMP_NUM_NETBUF=32
CONFIG_DEFAULT_UDP_RECVMBOX_SIZE=24
CONFIG_MEMP_NUM_SYS_TIMEOUT=12
CONFIG_LWIP_EAPOL_SUPPORT=0
CONFIG_LWIP_TX_PBUF_ZERO_COPY=0
CONFIG_CONFIG_TUYA_SOCK_SHIM=0
CONFIG_LWIP_DHCPC_STATIC_IPADDR_ENABLE=1
CONFIG_ETHARP_SUPPORT_STATIC_ENTRIES=1
CONFIG_LWIP_NETIF_STATUS_CALLBACK=1
CONFIG_LWIP_TIMEVAL_PRIVATE=0
CONFIG_IN_ADDR_T_DEFINED=y
CONFIG_ENABLE_LIBLVGL=y
```

存放配置文件

2. include文件夹

app_chat_bot.h:

```c
/**
 * @file app_chat_bot.h
 * @brief app_chat_bot module is used to
 * @version 0.1
 * @date 2025-03-25
 */

#ifndef __APP_CHAT_BOT_H__
#define __APP_CHAT_BOT_H__

#include "tuya_cloud_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************
************************macro define************************
***********************************************************/

/***********************************************************
***********************typedef define***********************
***********************************************************/

/***********************************************************
********************function declaration********************
***********************************************************/
OPERATE_RET app_chat_bot_init(void);

uint8_t app_chat_bot_get_enable(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CHAT_BOT_H__ */

```

对应app_chat_box.c: 

```c
/**
 * @file app_chat_bot.c
 * @brief 聊天机器人应用模块实现
 * 
 * 该模块负责管理聊天机器人的核心功能，包括：
 * 1. 音频处理与AI交互
 * 2. 不同聊天模式的管理
 * 3. 硬件控制（LED和按钮）
 * 4. 显示消息传递
 * 
 * @version 0.1
 * @date 2025-03-25
 */

#include "netmgr.h"              // 网络管理
#include "tkl_wifi.h"            // WiFi硬件抽象层
#include "tkl_gpio.h"            // GPIO硬件抽象层
#include "tkl_memory.h"          // 内存管理
#include "tal_api.h"             // Tuya抽象层API
#include "tuya_ringbuf.h"        // 环形缓冲区

// 条件编译：按钮功能支持
#if defined(ENABLE_BUTTON) && (ENABLE_BUTTON == 1)
#include "tdl_button_manage.h"   // 按钮管理
#endif

// 条件编译：LED功能支持
#if defined(ENABLE_LED) && (ENABLE_LED == 1)
#include "tdl_led_manage.h"      // LED管理
#endif

#include "app_display.h"         // 显示应用
#include "ai_audio.h"            // AI音频处理
#include "app_chat_bot.h"        // 聊天机器人应用头文件

/***********************************************************
************************宏定义******************************
***********************************************************/
#define AI_AUDIO_TEXT_BUFF_LEN (1024)    ///< AI文本缓冲区大小
#define AI_AUDIO_TEXT_SHOW_LEN (60 * 3)  ///< AI文本显示长度限制

/// 聊天模式枚举类型
typedef uint8_t APP_CHAT_MODE_E;

/// 按键长按模式：按住按钮开始单次对话
#define APP_CHAT_MODE_KEY_PRESS_HOLD_SINGLE 0

/// 按键触发模式：单击按钮开始/结束自由对话
#define APP_CHAT_MODE_KEY_TRIG_VAD_FREE 1

/// 语音唤醒模式：说出唤醒词开始单次对话，20秒内无对话需重新唤醒
#define APP_CHAT_MODE_ASR_WAKEUP_SINGLE 2

/// 自由唤醒模式：说出唤醒词开始自由对话，20秒内无对话需重新唤醒
#define APP_CHAT_MODE_ASR_WAKEUP_FREE 3

#define APP_CHAT_MODE_MAX 4  ///< 最大聊天模式数量

/***********************************************************
***********************类型定义*****************************
***********************************************************/
/**
 * @brief 聊天工作模式信息结构体
 * 
 * 包含聊天模式的各种配置信息
 */
typedef struct {
    APP_CHAT_MODE_E mode;          ///< 聊天模式
    AI_AUDIO_WORK_MODE_E auido_mode; ///< 音频工作模式
    AI_AUDIO_ALERT_TYPE_E mode_alert; ///< 模式提示音类型
    char *display_text;            ///< 显示文本
    bool is_open;                  ///< 是否开启
} CHAT_WORK_MODE_INFO_T;

/**
 * @brief 聊天机器人应用结构体
 * 
 * 包含聊天机器人的全局状态信息
 */
typedef struct {
    uint8_t is_enable;             ///< 是否启用
    const CHAT_WORK_MODE_INFO_T *work; ///< 当前工作模式
} APP_CHAT_BOT_S;

/***********************************************************
***********************常量声明*****************************
***********************************************************/
/// 按键长按模式配置
const CHAT_WORK_MODE_INFO_T cAPP_WORK_HOLD = {
    .mode = APP_CHAT_MODE_KEY_PRESS_HOLD_SINGLE,
    .auido_mode = AI_AUDIO_MODE_MANUAL_SINGLE_TALK,
    .mode_alert = AI_AUDIO_ALERT_LONG_KEY_TALK,
    .display_text = NULL,
    .is_open = true,
};

/// 按键触发模式配置
const CHAT_WORK_MODE_INFO_T cAPP_WORK_TRIG_VAD = {
    .mode = APP_CHAT_MODE_KEY_TRIG_VAD_FREE,
    .auido_mode = AI_AUDIO_WORK_VAD_FREE_TALK,
    .mode_alert = AI_AUDIO_ALERT_KEY_TALK,
    .display_text = NULL,
    .is_open = false,
};

/// 语音唤醒单次模式配置
const CHAT_WORK_MODE_INFO_T cAPP_WORK_WAKEUP_SINGLE = {
    .mode = APP_CHAT_MODE_ASR_WAKEUP_SINGLE,
    .auido_mode = AI_AUDIO_WORK_ASR_WAKEUP_SINGLE_TALK,
    .mode_alert = AI_AUDIO_ALERT_WAKEUP_TALK,
    .display_text = NULL,
    .is_open = true,
};

/// 语音唤醒自由模式配置
const CHAT_WORK_MODE_INFO_T cAPP_WORK_WAKEUP_FREE = {
    .mode = APP_CHAT_MODE_ASR_WAKEUP_FREE,
    .auido_mode = AI_AUDIO_WORK_ASR_WAKEUP_FREE_TALK,
    .mode_alert = AI_AUDIO_ALERT_FREE_TALK,
    .display_text = NULL,
    .is_open = true,
};

/***********************************************************
***********************变量定义*****************************
***********************************************************/
/// 全局聊天机器人状态实例
static APP_CHAT_BOT_S sg_chat_bot = {
// 根据编译配置选择工作模式
#if defined(ENABLE_CHAT_MODE_KEY_PRESS_HOLD_SINGEL) && (ENABLE_CHAT_MODE_KEY_PRESS_HOLD_SINGEL == 1)
    .work = &cAPP_WORK_HOLD,
#endif

#if defined(ENABLE_CHAT_MODE_KEY_TRIG_VAD_FREE) && (ENABLE_CHAT_MODE_KEY_TRIG_VAD_FREE == 1)
    .work = &cAPP_WORK_TRIG_VAD,
#endif

#if defined(ENABLE_CHAT_MODE_ASR_WAKEUP_SINGEL) && (ENABLE_CHAT_MODE_ASR_WAKEUP_SINGEL == 1)
    .work = &cAPP_WORK_WAKEUP_SINGLE,
#endif

#if defined(ENABLE_CHAT_MODE_ASR_WAKEUP_FREE) && (ENABLE_CHAT_MODE_ASR_WAKEUP_FREE == 1)
    .work = &cAPP_WORK_WAKEUP_FREE,
#endif
};

// 条件编译：LED支持
#if defined(ENABLE_LED) && (ENABLE_LED == 1)
static TDL_LED_HANDLE_T sg_led_hdl = NULL;  ///< LED句柄
#endif

// 条件编译：按钮支持
#if defined(ENABLE_BUTTON) && (ENABLE_BUTTON == 1)
static TDL_BUTTON_HANDLE sg_button_hdl = NULL;  ///< 按钮句柄
#endif

/***********************************************************
***********************函数实现*****************************
***********************************************************/

/**
 * @brief AI音频事件通知回调函数
 * 
 * 处理来自AI音频模块的各种事件，包括：
 * - 人类语音识别文本
 * - AI回复文本开始/数据/结束
 * - AI回复表情
 * - ASR唤醒事件
 * 
 * @param event 事件类型
 * @param data 事件数据
 * @param len 数据长度
 * @param arg 回调参数
 */
static void __app_ai_audio_evt_inform_cb(AI_AUDIO_EVENT_E event, uint8_t *data, uint32_t len, void *arg)
{
// 条件编译：显示支持
#if defined(ENABLE_CHAT_DISPLAY) && (ENABLE_CHAT_DISPLAY == 1)
// 条件编译：流式AI文本支持
#if !defined(ENABLE_GUI_STREAM_AI_TEXT) || (ENABLE_GUI_STREAM_AI_TEXT != 1)
    static uint8_t *p_ai_text = NULL;  ///< AI文本缓冲区指针
    static uint32_t ai_text_len = 0;   ///< AI文本长度
#endif
#endif

    switch (event) {
    case AI_AUDIO_EVT_HUMAN_ASR_TEXT:  // 人类语音识别文本事件
        if (len > 0 && data) {
// 发送ASR文本到显示
#if defined(ENABLE_CHAT_DISPLAY) && (ENABLE_CHAT_DISPLAY == 1)
            app_display_send_msg(TY_DISPLAY_TP_USER_MSG, data, len);
#endif
        }
        break;
    
    case AI_AUDIO_EVT_AI_REPLIES_TEXT_START:  // AI回复文本开始事件
#if defined(ENABLE_CHAT_DISPLAY) && (ENABLE_CHAT_DISPLAY == 1)
#if defined(ENABLE_GUI_STREAM_AI_TEXT) && (ENABLE_GUI_STREAM_AI_TEXT == 1)
        // 流式文本：开始显示
        app_display_send_msg(TY_DISPLAY_TP_ASSISTANT_MSG_STREAM_START, data, len);
#else
        // 非流式文本：分配缓冲区
        if (NULL == p_ai_text) {
            p_ai_text = tkl_system_psram_malloc(AI_AUDIO_TEXT_BUFF_LEN);
            if (NULL == p_ai_text) {
                return;
            }
        }
        ai_text_len = 0;  // 重置文本长度
#endif
#endif
        break;
    
    case AI_AUDIO_EVT_AI_REPLIES_TEXT_DATA:  // AI回复文本数据事件
#if defined(ENABLE_CHAT_DISPLAY) && (ENABLE_CHAT_DISPLAY == 1)
#if defined(ENABLE_GUI_STREAM_AI_TEXT) && (ENABLE_GUI_STREAM_AI_TEXT == 1)
        // 流式文本：发送数据片段
        app_display_send_msg(TY_DISPLAY_TP_ASSISTANT_MSG_STREAM_DATA, data, len);
#else
        // 非流式文本：缓存数据
        memcpy(p_ai_text + ai_text_len, data, len);
        ai_text_len += len;
        
        // 达到显示长度限制时发送
        if (ai_text_len >= AI_AUDIO_TEXT_SHOW_LEN) {
            app_display_send_msg(TY_DISPLAY_TP_ASSISTANT_MSG, p_ai_text, ai_text_len);
            ai_text_len = 0;
        }
#endif
#endif
        break;
    
    case AI_AUDIO_EVT_AI_REPLIES_TEXT_END:  // AI回复文本结束事件
#if defined(ENABLE_CHAT_DISPLAY) && (ENABLE_CHAT_DISPLAY == 1)
#if defined(ENABLE_GUI_STREAM_AI_TEXT) && (ENABLE_GUI_STREAM_AI_TEXT == 1)
        // 流式文本：结束显示
        app_display_send_msg(TY_DISPLAY_TP_ASSISTANT_MSG_STREAM_END, data, len);
#else
        // 非流式文本：发送剩余数据
        app_display_send_msg(TY_DISPLAY_TP_ASSISTANT_MSG, p_ai_text, ai_text_len);
        ai_text_len = 0;
#endif
#endif
        break;
    
    case AI_AUDIO_EVT_AI_REPLIES_EMO:  // AI回复表情事件
        {
            AI_AUDIO_EMOTION_T *emo = (AI_AUDIO_EMOTION_T *)data;
            PR_DEBUG("---> AI_MSG_TYPE_EMOTION");
            
            if (emo) {
                if (emo->name) {
                    PR_DEBUG("emotion name:%s", emo->name);
#if defined(ENABLE_CHAT_DISPLAY) && (ENABLE_CHAT_DISPLAY == 1)
                    // 发送表情名称到显示
                    app_display_send_msg(TY_DISPLAY_TP_EMOTION, (uint8_t *)emo->name, strlen(emo->name));
#endif
                }
                
                if (emo->text) {
                    PR_DEBUG("emotion text:%s", emo->text);
                }
            }
        }
        break;
    
    case AI_AUDIO_EVT_ASR_WAKEUP:  // ASR唤醒事件
#if defined(ENABLE_LED) && (ENABLE_LED == 1)
        {
            // LED闪烁指示唤醒
            TDL_LED_BLINK_CFG_T blink_cfg = {
                .cnt = 2,
                .start_stat = TDL_LED_ON,
                .end_stat = TDL_LED_OFF,
                .first_half_cycle_time = 100,
                .latter_half_cycle_time = 100,
            };
            tdl_led_blink(sg_led_hdl, &blink_cfg);
        }
#endif

#if defined(ENABLE_GUI_STREAM_AI_TEXT) && (ENABLE_GUI_STREAM_AI_TEXT == 1)
        // 确保流式文本结束
        app_display_send_msg(TY_DISPLAY_TP_ASSISTANT_MSG_STREAM_END, data, len);
#endif
        break;

    default:
        break;
    }
}

/**
 * @brief AI音频状态通知回调函数
 * 
 * 处理AI音频模块的状态变化，控制LED等硬件状态
 * 
 * @param state 新的音频状态
 */
static void __app_ai_audio_state_inform_cb(AI_AUDIO_STATE_E state)
{
    PR_DEBUG("ai audio state: %d", state);

    switch (state) {
    case AI_AUDIO_STATE_STANDBY:  // 待机状态
#if defined(ENABLE_LED) && (ENABLE_LED == 1)
        tdl_led_set_status(sg_led_hdl, TDL_LED_OFF);  // 关闭LED
#endif
        break;
    
    case AI_AUDIO_STATE_LISTEN:   // 监听状态
#if defined(ENABLE_LED) && (ENABLE_LED == 1)
        tdl_led_set_status(sg_led_hdl, TDL_LED_ON);   // 开启LED
#endif
        break;
    
    case AI_AUDIO_STATE_UPLOAD:   // 上传状态
        // 可添加特定处理
        break;
    
    case AI_AUDIO_STATE_AI_SPEAK: // AI说话状态
        // 可添加特定处理
        break;

    default:
        break;
    }
}

/**
 * @brief 启用或禁用聊天机器人
 * 
 * @param enable 1-启用，0-禁用
 * @return OPERATE_RET 操作结果
 */
static OPERATE_RET __app_chat_bot_enable(uint8_t enable)
{
    // 状态未变化时直接返回
    if (sg_chat_bot.is_enable == enable) {
        PR_DEBUG("chat bot enable is already %s", enable ? "enable" : "disable");
        return OPRT_OK;
    }

    PR_DEBUG("chat bot enable set %s", enable ? "enable" : "disable");

    // 设置AI音频启用状态
    ai_audio_set_open(enable);

    // 更新状态
    sg_chat_bot.is_enable = enable;

    return OPRT_OK;
}

/**
 * @brief 获取聊天机器人启用状态
 * 
 * @return uint8_t 1-启用，0-禁用
 */
uint8_t app_chat_bot_get_enable(void)
{
    return sg_chat_bot.is_enable;
}

// 条件编译：按钮功能支持
#if defined(ENABLE_BUTTON) && (ENABLE_BUTTON == 1)
/**
 * @brief 按钮功能回调函数
 * 
 * 处理各种按钮事件，根据当前聊天模式执行相应操作
 * 
 * @param name 按钮名称
 * @param event 按钮事件
 * @param argc 回调参数
 */
static void __app_button_function_cb(char *name, TDL_BUTTON_TOUCH_EVENT_E event, void *argc)
{
    APP_CHAT_MODE_E work_mode = sg_chat_bot.work->mode;
    PR_DEBUG("app button function cb, work mode: %d", work_mode);

    // 检查网络状态
    netmgr_status_e status = NETMGR_LINK_DOWN;
    netmgr_conn_get(NETCONN_AUTO, NETCONN_CMD_STATUS, &status);
    if (status == NETMGR_LINK_DOWN) {
        PR_DEBUG("network is down, ignore button event");
        
        // 如果正在播放音频，直接返回
        if (ai_audio_player_is_playing()) {
            return;
        }
        
        // 播放网络未激活提示音
        ai_audio_player_play_alert(AI_AUDIO_ALERT_NOT_ACTIVE);
        return;
    }

    // 处理不同按钮事件
    switch (event) {
    case TDL_BUTTON_PRESS_DOWN:  // 按钮按下
        if (work_mode == APP_CHAT_MODE_KEY_PRESS_HOLD_SINGLE) {
            PR_DEBUG("button press down, listen start");
#if defined(ENABLE_LED) && (ENABLE_LED == 1)
            tdl_led_set_status(sg_led_hdl, TDL_LED_ON);  // 开启LED
#endif
            ai_audio_manual_start_single_talk();  // 开始单次对话
        }
        break;
    
    case TDL_BUTTON_PRESS_UP:  // 按钮释放
        if (work_mode == APP_CHAT_MODE_KEY_PRESS_HOLD_SINGLE) {
            PR_DEBUG("button press up, listen end");
#if defined(ENABLE_LED) && (ENABLE_LED == 1)
            tdl_led_set_status(sg_led_hdl, TDL_LED_OFF);  // 关闭LED
#endif
            ai_audio_manual_stop_single_talk();  // 结束单次对话
        }
        break;
    
    case TDL_BUTTON_PRESS_SINGLE_CLICK:  // 单击
        if (work_mode == APP_CHAT_MODE_KEY_PRESS_HOLD_SINGLE) {
            break;  // 长按模式下忽略单击
        }

        if (sg_chat_bot.is_enable) {
            ai_audio_set_wakeup();  // 设置唤醒状态
        } else {
            __app_chat_bot_enable(true);  // 启用聊天机器人
        }
        PR_DEBUG("button single click");
        break;
    
    default:
        break;
    }
}

/**
 * @brief 初始化按钮功能
 * 
 * @return OPERATE_RET 操作结果
 */
static OPERATE_RET __app_open_button(void)
{
    OPERATE_RET rt = OPRT_OK;

    // 按钮配置
    TDL_BUTTON_CFG_T button_cfg = {
        .long_start_valid_time = 3000,    // 长按有效时间
        .long_keep_timer = 1000,          // 长按保持时间
        .button_debounce_time = 50,       // 消抖时间
        .button_repeat_valid_count = 2,    // 重复有效次数
        .button_repeat_valid_time = 500    // 重复有效时间
    };
    
    // 创建按钮
    TUYA_CALL_ERR_RETURN(tdl_button_create(BUTTON_NAME, &button_cfg, &sg_button_hdl));

    // 注册按钮事件回调
    tdl_button_event_register(sg_button_hdl, TDL_BUTTON_PRESS_DOWN, __app_button_function_cb);
    tdl_button_event_register(sg_button_hdl, TDL_BUTTON_PRESS_UP, __app_button_function_cb);
    tdl_button_event_register(sg_button_hdl, TDL_BUTTON_PRESS_SINGLE_CLICK, __app_button_function_cb);
    tdl_button_event_register(sg_button_hdl, TDL_BUTTON_PRESS_DOUBLE_CLICK, __app_button_function_cb);

    return rt;
}
#endif

/**
 * @brief 聊天机器人初始化函数
 * 
 * 初始化整个聊天机器人系统，包括：
 * - 显示模块
 * - AI音频模块
 * - 按钮功能（如果启用）
 * - LED功能（如果启用）
 * 
 * @return OPERATE_RET 操作结果
 */
OPERATE_RET app_chat_bot_init(void)
{
    OPERATE_RET rt = OPRT_OK;
    AI_AUDIO_CONFIG_T ai_audio_cfg;

    // 初始化显示模块
#if defined(ENABLE_CHAT_DISPLAY) && (ENABLE_CHAT_DISPLAY == 1)
    app_display_init();
#endif

    // 配置AI音频模块
    ai_audio_cfg.work_mode = sg_chat_bot.work->auido_mode;
    ai_audio_cfg.evt_inform_cb = __app_ai_audio_evt_inform_cb;
    ai_audio_cfg.state_inform_cb = __app_ai_audio_state_inform_cb;

    // 初始化AI音频模块
    TUYA_CALL_ERR_RETURN(ai_audio_init(&ai_audio_cfg));

    // 初始化按钮功能
#if defined(ENABLE_BUTTON) && (ENABLE_BUTTON == 1)
    TUYA_CALL_ERR_RETURN(__app_open_button());
#endif

    // 初始化LED功能
#if defined(ENABLE_LED) && (ENABLE_LED == 1)
    sg_led_hdl = tdl_led_find_dev(LED_NAME);  // 查找LED设备
    TUYA_CALL_ERR_RETURN(tdl_led_open(sg_led_hdl));  // 打开LED
#endif

    // 设置初始启用状态
    __app_chat_bot_enable(sg_chat_bot.work->is_open);

    return OPRT_OK;
}
```
app_display.h:

```c
/**
 * @file app_display.h
 * @brief Tuya 显示系统头文件
 *
 * 该头文件提供了初始化显示系统和向显示系统发送消息的声明。包含了与显示功能交互所需的
 * 数据类型和函数原型。它定义了显示消息的类型、表情符号标识以及网络状态表示方法。
 *
 * @copyright Copyright (c) 2021-2025 Tuya Inc. All Rights Reserved.
 *
 */

#ifndef __APP_DISPLAY_H__
#define __APP_DISPLAY_H__

#include "tuya_cloud_types.h"  // Tuya 云平台类型定义

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************
************************宏定义******************************
***********************************************************/
// 定义 WiFi 网络状态枚举
typedef uint8_t UI_WIFI_STATUS_E;

/// WiFi 断开状态
#define UI_WIFI_STATUS_DISCONNECTED 0

/// WiFi 信号良好状态
#define UI_WIFI_STATUS_GOOD         1

/// WiFi 信号一般状态
#define UI_WIFI_STATUS_FAIR         2

/// WiFi 信号弱状态
#define UI_WIFI_STATUS_WEAK         3

// 定义表情符号标识
#define EMOJI_NEUTRAL      "NEUTRAL"      ///< 中性表情
#define EMOJI_SAD          "SAD"          ///< 悲伤表情
#define EMOJI_ANGRY        "ANGRY"        ///< 生气表情
#define EMOJI_SURPRISE     "SURPRISE"     ///< 惊讶表情
#define EMOJI_CONFUSED     "CONFUSED"     ///< 困惑表情
#define EMOJI_THINKING     "THINKING"     ///< 思考表情
#define EMOJI_HAPPY        "HAPPY"        ///< 开心表情
#define EMOJI_TOUCH        "TOUCH"        ///< 触摸表情
#define EMOJI_FEARFUL      "FEARFUL"      ///< 害怕表情
#define EMOJI_DISAPPOINTED "DISAPPOINTED" ///< 失望表情
#define EMOJI_ANNOYED      "ANNOYED"      ///< 烦恼表情

/***********************************************************
***********************类型定义*****************************
***********************************************************/
/**
 * @brief 显示消息类型枚举
 * 
 * 定义了可以发送到显示系统的各种消息类型
 */
typedef enum {
    TY_DISPLAY_TP_USER_MSG,                   ///< 用户消息
    TY_DISPLAY_TP_ASSISTANT_MSG,              ///< 助手消息（完整）
    TY_DISPLAY_TP_ASSISTANT_MSG_STREAM_START, ///< 助手消息流式传输开始
    TY_DISPLAY_TP_ASSISTANT_MSG_STREAM_DATA,  ///< 助手消息流式传输数据
    TY_DISPLAY_TP_ASSISTANT_MSG_STREAM_END,   ///< 助手消息流式传输结束
    TY_DISPLAY_TP_SYSTEM_MSG,                 ///< 系统消息

    TY_DISPLAY_TP_EMOTION,                    ///< 表情消息

    // 状态栏相关消息类型
    TY_DISPLAY_TP_STATUS,                     ///< 状态信息
    TY_DISPLAY_TP_NOTIFICATION,               ///< 通知信息
    TY_DISPLAY_TP_NETWORK,                    ///< 网络状态
    TY_DISPLAY_TP_CHAT_MODE,                  ///< 聊天模式

    TY_DISPLAY_TP_MAX                         ///< 最大消息类型数量（内部使用）
} TY_DISPLAY_TYPE_E;

/***********************************************************
********************函数声明********************************
***********************************************************/

/**
 * @brief 初始化显示系统
 *
 * 该函数负责初始化整个显示系统，包括：
 * - 设置显示硬件
 * - 初始化显示驱动
 * - 创建必要的显示对象
 * - 设置默认显示状态
 *
 * @note 此函数应在系统启动时调用一次
 * 
 * @return OPERATE_RET 初始化结果
 *         - OPRT_OK: 初始化成功
 *         - 其他错误码: 初始化失败
 */
OPERATE_RET app_display_init(void);

/**
 * @brief 向显示系统发送消息
 *
 * 该函数用于向显示系统发送各种类型的消息，包括：
 * - 用户和助手的聊天消息
 * - 表情状态更新
 * - 网络状态更新
 * - 系统通知
 * - 聊天模式变更
 *
 * @param[in] tp 消息类型 (TY_DISPLAY_TYPE_E 枚举值)
 * @param[in] data 指向消息数据的指针
 * @param[in] len 消息数据的长度
 * 
 * @note 对于文本消息，data 应为 UTF-8 编码的字符串
 * @note 对于表情消息，data 应为表情标识符字符串
 * @note 对于流式消息，应按照 START -> DATA... -> END 的顺序发送
 *
 * @return OPERATE_RET 发送结果
 *         - OPRT_OK: 发送成功
 *         - OPRT_INVALID_PARM: 参数无效
 *         - OPRT_COM_ERROR: 通信错误
 */
OPERATE_RET app_display_send_msg(TY_DISPLAY_TYPE_E tp, uint8_t *data, int len);

#ifdef __cplusplus
}
#endif

#endif /* __APP_DISPLAY_H__ */
```

reset_netcfg.h:

```c
/**
 * @file reset_netcfg.h
 * @brief 物联网设备网络配置重置功能接口
 *
 * 该头文件定义了物联网设备网络配置重置功能的接口，包括启动重置过程、检查重置状态等功能。
 * 实现支持与涂鸦物联网平台集成，确保正确处理网络配置重置操作。该文件对于需要健壮网络配置
 * 重置机制的物联网应用开发至关重要。
 *
 * @copyright Copyright (c) 2021-2025 Tuya Inc. All Rights Reserved.
 */

#ifndef __RESET_NETCFG_H__
#define __RESET_NETCFG_H__

#include "tuya_cloud_types.h"  // 涂鸦云平台类型定义

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************
************************宏定义******************************
***********************************************************/
// 此文件当前未定义宏，保留此部分以便未来扩展

/***********************************************************
***********************类型定义*****************************
***********************************************************/
// 此文件当前未定义类型，保留此部分以便未来扩展

/***********************************************************
********************函数声明********************************
***********************************************************/

/**
 * @brief 启动网络配置重置过程
 *
 * 此函数初始化设备网络配置的重置过程。主要功能包括：
 * 1. 清除现有的网络设置
 * 2. 恢复网络配置到出厂默认状态
 * 3. 准备设备进行重新配置
 * 
 * @note 调用此函数后，设备将断开当前网络连接
 * @note 重置完成后，设备将进入配网模式
 * 
 * @return int 操作结果
 *   - 0: 重置过程成功启动
 *   - 负数: 启动失败，具体错误码见实现
 */
int reset_netconfig_start(void);

/**
 * @brief 检查网络配置重置状态
 *
 * 此函数用于检查网络配置重置过程的状态：
 * 1. 确认重置过程是否完成
 * 2. 验证重置操作是否成功
 * 3. 获取重置过程的进度信息
 * 
 * @note 在调用 reset_netconfig_start() 后使用此函数检查状态
 * @note 重置过程可能需要几秒到几十秒，建议轮询此函数
 * 
 * @return int 重置状态
 *   - 0: 重置成功完成
 *   - 正数: 重置进行中（可能表示进度百分比）
 *   - 负数: 重置失败或错误状态
 */
int reset_netconfig_check(void);

#ifdef __cplusplus
}
#endif

#endif /* __RESET_NETCFG_H__ */
```

tuya_config.h:

```c
/**
 * @file tuya_config.h
 * @brief IoT specific configuration file
 */

#ifndef TUYA_CONFIG_H_
#define TUYA_CONFIG_H_

#include "tuya_cloud_types.h"

/**
 * @brief configure the product information
 *
 * TUYA_PRODUCT_ID: PID, create on the Tuya IoT platform
 * TUYA_DEVICE_UUID: UUID, the unique ID of the device, you can get 2 free from Tuya IoT platform
 * TUYA_DEVICE_AUTHKEY: AUTHKEY, the private key of the device, you can get 2 free from Tuya IoT platform
 *
 * detail please refer to: https://developer.tuya.com/cn/docs/iot-device-dev/application-creation?id=Kbxw7ket3aujc
 *
 * warning: please replace these production information with your product id and license, otherwise the demo cannot
 * work.
 *
 */

#ifndef TUYA_PRODUCT_ID
#define TUYA_PRODUCT_ID "8ixyalzpn0jrun9y"
#endif

#define TUYA_OPENSDK_UUID    "uuid1719b8ae1af30df8"             // Please change the correct uuid
#define TUYA_OPENSDK_AUTHKEY "BG4vrOlPggpE8ohu2e1J1r2jW4nU5ihS" // Please change the correct authkey

/**
 * @brief PINCODE for AP provisioning
 *
 * TUYA_NETCFG_PINCODE: a random PINCODE for AP provisioning, PINCODE was generated BY TUYA PMS system!!!
 *
 * WARNING: PINCODE is mandatory for AP provisioning
 */
// #define TUYA_NETCFG_PINCODE   "69832860"

#endif

```

3. src/display/image/eyes128

