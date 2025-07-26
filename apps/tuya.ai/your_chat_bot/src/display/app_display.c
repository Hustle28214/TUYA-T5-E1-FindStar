/**
 * @file app_display.c
 * @brief Handle display initialization and message processing
 *
 * This source file provides the implementation for initializing the display system,
 * creating a message queue, and handling display messages in a separate task.
 * It includes functions to initialize the display, send messages to the display,
 * and manage the display task.
 *
 * @copyright Copyright (c) 2021-2025 Tuya Inc. All Rights Reserved.
 *
 */

 #include "tuya_cloud_types.h"

 #include "app_display.h"
 #include "tuya_lvgl.h"
 
 #include "tal_log.h"
 #include "tal_queue.h"
 #include "tal_thread.h"
 
 #include "tkl_memory.h"
 
 #include "lvgl.h"
 
 // 直接包含背景图实现文件
 #include "Compass/background.c"
 
 /***********************************************************
 ************************macro define************************
 ***********************************************************/
 
 /***********************************************************
 ***********************typedef define***********************
 ***********************************************************/
 typedef struct {
     TY_DISPLAY_TYPE_E type;
     int len;
     char *data;
 } DISPLAY_MSG_T;
 
 typedef struct {
     QUEUE_HANDLE queue_hdl;
     THREAD_HANDLE thrd_hdl;
 } TUYA_DISPLAY_T;
 
 /***********************************************************
 ***********************variable define**********************
 ***********************************************************/
 static TUYA_DISPLAY_T sg_display = {0};
 static lv_obj_t *sg_background_img = NULL;
 
 /***********************************************************
 ***********************function define**********************
 ***********************************************************/
 
 static void __app_display_msg_handle(DISPLAY_MSG_T *msg_data)
 {
     if (msg_data == NULL) {
         return;
     }
     
     if (msg_data->data) {
         tkl_system_psram_free(msg_data->data);
         msg_data->data = NULL;
     }
 }
 
 static void __display_task(void *args)
 {
     (void)args;
 
     tuya_lvgl_mutex_lock();
     
     // 创建背景图对象
     sg_background_img = lv_image_create(lv_scr_act());
     if (!sg_background_img) {
         PR_ERR("Failed to create background image object");
     } else {
         // 设置背景图并居中
         lv_image_set_src(sg_background_img, &background);
         lv_obj_align(sg_background_img, LV_ALIGN_CENTER, 0, 0);
         
         // 设置图片缩放模式
         lv_img_set_zoom(sg_background_img, 256); // 256 = 100% 缩放
         lv_image_set_rotation(sg_background_img, 0);
     }
     
 #if defined(BOARD_CHOICE_WAVESHARE_ESP32_S3_TOUCH_AMOLED_1_8)
     extern void lcd_sh8601_set_backlight(uint8_t brightness);
     lcd_sh8601_set_backlight(80); // set backlight to 80%
 #endif
     
     tuya_lvgl_mutex_unlock();
     PR_DEBUG("Display init success");
 
     for (;;) {
         DISPLAY_MSG_T msg_data = {0};
         tal_queue_fetch(sg_display.queue_hdl, &msg_data, 0xFFFFFFFF);
 
         __app_display_msg_handle(&msg_data);
     }
 }
 
 /**
  * @brief Initialize the display system
  *
  * @param None
  * @return OPERATE_RET Initialization result, OPRT_OK indicates success
  */
 OPERATE_RET app_display_init(void)
 {
     OPERATE_RET rt = OPRT_OK;
 
     memset(&sg_display, 0, sizeof(TUYA_DISPLAY_T));
 
     // lvgl initialization
     TUYA_CALL_ERR_RETURN(tuya_lvgl_init());
     PR_DEBUG("lvgl init success");
 
     TUYA_CALL_ERR_RETURN(tal_queue_create_init(&sg_display.queue_hdl, sizeof(DISPLAY_MSG_T), 8));
     THREAD_CFG_T cfg = {
         .thrdname = "display_task",
         .priority = THREAD_PRIO_2,
         .stackDepth = 1024 * 4,
     };
     TUYA_CALL_ERR_RETURN(tal_thread_create_and_start(&sg_display.thrd_hdl, NULL, NULL, __display_task, NULL, &cfg));
     PR_DEBUG("Display task created");
 
     return rt;
 }
 
 /**
  * @brief Send display message to the display system
  *
  * @param tp Type of the display message
  * @param data Pointer to the message data
  * @param len Length of the message data
  * @return OPERATE_RET Result of sending the message, OPRT_OK indicates success
  */
 OPERATE_RET app_display_send_msg(TY_DISPLAY_TYPE_E tp, uint8_t *data, int len)
 {
     DISPLAY_MSG_T msg_data;
 
     msg_data.type = tp;
     msg_data.len = len;
     if (len && data != NULL) {
         msg_data.data = (char *)tkl_system_psram_malloc(len + 1);
         if (NULL == msg_data.data) {
             return OPRT_MALLOC_FAILED;
         }
         memcpy(msg_data.data, data, len);
         msg_data.data[len] = 0; //"\0"
     } else {
         msg_data.data = NULL;
     }
 
     tal_queue_post(sg_display.queue_hdl, &msg_data, 0xFFFFFFFF);
 
     return OPRT_OK;
 }