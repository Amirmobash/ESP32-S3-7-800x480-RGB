/* lv_conf.h - Minimal config for LVGL 8.x (ESP32-S3 RGB 800x480) */
#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*====================
   COLOR SETTINGS
 *====================*/
#define LV_COLOR_DEPTH              16
#define LV_COLOR_16_SWAP            0

/*====================
   MEMORY SETTINGS
 *====================*/
#define LV_MEM_CUSTOM               1
#define LV_MEM_SIZE                 (128U * 1024U)
#define LV_MEM_ATTR

/*====================
   HAL SETTINGS
 *====================*/
#define LV_TICK_CUSTOM              1
#define LV_TICK_CUSTOM_INCLUDE      "Arduino.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())

/*====================
   FEATURE CONFIG
 *====================*/
#define LV_USE_LOG                  0
#define LV_USE_PERF_MONITOR         0
#define LV_USE_ASSERT_NULL          0
#define LV_USE_ASSERT_MALLOC        0
#define LV_USE_ASSERT_STYLE         0
#define LV_USE_ASSERT_OBJ           0
#define LV_USE_ASSERT_SCREEN        0
#define LV_USE_USER_DATA            0

/*====================
   FONT USAGE
 *====================*/
#define LV_FONT_MONTSERRAT_14       1
#define LV_FONT_MONTSERRAT_16       1
#define LV_FONT_MONTSERRAT_18       1
#define LV_FONT_MONTSERRAT_20       1
#define LV_FONT_MONTSERRAT_22       1
#define LV_FONT_MONTSERRAT_24       1
#define LV_FONT_MONTSERRAT_28       1
#define LV_FONT_MONTSERRAT_36       1
#define LV_FONT_MONTSERRAT_48       1
#define LV_FONT_DEFAULT             &lv_font_montserrat_20

/*====================
   TEXT SETTINGS
 *====================*/
#define LV_TXT_ENC                  LV_TXT_ENC_UTF8
#define LV_USE_BIDI                 0
#define LV_USE_ARABIC_PERSIAN_CHARS 0

/*====================
   WIDGETS
 *====================*/
#define LV_USE_ARC                  1
#define LV_USE_BAR                  1
#define LV_USE_BTN                  1
#define LV_USE_BTNMATRIX            1
#define LV_USE_CALENDAR             1
#define LV_USE_CANVAS               1
#define LV_USE_CHECKBOX             1
#define LV_USE_DROPDOWN             1
#define LV_USE_IMG                  1
#define LV_USE_LABEL                1
#define LV_USE_LINE                 1
#define LV_USE_ROLLER               1
#define LV_USE_SLIDER               1
#define LV_USE_SWITCH               1
#define LV_USE_TEXTAREA             1
#define LV_USE_TABLE                1

/*====================
   LAYOUTS
 *====================*/
#define LV_USE_FLEX                 1
#define LV_USE_GRID                 1

/*====================
   OTHERS
 *====================*/
#define LV_USE_ANIMATION            1
#define LV_USE_SHADOW               1
#define LV_USE_OBJ_POS              1
#define LV_USE_GROUP                1
#define LV_USE_IMG_TRANSFORM        1
#define LV_USE_FILE_EXPLORER        0
#define LV_USE_THEME_DEFAULT        1
#define LV_USE_THEME_BASIC          1
#define LV_USE_THEME_MONO           1

#endif /*LV_CONF_H*/
