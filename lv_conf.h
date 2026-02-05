/* lv_conf.h - Minimal config for LVGL 8.x (ESP32-S3 RGB 800x480) */
#ifndef LV_CONF_H
#define LV_CONF_H

/*====================
   COLOR SETTINGS
 *====================*/
#define LV_COLOR_DEPTH              16
#define LV_COLOR_16_SWAP            0

/*====================
   MEMORY SETTINGS
 *====================*/
#define LV_MEM_CUSTOM               0
#define LV_MEM_SIZE                 (128U * 1024U)

/*====================
   HAL SETTINGS
 *====================*/
#define LV_TICK_CUSTOM              0

/*====================
   FEATURE CONFIG
 *====================*/
#define LV_USE_LOG                  0
#define LV_USE_PERF_MONITOR         0

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
#define LV_USE_BTN                  1
#define LV_USE_LABEL                1
#define LV_USE_BAR                  1
#define LV_USE_SLIDER               1
#define LV_USE_BTNMATRIX            1
#define LV_USE_MSGBOX               1

/*====================
   LAYOUTS
 *====================*/
#define LV_USE_FLEX                 1
#define LV_USE_GRID                 1

/*====================
   OTHERS
 *====================*/
#define LV_USE_ANIMATION            1
#define LV_USE_IMG                  1
#define LV_USE_THEME_DEFAULT        1

#endif /*LV_CONF_H*/
