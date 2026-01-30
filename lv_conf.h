#ifndef LV_CONF_H
#define LV_CONF_H

/* =====================
 *  LVGL v8.x CONFIG
 * ===================== */

#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 0

#define LV_MEM_CUSTOM 0
#define LV_MEM_SIZE (64U * 1024U)

#define LV_USE_LOG 0

#define LV_TICK_CUSTOM 1
#define LV_TICK_CUSTOM_INCLUDE "Arduino.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR ((uint32_t)millis())

#define LV_DPI_DEF 130

/* Draw buffers */
#define LV_DISP_DEF_REFR_PERIOD 30

/* Fonts */
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_28 1
#define LV_FONT_DEFAULT &lv_font_montserrat_20

/* Widgets (حداقل‌های لازم) */
#define LV_USE_LABEL 1
#define LV_USE_BTN 1
#define LV_USE_BAR 1
#define LV_USE_SLIDER 1
#define LV_USE_TEXTAREA 1
#define LV_USE_KEYBOARD 1

/* Themes */
#define LV_USE_THEME_DEFAULT 1

/* Animations */
#define LV_USE_ANIMIMG 0

#endif /*LV_CONF_H*/
