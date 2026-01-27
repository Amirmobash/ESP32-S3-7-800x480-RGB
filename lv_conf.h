#ifndef LV_CONF_H
#define LV_CONF_H

/* =====================
 *  Grund-Konfiguration
 * ===================== */
#define LV_COLOR_DEPTH 16
/* Für RGB Panels meistens 0 richtig. Wenn Farben vertauscht: 1 */
#define LV_COLOR_16_SWAP 0

/* Speicher für LVGL (bei PSRAM darf höher sein) */
#define LV_MEM_SIZE (128U * 1024U)

/* Logging (optional) */
#define LV_USE_LOG 0

/* =====================
 *  Fonts (UI groß, Industrie)
 * ===================== */
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_18 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_22 1
#define LV_FONT_MONTSERRAT_24 1
#define LV_FONT_MONTSERRAT_28 1

/* =====================
 *  Widgets (Standard)
 * ===================== */
#define LV_USE_LABEL 1
#define LV_USE_BTN 1
#define LV_USE_BAR 1
#define LV_USE_SLIDER 1
#define LV_USE_TEXTAREA 1
#define LV_USE_KEYBOARD 1

#endif /*LV_CONF_H*/
