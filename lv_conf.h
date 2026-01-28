#ifndef LV_CONF_H
#define LV_CONF_H

/* =====================
 *  Color settings
 * ===================== */
#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 0   /* اگر رنگ‌ها بهم ریخت، 1 کن */

/* =====================
 *  Memory settings
 * ===================== */
#define LV_MEM_SIZE (128U * 1024U)

/* =====================
 *  Logging
 * ===================== */
#define LV_USE_LOG 0

/* =====================
 *  Fonts (needed for UI)
 * ===================== */
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_18 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_22 1
#define LV_FONT_MONTSERRAT_24 1
#define LV_FONT_MONTSERRAT_28 1

/* =====================
 *  Widgets used
 * ===================== */
#define LV_USE_LABEL 1
#define LV_USE_BTN 1
#define LV_USE_BAR 1
#define LV_USE_SLIDER 1
#define LV_USE_TEXTAREA 1
#define LV_USE_KEYBOARD 1

#endif /* LV_CONF_H */
