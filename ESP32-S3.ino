/****************************************************
 * Bandware Zähler – ESP32-S3 + 7" 800x480 RGB + LVGL
 * UI: Deutsch, Orange/Weiß, industriell, animiert
 * Funktionen:
 *  - Zielmenge (1..1000) eingeben (On-Screen Keypad)
 *  - START/STOP/RESET
 *  - Zählen über Sensor (PC817 -> GPIO, open-collector)
 *  - Motor EIN/AUS (MOSFET/SSR-DC)
 *  - Entprellung (ms)
 *  - Zustandsmaschine (IDLE/RUNNING/DONE/STOPPED/ERROR)
 *  - Persistenz (letzte Zielmenge, Debounce, Helligkeit)
 *  - Failsafe & Watchdog: bei UI-Hänger -> Motor AUS
 *
 * Hinweis:
 *  - RGB Pinmapping & Timings sind boardabhängig.
 *  - In CONFIG: DISPLAY_PRESET auswählen/Pinmap anpassen.
 ****************************************************/

#include <Arduino.h>
#include <Preferences.h>

#include <lvgl.h>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>

// Optional Watchdog (ESP-IDF)
#include "esp_task_wdt.h"

// ========================= CONFIG =========================

// ---- IO Pins (freie GPIOs wählen, die nicht vom RGB Bus belegt sind!) ----
static const int GPIO_MOTOR  = 17;   // HIGH = Motor ON
static const int GPIO_SENSOR = 18;   // Sensorpuls (PC817 OUT) -> Interrupt FALLING

// Zielbereich
static const int TARGET_MIN = 1;
static const int TARGET_MAX = 1000;

// Display
static const int SCREEN_W = 800;
static const int SCREEN_H = 480;
static const int LVGL_BUF_LINES = 40; // mehr = flüssiger, braucht mehr RAM

// Display Preset Auswahl:
// 0 = Preset A (häufiges 7" ESP32-S3 HMI "Sunton-Style")
// 1 = Preset B (eigene Pins/Timings)
static const int DISPLAY_PRESET = 0;

// Touch (GT911) I2C Config (falls abweichend, hier ändern)
static const int TOUCH_I2C_SDA = 19;
static const int TOUCH_I2C_SCL = 20;
static const int TOUCH_RST_PIN = 38;   // manche Boards haben Reset-Pin
static const int TOUCH_INT_PIN = -1;   // oft NC

// Backlight Pin (falls abweichend, hier ändern)
static const int BL_PIN = 2;

// Default Parameter (werden aus NVS überschrieben)
static uint32_t debounce_ms = 3;
static uint8_t  brightness = 220; // 0..255

// Watchdog / Failsafe
static const uint32_t UI_HEARTBEAT_TIMEOUT_MS = 1500; // wenn LVGL nicht "lebt" -> Motor AUS
static const bool ENABLE_WDT = true;

// ==========================================================


// =============== Display Driver (LovyanGFX RGB) ===============
class LGFX : public lgfx::LGFX_Device {
public:
  lgfx::Bus_RGB     bus;
  lgfx::Panel_RGB   panel;
  lgfx::Light_PWM   light;
  lgfx::Touch_GT911 touch;

  LGFX() {
    // Panel
    {
      auto cfg = panel.config();
      cfg.memory_width  = SCREEN_W;
      cfg.memory_height = SCREEN_H;
      cfg.panel_width   = SCREEN_W;
      cfg.panel_height  = SCREEN_H;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      panel.config(cfg);
    }
    {
      auto cfg = panel.config_detail();
      cfg.use_psram = 1;
      panel.config_detail(cfg);
    }

    // RGB Bus
    {
      auto cfg = bus.config();
      cfg.panel = &panel;

      if (DISPLAY_PRESET == 0) {
        // ---------- PRESET A (sehr häufiges 7" ESP32-S3 HMI 800x480) ----------
        // D0..D15 (RGB565) + DE/VSYNC/HSYNC/PCLK
        cfg.pin_d0  = GPIO_NUM_15;
        cfg.pin_d1  = GPIO_NUM_7;
        cfg.pin_d2  = GPIO_NUM_6;
        cfg.pin_d3  = GPIO_NUM_5;
        cfg.pin_d4  = GPIO_NUM_4;
        cfg.pin_d5  = GPIO_NUM_9;
        cfg.pin_d6  = GPIO_NUM_46;
        cfg.pin_d7  = GPIO_NUM_3;
        cfg.pin_d8  = GPIO_NUM_8;
        cfg.pin_d9  = GPIO_NUM_16;
        cfg.pin_d10 = GPIO_NUM_1;
        cfg.pin_d11 = GPIO_NUM_14;
        cfg.pin_d12 = GPIO_NUM_21;
        cfg.pin_d13 = GPIO_NUM_47;
        cfg.pin_d14 = GPIO_NUM_48;
        cfg.pin_d15 = GPIO_NUM_45;

        cfg.pin_henable = GPIO_NUM_41; // DE
        cfg.pin_vsync   = GPIO_NUM_40; // VSYNC
        cfg.pin_hsync   = GPIO_NUM_39; // HSYNC
        cfg.pin_pclk    = GPIO_NUM_42; // PCLK

        cfg.freq_write = 12000000;     // Pixelclock ~12MHz (stabil bei vielen 7" TN)

        cfg.hsync_polarity    = 0;
        cfg.hsync_front_porch = 8;
        cfg.hsync_pulse_width = 2;
        cfg.hsync_back_porch  = 43;

        cfg.vsync_polarity    = 0;
        cfg.vsync_front_porch = 8;
        cfg.vsync_pulse_width = 2;
        cfg.vsync_back_porch  = 12;

        cfg.pclk_idle_high = 1;
      } else {
        // ---------- PRESET B (DEIN BOARD: Pins & Timings hier eintragen) ----------
        // TODO: Anpassen nach Pinout/Demo des Verkäufers
        cfg.pin_d0  = GPIO_NUM_8;
        cfg.pin_d1  = GPIO_NUM_9;
        cfg.pin_d2  = GPIO_NUM_10;
        cfg.pin_d3  = GPIO_NUM_11;
        cfg.pin_d4  = GPIO_NUM_12;
        cfg.pin_d5  = GPIO_NUM_13;
        cfg.pin_d6  = GPIO_NUM_14;
        cfg.pin_d7  = GPIO_NUM_21;
        cfg.pin_d8  = GPIO_NUM_47;
        cfg.pin_d9  = GPIO_NUM_48;
        cfg.pin_d10 = GPIO_NUM_45;
        cfg.pin_d11 = GPIO_NUM_0;
        cfg.pin_d12 = GPIO_NUM_35;
        cfg.pin_d13 = GPIO_NUM_36;
        cfg.pin_d14 = GPIO_NUM_37;
        cfg.pin_d15 = GPIO_NUM_38;

        cfg.pin_henable = GPIO_NUM_40;
        cfg.pin_vsync   = GPIO_NUM_41;
        cfg.pin_hsync   = GPIO_NUM_42;
        cfg.pin_pclk    = GPIO_NUM_39;

        cfg.freq_write = 16000000;

        cfg.hsync_polarity    = 0;
        cfg.hsync_front_porch = 40;
        cfg.hsync_pulse_width = 48;
        cfg.hsync_back_porch  = 40;

        cfg.vsync_polarity    = 0;
        cfg.vsync_front_porch = 13;
        cfg.vsync_pulse_width = 3;
        cfg.vsync_back_porch  = 29;

        cfg.pclk_idle_high = 1;
      }

      bus.config(cfg);
    }

    panel.setBus(&bus);

    // Backlight
    {
      auto cfg = light.config();
      cfg.pin_bl = (lgfx::pin_t)BL_PIN;
      light.config(cfg);
      panel.light(&light);
    }

    // Touch (GT911)
    {
      auto cfg = touch.config();
      cfg.x_min = 0;
      cfg.y_min = 0;
      cfg.x_max = SCREEN_W;
      cfg.y_max = SCREEN_H;

      cfg.bus_shared = false;
      cfg.offset_rotation = 0;

      cfg.i2c_port = I2C_NUM_0;
      cfg.pin_sda  = (lgfx::pin_t)TOUCH_I2C_SDA;
      cfg.pin_scl  = (lgfx::pin_t)TOUCH_I2C_SCL;
      cfg.pin_int  = (TOUCH_INT_PIN < 0) ? GPIO_NUM_NC : (lgfx::pin_t)TOUCH_INT_PIN;
      cfg.pin_rst  = (TOUCH_RST_PIN < 0) ? GPIO_NUM_NC : (lgfx::pin_t)TOUCH_RST_PIN;
      cfg.freq     = 100000;

      touch.config(cfg);
      panel.setTouch(&touch);
    }

    setPanel(&panel);
  }
};

static LGFX gfx;

// =============== LVGL Glue ===============
static lv_disp_draw_buf_t draw_buf;
static lv_color_t* lv_buf1 = nullptr;

static void lv_flush_cb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
  int32_t w = area->x2 - area->x1 + 1;
  int32_t h = area->y2 - area->y1 + 1;

  gfx.startWrite();
  gfx.setAddrWindow(area->x1, area->y1, w, h);
  gfx.writePixels((lgfx::rgb565_t*)color_p, (uint32_t)w * (uint32_t)h);
  gfx.endWrite();

  lv_disp_flush_ready(disp);
}

static void lv_touch_read_cb(lv_indev_drv_t* indev, lv_indev_data_t* data) {
  uint16_t x, y;
  bool touched = gfx.getTouch(&x, &y);
  if (!touched) {
    data->state = LV_INDEV_STATE_RELEASED;
  } else {
    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = x;
    data->point.y = y;
  }
}

// =============== App State Machine ===============
enum class State : uint8_t { IDLE, RUNNING, DONE, STOPPED, ERROR };
static State state = State::IDLE;

static Preferences prefs;

// Zähler (ISR)
volatile uint32_t g_count = 0;
volatile uint32_t g_last_ms = 0;

// Ziel
static int target = 100;

// UI heartbeat (Failsafe)
static uint32_t last_ui_heartbeat_ms = 0;

// =============== UI Objects ===============
static lv_obj_t* label_status;
static lv_obj_t* label_count;
static lv_obj_t* label_target;
static lv_obj_t* bar_progress;

static lv_obj_t* ta_target;
static lv_obj_t* kb;

static lv_obj_t* btn_start;
static lv_obj_t* btn_stop;
static lv_obj_t* btn_reset;
static lv_obj_t* btn_settings;

static lv_obj_t* panel_settings;
static lv_obj_t* slider_bright;
static lv_obj_t* slider_debounce;
static lv_obj_t* label_deb;
static lv_obj_t* label_bri;

// =============== Styles (Orange/Weiß) ===============
static lv_style_t st_btn_orange;
static lv_style_t st_btn_white;
static lv_style_t st_btn_red;
static lv_style_t st_card;
static lv_style_t st_title;
static lv_style_t st_label;

// Farben
static lv_color_t C_ORANGE;
static lv_color_t C_WHITE;
static lv_color_t C_BG;
static lv_color_t C_TEXT;
static lv_color_t C_RED;

// ===== Motor control (failsafe) =====
static void motor_set(bool on) {
  digitalWrite(GPIO_MOTOR, on ? HIGH : LOW);
}

// ===== Sensor ISR =====
static void IRAM_ATTR isr_sensor() {
  uint32_t now = millis();
  if (now - g_last_ms < debounce_ms) return;
  g_last_ms = now;

  if (state == State::RUNNING) {
    g_count++;
  }
}

// ===== Helpers =====
static const char* state_text(State s) {
  switch (s) {
    case State::IDLE:    return "Bereit";
    case State::RUNNING: return "Läuft…";
    case State::DONE:    return "Fertig";
    case State::STOPPED: return "Stopp";
    case State::ERROR:   return "Fehler";
    default:             return "—";
  }
}

static void save_settings() {
  prefs.putInt("target", target);
  prefs.putUInt("deb", debounce_ms);
  prefs.putUChar("bri", brightness);
}

static void load_settings() {
  target      = prefs.getInt("target", 100);
  debounce_ms = prefs.getUInt("deb", 3);
  brightness  = prefs.getUChar("bri", 220);
  if (target < TARGET_MIN) target = TARGET_MIN;
  if (target > TARGET_MAX) target = TARGET_MAX;
  if (debounce_ms < 1) debounce_ms = 1;
  if (debounce_ms > 50) debounce_ms = 50;
}

static void apply_brightness() {
  gfx.setBrightness(brightness);
}

static void ui_set_status(State s) {
  state = s;
  lv_label_set_text(label_status, state_text(state));

  // kleine Status-Animation (Fade)
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, label_status);
  lv_anim_set_values(&a, LV_OPA_30, LV_OPA_COVER);
  lv_anim_set_time(&a, 180);
  lv_anim_set_exec_cb(&a, [](void* obj, int32_t v) {
    lv_obj_set_style_opa((lv_obj_t*)obj, (lv_opa_t)v, 0);
  });
  lv_anim_start(&a);
}

static void ui_update_numbers() {
  char buf1[64], buf2[64];
  snprintf(buf1, sizeof(buf1), "IST: %lu", (unsigned long)g_count);
  snprintf(buf2, sizeof(buf2), "ZIEL: %d", target);

  lv_label_set_text(label_count, buf1);
  lv_label_set_text(label_target, buf2);

  int pct = 0;
  if (target > 0) pct = (int)((g_count * 100UL) / (uint32_t)target);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  lv_bar_set_value(bar_progress, pct, LV_ANIM_ON);
}

static void stop_run(State next) {
  motor_set(false);
  ui_set_status(next);
}

static void start_run() {
  // Ziel aus Textfeld
  int v = atoi(lv_textarea_get_text(ta_target));
  if (v < TARGET_MIN) v = TARGET_MIN;
  if (v > TARGET_MAX) v = TARGET_MAX;
  target = v;

  g_count = 0;
  motor_set(true);
  ui_set_status(State::RUNNING);
  save_settings();
}

// ===== Button press animation helper =====
static void animate_button_press(lv_obj_t* btn) {
  // scale down/up effect via transform zoom
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, btn);
  lv_anim_set_time(&a, 80);
  lv_anim_set_values(&a, 256, 235);
  lv_anim_set_playback_time(&a, 100);
  lv_anim_set_exec_cb(&a, [](void* obj, int32_t v) {
    lv_obj_set_style_transform_zoom((lv_obj_t*)obj, v, 0);
  });
  lv_anim_start(&a);
}

// ===== UI callbacks =====
static void btn_start_cb(lv_event_t* e) {
  animate_button_press(btn_start);
  if (state == State::RUNNING) return;
  start_run();
}

static void btn_stop_cb(lv_event_t* e) {
  animate_button_press(btn_stop);
  if (state == State::RUNNING) stop_run(State::STOPPED);
}

static void btn_reset_cb(lv_event_t* e) {
  animate_button_press(btn_reset);
  motor_set(false);
  g_count = 0;
  ui_set_status(State::IDLE);
  ui_update_numbers();
}

static void kb_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
    lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
  }
}

static void ta_event_cb(lv_event_t* e) {
  if (lv_event_get_code(e) == LV_EVENT_FOCUSED) {
    lv_keyboard_set_textarea(kb, ta_target);
    lv_obj_clear_flag(kb, LV_OBJ_FLAG_HIDDEN);
  }
}

static void toggle_settings(bool show) {
  if (show) {
    lv_obj_clear_flag(panel_settings, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(panel_settings, LV_OBJ_FLAG_HIDDEN);
  }
}

static void btn_settings_cb(lv_event_t* e) {
  animate_button_press(btn_settings);
  bool hidden = lv_obj_has_flag(panel_settings, LV_OBJ_FLAG_HIDDEN);
  toggle_settings(hidden);
}

// Slider events
static void slider_bri_cb(lv_event_t* e) {
  brightness = (uint8_t)lv_slider_get_value(slider_bright);
  char b[48];
  snprintf(b, sizeof(b), "Helligkeit: %u", brightness);
  lv_label_set_text(label_bri, b);
  apply_brightness();
  save_settings();
}

static void slider_deb_cb(lv_event_t* e) {
  debounce_ms = (uint32_t)lv_slider_get_value(slider_debounce);
  char b[48];
  snprintf(b, sizeof(b), "Entprellung: %lu ms", (unsigned long)debounce_ms);
  lv_label_set_text(label_deb, b);
  save_settings();
}

// =============== UI Build ===============
static void init_styles() {
  C_ORANGE = lv_color_hex(0xFF7A00);
  C_WHITE  = lv_color_hex(0xFFFFFF);
  C_BG     = lv_color_hex(0xF3F4F6);
  C_TEXT   = lv_color_hex(0x111827);
  C_RED    = lv_color_hex(0xE11D48);

  lv_style_init(&st_title);
  lv_style_set_text_font(&st_title, &lv_font_montserrat_28);
  lv_style_set_text_color(&st_title, C_TEXT);

  lv_style_init(&st_label);
  lv_style_set_text_font(&st_label, &lv_font_montserrat_22);
  lv_style_set_text_color(&st_label, C_TEXT);

  lv_style_init(&st_card);
  lv_style_set_radius(&st_card, 18);
  lv_style_set_bg_color(&st_card, C_WHITE);
  lv_style_set_bg_opa(&st_card, LV_OPA_COVER);
  lv_style_set_pad_all(&st_card, 16);
  lv_style_set_shadow_width(&st_card, 24);
  lv_style_set_shadow_opa(&st_card, LV_OPA_20);
  lv_style_set_shadow_ofs_y(&st_card, 8);

  lv_style_init(&st_btn_orange);
  lv_style_set_radius(&st_btn_orange, 18);
  lv_style_set_bg_color(&st_btn_orange, C_ORANGE);
  lv_style_set_bg_opa(&st_btn_orange, LV_OPA_COVER);
  lv_style_set_shadow_width(&st_btn_orange, 20);
  lv_style_set_shadow_opa(&st_btn_orange, LV_OPA_25);
  lv_style_set_shadow_ofs_y(&st_btn_orange, 6);
  lv_style_set_text_color(&st_btn_orange, C_WHITE);

  lv_style_init(&st_btn_white);
  lv_style_set_radius(&st_btn_white, 18);
  lv_style_set_bg_color(&st_btn_white, C_WHITE);
  lv_style_set_bg_opa(&st_btn_white, LV_OPA_COVER);
  lv_style_set_border_color(&st_btn_white, lv_color_hex(0xD1D5DB));
  lv_style_set_border_width(&st_btn_white, 2);
  lv_style_set_shadow_width(&st_btn_white, 14);
  lv_style_set_shadow_opa(&st_btn_white, LV_OPA_15);
  lv_style_set_shadow_ofs_y(&st_btn_white, 4);
  lv_style_set_text_color(&st_btn_white, C_TEXT);

  lv_style_init(&st_btn_red);
  lv_style_set_radius(&st_btn_red, 18);
  lv_style_set_bg_color(&st_btn_red, C_RED);
  lv_style_set_bg_opa(&st_btn_red, LV_OPA_COVER);
  lv_style_set_shadow_width(&st_btn_red, 20);
  lv_style_set_shadow_opa(&st_btn_red, LV_OPA_25);
  lv_style_set_shadow_ofs_y(&st_btn_red, 6);
  lv_style_set_text_color(&st_btn_red, C_WHITE);
}

static lv_obj_t* make_button(lv_obj_t* parent, const char* txt, lv_style_t* st, int w, int h) {
  lv_obj_t* b = lv_btn_create(parent);
  lv_obj_set_size(b, w, h);
  lv_obj_add_style(b, st, 0);

  lv_obj_t* l = lv_label_create(b);
  lv_label_set_text(l, txt);
  lv_obj_set_style_text_font(l, &lv_font_montserrat_24, 0);
  lv_obj_center(l);

  return b;
}

static void create_ui() {
  lv_obj_t* scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, C_BG, 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  // Header
  lv_obj_t* title = lv_label_create(scr);
  lv_obj_add_style(title, &st_title, 0);
  lv_label_set_text(title, "Bandware Zähler");
  lv_obj_align(title, LV_ALIGN_TOP_LEFT, 24, 16);

  btn_settings = make_button(scr, "Einstellungen", &st_btn_white, 210, 56);
  lv_obj_align(btn_settings, LV_ALIGN_TOP_RIGHT, -24, 12);
  lv_obj_add_event_cb(btn_settings, btn_settings_cb, LV_EVENT_CLICKED, nullptr);

  // Card main
  lv_obj_t* card = lv_obj_create(scr);
  lv_obj_add_style(card, &st_card, 0);
  lv_obj_set_size(card, 760, 250);
  lv_obj_align(card, LV_ALIGN_TOP_MID, 0, 80);

  lv_obj_t* lbl = lv_label_create(card);
  lv_obj_add_style(lbl, &st_label, 0);
  lv_label_set_text(lbl, "Zielmenge (1..1000):");
  lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 0, 0);

  ta_target = lv_textarea_create(card);
  lv_textarea_set_one_line(ta_target, true);
  lv_textarea_set_max_length(ta_target, 4);
  lv_textarea_set_accepted_chars(ta_target, "0123456789");
  char tbuf[8]; snprintf(tbuf, sizeof(tbuf), "%d", target);
  lv_textarea_set_text(ta_target, tbuf);
  lv_obj_set_size(ta_target, 220, 56);
  lv_obj_align(ta_target, LV_ALIGN_TOP_LEFT, 0, 44);
  lv_obj_add_event_cb(ta_target, ta_event_cb, LV_EVENT_ALL, nullptr);

  // Buttons row
  btn_start = make_button(card, "START", &st_btn_orange, 200, 70);
  lv_obj_align(btn_start, LV_ALIGN_TOP_LEFT, 0, 120);
  lv_obj_add_event_cb(btn_start, btn_start_cb, LV_EVENT_CLICKED, nullptr);

  btn_stop = make_button(card, "STOP", &st_btn_red, 200, 70);
  lv_obj_align(btn_stop, LV_ALIGN_TOP_LEFT, 220, 120);
  lv_obj_add_event_cb(btn_stop, btn_stop_cb, LV_EVENT_CLICKED, nullptr);

  btn_reset = make_button(card, "RESET", &st_btn_white, 200, 70);
  lv_obj_align(btn_reset, LV_ALIGN_TOP_LEFT, 440, 120);
  lv_obj_add_event_cb(btn_reset, btn_reset_cb, LV_EVENT_CLICKED, nullptr);

  // Status / Counters
  label_status = lv_label_create(scr);
  lv_obj_set_style_text_font(label_status, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(label_status, C_ORANGE, 0);
  lv_obj_align(label_status, LV_ALIGN_TOP_LEFT, 24, 350);
  lv_label_set_text(label_status, "Bereit");

  label_count = lv_label_create(scr);
  lv_obj_set_style_text_font(label_count, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(label_count, C_TEXT, 0);
  lv_obj_align(label_count, LV_ALIGN_TOP_LEFT, 24, 392);

  label_target = lv_label_create(scr);
  lv_obj_set_style_text_font(label_target, &lv_font_montserrat_22, 0);
  lv_obj_set_style_text_color(label_target, lv_color_hex(0x374151), 0);
  lv_obj_align(label_target, LV_ALIGN_TOP_LEFT, 24, 432);

  // Progress bar
  bar_progress = lv_bar_create(scr);
  lv_obj_set_size(bar_progress, 420, 26);
  lv_obj_align(bar_progress, LV_ALIGN_TOP_RIGHT, -24, 400);
  lv_bar_set_range(bar_progress, 0, 100);
  lv_bar_set_value(bar_progress, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(bar_progress, lv_color_hex(0xE5E7EB), 0);
  lv_obj_set_style_bg_opa(bar_progress, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(bar_progress, 14, 0);
  lv_obj_set_style_bg_color(bar_progress, C_ORANGE, LV_PART_INDICATOR);
  lv_obj_set_style_radius(bar_progress, 14, LV_PART_INDICATOR);

  // On-screen keypad (LVGL keyboard)
  kb = lv_keyboard_create(scr);
  lv_obj_set_size(kb, 760, 200);
  lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_NUMBER);
  lv_obj_add_event_cb(kb, kb_event_cb, LV_EVENT_ALL, nullptr);
  lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);

  // Settings panel
  panel_settings = lv_obj_create(scr);
  lv_obj_add_style(panel_settings, &st_card, 0);
  lv_obj_set_size(panel_settings, 760, 180);
  lv_obj_align(panel_settings, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_obj_add_flag(panel_settings, LV_OBJ_FLAG_HIDDEN);

  label_bri = lv_label_create(panel_settings);
  lv_obj_set_style_text_font(label_bri, &lv_font_montserrat_20, 0);
  lv_obj_align(label_bri, LV_ALIGN_TOP_LEFT, 0, 0);

  slider_bright = lv_slider_create(panel_settings);
  lv_obj_set_size(slider_bright, 500, 18);
  lv_obj_align(slider_bright, LV_ALIGN_TOP_LEFT, 0, 36);
  lv_slider_set_range(slider_bright, 10, 255);
  lv_slider_set_value(slider_bright, brightness, LV_ANIM_OFF);
  lv_obj_add_event_cb(slider_bright, slider_bri_cb, LV_EVENT_VALUE_CHANGED, nullptr);

  label_deb = lv_label_create(panel_settings);
  lv_obj_set_style_text_font(label_deb, &lv_font_montserrat_20, 0);
  lv_obj_align(label_deb, LV_ALIGN_TOP_LEFT, 0, 70);

  slider_debounce = lv_slider_create(panel_settings);
  lv_obj_set_size(slider_debounce, 500, 18);
  lv_obj_align(slider_debounce, LV_ALIGN_TOP_LEFT, 0, 106);
  lv_slider_set_range(slider_debounce, 1, 50);
  lv_slider_set_value(slider_debounce, debounce_ms, LV_ANIM_OFF);
  lv_obj_add_event_cb(slider_debounce, slider_deb_cb, LV_EVENT_VALUE_CHANGED, nullptr);

  // init labels
  char b1[48], b2[48];
  snprintf(b1, sizeof(b1), "Helligkeit: %u", brightness);
  snprintf(b2, sizeof(b2), "Entprellung: %lu ms", (unsigned long)debounce_ms);
  lv_label_set_text(label_bri, b1);
  lv_label_set_text(label_deb, b2);

  ui_update_numbers();
}

// =============== Setup/Loop ===============
static void init_lvgl() {
  lv_init();

  // Draw buffer (PSRAM bevorzugt)
  lv_buf1 = (lv_color_t*)heap_caps_malloc(SCREEN_W * LVGL_BUF_LINES * sizeof(lv_color_t),
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!lv_buf1) {
    lv_buf1 = (lv_color_t*)heap_caps_malloc(SCREEN_W * LVGL_BUF_LINES * sizeof(lv_color_t),
                                           MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }

  lv_disp_draw_buf_init(&draw_buf, lv_buf1, nullptr, SCREEN_W * LVGL_BUF_LINES);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCREEN_W;
  disp_drv.ver_res = SCREEN_H;
  disp_drv.flush_cb = lv_flush_cb;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = lv_touch_read_cb;
  lv_indev_drv_register(&indev_drv);
}

static void failsafe_check() {
  // Wenn UI nicht mehr "lebt" -> Motor aus
  uint32_t now = millis();
  if (state == State::RUNNING) {
    if (now - last_ui_heartbeat_ms > UI_HEARTBEAT_TIMEOUT_MS) {
      motor_set(false);
      state = State::ERROR;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(50);

  // IO
  pinMode(GPIO_MOTOR, OUTPUT);
  motor_set(false);

  // Sensor: PC817 open-collector -> Pullup
  pinMode(GPIO_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GPIO_SENSOR), isr_sensor, FALLING);

  // Persistenz
  prefs.begin("bandware", false);
  load_settings();

  // Display init
  gfx.init();
  gfx.setRotation(0);
  apply_brightness();

  // LVGL init
  init_lvgl();
  init_styles();
  create_ui();
  ui_set_status(State::IDLE);

  // Watchdog
  if (ENABLE_WDT) {
    // 3 Sekunden WDT
    esp_task_wdt_init(3, true);
    esp_task_wdt_add(NULL);
  }

  last_ui_heartbeat_ms = millis();
  Serial.println("[OK] Bandware Zaehler gestartet.");
}

void loop() {
  // LVGL tick
  lv_timer_handler();
  delay(5);

  // UI heartbeat (für Failsafe)
  last_ui_heartbeat_ms = millis();

  // Watchdog feed
  if (ENABLE_WDT) {
    esp_task_wdt_reset();
  }

  // Lauf-Logik
  if (state == State::RUNNING) {
    if ((int)g_count >= target) {
      stop_run(State::DONE);
    }
  }

  // UI Refresh (nicht zu oft)
  static uint32_t lastUi = 0;
  uint32_t now = millis();
  if (now - lastUi > 100) {
    lastUi = now;
    ui_update_numbers();
  }

  // Failsafe check
  failsafe_check();
}
