#include <Arduino.h>
#include <Preferences.h>
#include <lvgl.h>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>

#include "esp_task_wdt.h"

// ========================= CONFIG =========================
static const int GPIO_MOTOR  = 17;   // HIGH = Motor ON
static const int GPIO_SENSOR = 18;   // PC817 OUT -> GPIO (Interrupt FALLING)

static const int TARGET_MIN = 1;
static const int TARGET_MAX = 1000;

static const int SCREEN_W = 800;
static const int SCREEN_H = 480;
static const int LVGL_BUF_LINES = 40;

static const int DISPLAY_PRESET = 0; // اگر تصویر نیامد، Pinmap/Timing باید مطابق برد شود

// Touch GT911 (ممکن است روی برد شما فرق کند)
static const int TOUCH_I2C_SDA = 19;
static const int TOUCH_I2C_SCL = 20;
static const int TOUCH_RST_PIN = 38;
static const int TOUCH_INT_PIN = -1;

// Backlight pin
static const int BL_PIN = 2;

// Settings (NVS)
static uint32_t debounce_ms = 3;  // 1..50
static uint8_t brightness  = 220; // 10..255

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

    {
      auto cfg = bus.config();
      cfg.panel = &panel;

      if (DISPLAY_PRESET == 0) {
        // PRESET A (رایج)
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
        cfg.pin_vsync   = GPIO_NUM_40;
        cfg.pin_hsync   = GPIO_NUM_39;
        cfg.pin_pclk    = GPIO_NUM_42;

        cfg.freq_write = 12000000;

        cfg.hsync_polarity    = 0;
        cfg.hsync_front_porch = 8;
        cfg.hsync_pulse_width = 2;
        cfg.hsync_back_porch  = 43;

        cfg.vsync_polarity    = 0;
        cfg.vsync_front_porch = 8;
        cfg.vsync_pulse_width = 2;
        cfg.vsync_back_porch  = 12;

        cfg.pclk_idle_high = 1;
      }

      bus.config(cfg);
    }

    panel.setBus(&bus);

    // Backlight: از int استفاده می‌کنیم (نه lgfx::pin_t)
    {
      auto cfg = light.config();
      cfg.pin_bl = BL_PIN;
      light.config(cfg);
      panel.light(&light);
    }

    // Touch GT911
    {
      auto cfg = touch.config();
      cfg.x_min = 0; cfg.y_min = 0;
      cfg.x_max = SCREEN_W; cfg.y_max = SCREEN_H;

      cfg.bus_shared = false;
      cfg.offset_rotation = 0;

      cfg.i2c_port = I2C_NUM_0;
      cfg.pin_sda  = TOUCH_I2C_SDA;
      cfg.pin_scl  = TOUCH_I2C_SCL;
      cfg.pin_int  = (TOUCH_INT_PIN < 0) ? GPIO_NUM_NC : TOUCH_INT_PIN;
      cfg.pin_rst  = (TOUCH_RST_PIN < 0) ? GPIO_NUM_NC : TOUCH_RST_PIN;
      cfg.freq = 100000;

      touch.config(cfg);
      panel.setTouch(&touch);
    }

    setPanel(&panel);
  }
};

static LGFX gfx;

// =============== LVGL ===============
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
  (void) indev;
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

// =============== State Machine ===============
enum class State : uint8_t { IDLE, RUNNING, DONE, STOPPED, ERROR };
static State state = State::IDLE;

static Preferences prefs;

volatile uint32_t g_count = 0;
volatile uint32_t g_last_ms = 0;
static int target = 100;

// UI
static lv_obj_t* label_status;
static lv_obj_t* label_count;
static lv_obj_t* label_target;
static lv_obj_t* bar_progress;

static lv_obj_t* ta_target;
static lv_obj_t* kb;

static lv_style_t st_btn_orange, st_btn_white, st_btn_red, st_card, st_title;

static void motor_set(bool on) { digitalWrite(GPIO_MOTOR, on ? HIGH : LOW); }

static void IRAM_ATTR isr_sensor() {
  uint32_t now = millis();
  if (now - g_last_ms < debounce_ms) return;
  g_last_ms = now;
  if (state == State::RUNNING) g_count++;
}

static const char* state_text(State s) {
  switch (s) {
    case State::IDLE:    return "Bereit";
    case State::RUNNING: return "Läuft…";
    case State::DONE:    return "Fertig: Bitte Band entnehmen";
    case State::STOPPED: return "Stopp";
    case State::ERROR:   return "Fehler";
  }
  return "—";
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
  if (brightness < 10) brightness = 10;
}

static void apply_brightness() { gfx.setBrightness(brightness); }

static void ui_set_status(State s) {
  state = s;
  lv_label_set_text(label_status, state_text(state));
}

static void ui_update_numbers() {
  char buf1[64], buf2[64];
  snprintf(buf1, sizeof(buf1), "IST: %lu", (unsigned long)g_count);
  snprintf(buf2, sizeof(buf2), "ZIEL: %d", target);
  lv_label_set_text(label_count, buf1);
  lv_label_set_text(label_target, buf2);

  int pct = 0;
  if (target > 0) pct = (int)((g_count * 100UL) / (uint32_t)target);
  if (pct > 100) pct = 100;
  lv_bar_set_value(bar_progress, pct, LV_ANIM_ON);
}

static void start_run() {
  int v = atoi(lv_textarea_get_text(ta_target));
  if (v < TARGET_MIN) v = TARGET_MIN;
  if (v > TARGET_MAX) v = TARGET_MAX;
  target = v;

  g_count = 0;
  motor_set(true);
  ui_set_status(State::RUNNING);
  save_settings();
}

static void stop_run(State next) {
  motor_set(false);
  ui_set_status(next);
}

static void animate_button_press(lv_obj_t* btn) {
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

static void btn_start_cb(lv_event_t* e) {
  lv_obj_t* btn = lv_event_get_target(e);
  animate_button_press(btn);
  if (state != State::RUNNING) start_run();
}
static void btn_stop_cb(lv_event_t* e) {
  lv_obj_t* btn = lv_event_get_target(e);
  animate_button_press(btn);
  if (state == State::RUNNING) stop_run(State::STOPPED);
}
static void btn_reset_cb(lv_event_t* e) {
  lv_obj_t* btn = lv_event_get_target(e);
  animate_button_press(btn);
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

static void init_styles() {
  lv_color_t ORANGE = lv_color_hex(0xFF7A00);
  lv_color_t WHITE  = lv_color_hex(0xFFFFFF);
  lv_color_t BG     = lv_color_hex(0xF3F4F6);
  lv_color_t TEXT   = lv_color_hex(0x111827);
  lv_color_t RED    = lv_color_hex(0xE11D48);

  lv_obj_set_style_bg_color(lv_scr_act(), BG, 0);
  lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);

  lv_style_init(&st_title);
  lv_style_set_text_font(&st_title, &lv_font_montserrat_28);
  lv_style_set_text_color(&st_title, TEXT);

  lv_style_init(&st_card);
  lv_style_set_radius(&st_card, 18);
  lv_style_set_bg_color(&st_card, WHITE);
  lv_style_set_bg_opa(&st_card, LV_OPA_COVER);
  lv_style_set_pad_all(&st_card, 16);
  lv_style_set_shadow_width(&st_card, 24);
  lv_style_set_shadow_opa(&st_card, LV_OPA_20);
  lv_style_set_shadow_ofs_y(&st_card, 8);

  lv_style_init(&st_btn_orange);
  lv_style_set_radius(&st_btn_orange, 18);
  lv_style_set_bg_color(&st_btn_orange, ORANGE);
  lv_style_set_bg_opa(&st_btn_orange, LV_OPA_COVER);
  lv_style_set_shadow_width(&st_btn_orange, 20);
  lv_style_set_shadow_opa(&st_btn_orange, LV_OPA_20); // جایگزین LV_OPA_25
  lv_style_set_shadow_ofs_y(&st_btn_orange, 6);
  lv_style_set_text_color(&st_btn_orange, WHITE);

  lv_style_init(&st_btn_white);
  lv_style_set_radius(&st_btn_white, 18);
  lv_style_set_bg_color(&st_btn_white, WHITE);
  lv_style_set_bg_opa(&st_btn_white, LV_OPA_COVER);
  lv_style_set_border_color(&st_btn_white, lv_color_hex(0xD1D5DB));
  lv_style_set_border_width(&st_btn_white, 2);
  lv_style_set_shadow_width(&st_btn_white, 14);
  lv_style_set_shadow_opa(&st_btn_white, LV_OPA_10); // جایگزین LV_OPA_15
  lv_style_set_shadow_ofs_y(&st_btn_white, 4);
  lv_style_set_text_color(&st_btn_white, TEXT);

  lv_style_init(&st_btn_red);
  lv_style_set_radius(&st_btn_red, 18);
  lv_style_set_bg_color(&st_btn_red, RED);
  lv_style_set_bg_opa(&st_btn_red, LV_OPA_COVER);
  lv_style_set_shadow_width(&st_btn_red, 20);
  lv_style_set_shadow_opa(&st_btn_red, LV_OPA_20);
  lv_style_set_shadow_ofs_y(&st_btn_red, 6);
  lv_style_set_text_color(&st_btn_red, WHITE);
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

  lv_obj_t* title = lv_label_create(scr);
  lv_obj_add_style(title, &st_title, 0);
  lv_label_set_text(title, "Bandware Zähler");
  lv_obj_align(title, LV_ALIGN_TOP_LEFT, 24, 16);

  lv_obj_t* card = lv_obj_create(scr);
  lv_obj_add_style(card, &st_card, 0);
  lv_obj_set_size(card, 760, 250);
  lv_obj_align(card, LV_ALIGN_TOP_MID, 0, 80);

  lv_obj_t* lbl = lv_label_create(card);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_22, 0);
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

  lv_obj_t* btn_start = make_button(card, "START", &st_btn_orange, 200, 70);
  lv_obj_align(btn_start, LV_ALIGN_TOP_LEFT, 0, 120);
  lv_obj_add_event_cb(btn_start, btn_start_cb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* btn_stop = make_button(card, "STOP", &st_btn_red, 200, 70);
  lv_obj_align(btn_stop, LV_ALIGN_TOP_LEFT, 220, 120);
  lv_obj_add_event_cb(btn_stop, btn_stop_cb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* btn_reset = make_button(card, "RESET", &st_btn_white, 200, 70);
  lv_obj_align(btn_reset, LV_ALIGN_TOP_LEFT, 440, 120);
  lv_obj_add_event_cb(btn_reset, btn_reset_cb, LV_EVENT_CLICKED, nullptr);

  label_status = lv_label_create(scr);
  lv_obj_set_style_text_font(label_status, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(label_status, lv_color_hex(0xFF7A00), 0);
  lv_obj_align(label_status, LV_ALIGN_TOP_LEFT, 24, 350);

  label_count = lv_label_create(scr);
  lv_obj_set_style_text_font(label_count, &lv_font_montserrat_28, 0);
  lv_obj_align(label_count, LV_ALIGN_TOP_LEFT, 24, 392);

  label_target = lv_label_create(scr);
  lv_obj_set_style_text_font(label_target, &lv_font_montserrat_22, 0);
  lv_obj_set_style_text_color(label_target, lv_color_hex(0x374151), 0);
  lv_obj_align(label_target, LV_ALIGN_TOP_LEFT, 24, 432);

  bar_progress = lv_bar_create(scr);
  lv_obj_set_size(bar_progress, 420, 26);
  lv_obj_align(bar_progress, LV_ALIGN_TOP_RIGHT, -24, 400);
  lv_bar_set_range(bar_progress, 0, 100);
  lv_bar_set_value(bar_progress, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(bar_progress, lv_color_hex(0xE5E7EB), 0);
  lv_obj_set_style_bg_opa(bar_progress, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(bar_progress, 14, 0);
  lv_obj_set_style_bg_color(bar_progress, lv_color_hex(0xFF7A00), LV_PART_INDICATOR);
  lv_obj_set_style_radius(bar_progress, 14, LV_PART_INDICATOR);

  kb = lv_keyboard_create(scr);
  lv_obj_set_size(kb, 760, 200);
  lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_NUMBER);
  lv_obj_add_event_cb(kb, kb_event_cb, LV_EVENT_ALL, nullptr);
  lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);

  ui_set_status(State::IDLE);
  ui_update_numbers();
}

static void init_lvgl() {
  lv_init();

  lv_buf1 = (lv_color_t*)heap_caps_malloc(SCREEN_W * LVGL_BUF_LINES * sizeof(lv_color_t),
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!lv_buf1) lv_buf1 = (lv_color_t*)malloc(SCREEN_W * LVGL_BUF_LINES * sizeof(lv_color_t));

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

static void wdt_setup() {
  if (!ENABLE_WDT) return;
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms = 3000;
  cfg.trigger_panic = true;
  cfg.idle_core_mask = (1 << 0) | (1 << 1);
  esp_task_wdt_init(&cfg);
  esp_task_wdt_add(NULL);
}

void setup() {
  Serial.begin(115200);

  pinMode(GPIO_MOTOR, OUTPUT);
  motor_set(false);

  pinMode(GPIO_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GPIO_SENSOR), isr_sensor, FALLING);

  prefs.begin("bandware", false);
  load_settings();

  gfx.init();
  gfx.setRotation(0);
  apply_brightness();

  init_lvgl();
  init_styles();
  create_ui();

  wdt_setup();
}

void loop() {
  lv_timer_handler();
  delay(5);

  if (ENABLE_WDT) esp_task_wdt_reset();

  if (state == State::RUNNING) {
    if ((int)g_count >= target) {
      stop_run(State::DONE);
    }
  }

  static uint32_t lastUi = 0;
  uint32_t now = millis();
  if (now - lastUi > 100) {
    lastUi = now;
    ui_update_numbers();
  }
}
