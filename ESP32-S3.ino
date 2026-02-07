#define LGFX_USE_V1
#include <Arduino.h>
#include <lvgl.h>

#include <Wire.h>
#include <Preferences.h>

#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>

/* =========================
   CONFIG (Pins & Behavior)
   ========================= */
static const int SCREEN_W = 800;
static const int SCREEN_H = 480;

/* Your RGB mapping (the one that worked) */
static const gpio_num_t PIN_D0  = GPIO_NUM_15;
static const gpio_num_t PIN_D1  = GPIO_NUM_7;
static const gpio_num_t PIN_D2  = GPIO_NUM_6;
static const gpio_num_t PIN_D3  = GPIO_NUM_5;
static const gpio_num_t PIN_D4  = GPIO_NUM_4;
static const gpio_num_t PIN_D5  = GPIO_NUM_9;
static const gpio_num_t PIN_D6  = GPIO_NUM_46;
static const gpio_num_t PIN_D7  = GPIO_NUM_3;
static const gpio_num_t PIN_D8  = GPIO_NUM_8;
static const gpio_num_t PIN_D9  = GPIO_NUM_16;
static const gpio_num_t PIN_D10 = GPIO_NUM_1;
static const gpio_num_t PIN_D11 = GPIO_NUM_14;
static const gpio_num_t PIN_D12 = GPIO_NUM_21;
static const gpio_num_t PIN_D13 = GPIO_NUM_47;
static const gpio_num_t PIN_D14 = GPIO_NUM_48;
static const gpio_num_t PIN_D15 = GPIO_NUM_45;

static const gpio_num_t PIN_DE    = GPIO_NUM_41; // "henable" in Bus_RGB
static const gpio_num_t PIN_VSYNC = GPIO_NUM_40;
static const gpio_num_t PIN_HSYNC = GPIO_NUM_39;
static const gpio_num_t PIN_PCLK  = GPIO_NUM_42;

/* Panel clock & timings (same as your working test) */
static const uint32_t RGB_FREQ = 12000000;

/* I/O for machine */
static const int GPIO_SENSOR = 10;   // <-- change if needed (input from PC817 OUT)
static const int GPIO_MOTOR  = 11;   // <-- change if needed (output to MOSFET/SSR)

static const bool MOTOR_ON_LEVEL = HIGH;

/* Touch (GT911 typical). If you don't know pins, leave as-is and only set SDA/SCL if needed. */
static const int TOUCH_I2C_SDA = 18;    // <-- change if your board uses other pins
static const int TOUCH_I2C_SCL = 17;    // <-- change if your board uses other pins
static const int TOUCH_RST_PIN = -1;    // optional
static const int TOUCH_INT_PIN = -1;    // optional
static const uint8_t GT911_ADDR_1 = 0x5D;
static const uint8_t GT911_ADDR_2 = 0x14;

/* Debounce */
static const uint32_t DEFAULT_DEBOUNCE_MS = 5;
static const uint16_t TARGET_MIN = 1;
static const uint16_t TARGET_MAX = 1000;

/* LVGL draw buffer lines */
static const uint16_t LV_BUF_LINES = 20;

/* =========================
   Display driver (LovyanGFX RGB)
   ========================= */
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[SCREEN_W * LV_BUF_LINES];

class LGFX : public lgfx::LGFX_Device {
  lgfx::Bus_RGB bus;
  lgfx::Panel_RGB panel;

public:
  LGFX() {
    {
      auto cfg = bus.config();

      cfg.pin_d0  = PIN_D0;
      cfg.pin_d1  = PIN_D1;
      cfg.pin_d2  = PIN_D2;
      cfg.pin_d3  = PIN_D3;
      cfg.pin_d4  = PIN_D4;
      cfg.pin_d5  = PIN_D5;
      cfg.pin_d6  = PIN_D6;
      cfg.pin_d7  = PIN_D7;
      cfg.pin_d8  = PIN_D8;
      cfg.pin_d9  = PIN_D9;
      cfg.pin_d10 = PIN_D10;
      cfg.pin_d11 = PIN_D11;
      cfg.pin_d12 = PIN_D12;
      cfg.pin_d13 = PIN_D13;
      cfg.pin_d14 = PIN_D14;
      cfg.pin_d15 = PIN_D15;

      cfg.pin_henable = PIN_DE;    // DE
      cfg.pin_vsync   = PIN_VSYNC;
      cfg.pin_hsync   = PIN_HSYNC;
      cfg.pin_pclk    = PIN_PCLK;

      cfg.freq_write = RGB_FREQ;

      cfg.hsync_polarity    = 0;
      cfg.hsync_front_porch = 8;
      cfg.hsync_pulse_width = 2;
      cfg.hsync_back_porch  = 43;

      cfg.vsync_polarity    = 0;
      cfg.vsync_front_porch = 8;
      cfg.vsync_pulse_width = 2;
      cfg.vsync_back_porch  = 12;

      cfg.pclk_idle_high = 1;

      bus.config(cfg);
      panel.setBus(&bus);
    }

    {
      auto cfg = panel.config();
      cfg.panel_width   = SCREEN_W;
      cfg.panel_height  = SCREEN_H;
      cfg.memory_width  = SCREEN_W;
      cfg.memory_height = SCREEN_H;
      cfg.offset_x      = 0;
      cfg.offset_y      = 0;
      panel.config(cfg);
    }

    setPanel(&panel);
  }
};

static LGFX lcd;

/* LVGL flush */
static void flush_cb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
  uint32_t w = (uint32_t)(area->x2 - area->x1 + 1);
  uint32_t h = (uint32_t)(area->y2 - area->y1 + 1);

  lcd.startWrite();
  lcd.setAddrWindow(area->x1, area->y1, w, h);
  lcd.writePixels((lgfx::rgb565_t*)color_p, w * h);
  lcd.endWrite();

  lv_disp_flush_ready(disp);
}

/* =========================
   Touch GT911 (minimal)
   ========================= */
static uint8_t g_gt911_addr = 0;

static bool i2c_read(uint8_t addr, uint16_t reg, uint8_t* data, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)(reg & 0xFF));
  Wire.write((uint8_t)(reg >> 8));
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((int)addr, (int)len) != (int)len) return false;
  for (size_t i = 0; i < len; i++) data[i] = Wire.read();
  return true;
}

static bool i2c_write(uint8_t addr, uint16_t reg, const uint8_t* data, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)(reg & 0xFF));
  Wire.write((uint8_t)(reg >> 8));
  for (size_t i = 0; i < len; i++) Wire.write(data[i]);
  return (Wire.endTransmission() == 0);
}

static bool gt911_detect() {
  uint8_t tmp;
  if (i2c_read(GT911_ADDR_1, 0x8140, &tmp, 1)) { g_gt911_addr = GT911_ADDR_1; return true; }
  if (i2c_read(GT911_ADDR_2, 0x8140, &tmp, 1)) { g_gt911_addr = GT911_ADDR_2; return true; }
  return false;
}

// Returns pressed + x/y
static bool gt911_read_point(uint16_t &x, uint16_t &y) {
  if (!g_gt911_addr) return false;

  uint8_t status = 0;
  if (!i2c_read(g_gt911_addr, 0x814E, &status, 1)) return false;

  uint8_t points = status & 0x0F;
  if (points == 0) {
    // Clear status
    uint8_t z = 0;
    i2c_write(g_gt911_addr, 0x814E, &z, 1);
    return false;
  }

  uint8_t buf[8] = {0};
  if (!i2c_read(g_gt911_addr, 0x8150, buf, 8)) return false;

  // GT911: X = buf[1]<<8 | buf[0], Y = buf[3]<<8 | buf[2] (common)
  x = (uint16_t)(buf[1] << 8 | buf[0]);
  y = (uint16_t)(buf[3] << 8 | buf[2]);

  uint8_t z = 0;
  i2c_write(g_gt911_addr, 0x814E, &z, 1);
  return true;
}

static void lv_touch_cb(lv_indev_drv_t* indev, lv_indev_data_t* data) {
  (void)indev;
  uint16_t x, y;
  bool pressed = gt911_read_point(x, y);
  if (pressed) {
    // clamp
    if (x >= SCREEN_W) x = SCREEN_W - 1;
    if (y >= SCREEN_H) y = SCREEN_H - 1;
    data->state = LV_INDEV_STATE_PR;
    data->point.x = x;
    data->point.y = y;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

/* =========================
   App logic
   ========================= */
enum class State : uint8_t { IDLE, RUNNING, DONE, STOPPED, ERROR };

static volatile uint32_t g_count = 0;
static uint32_t g_count_last_ui = 0;
static uint16_t g_target = 100;
static uint32_t g_debounce_ms = DEFAULT_DEBOUNCE_MS;
static volatile uint32_t g_last_pulse_ms = 0;

static Preferences prefs;

/* UI objects */
static lv_obj_t* lbl_title = nullptr;
static lv_obj_t* lbl_status = nullptr;
static lv_obj_t* lbl_count = nullptr;
static lv_obj_t* lbl_target = nullptr;
static lv_obj_t* bar = nullptr;
static lv_obj_t* ta_target = nullptr;

static lv_obj_t* kb = nullptr;

static lv_style_t st_btn_orange, st_btn_white, st_btn_stop, st_card, st_title, st_status, st_big, st_mid;

static State state = State::IDLE;

static const char* state_text(State s) {
  switch (s) {
    case State::IDLE:    return "Bereit";
    case State::RUNNING: return "Läuft…";
    case State::DONE:    return "Fertig: Bitte Band entnehmen";
    case State::STOPPED: return "Stopp";
    default:             return "Fehler";
  }
}

static void motor_set(bool on) {
  digitalWrite(GPIO_MOTOR, on ? MOTOR_ON_LEVEL : !MOTOR_ON_LEVEL);
}

static void ui_update_numbers() {
  char b1[32], b2[32];
  snprintf(b1, sizeof(b1), "IST: %lu", (unsigned long)g_count);
  snprintf(b2, sizeof(b2), "ZIEL: %u", (unsigned)g_target);
  lv_label_set_text(lbl_count, b1);
  lv_label_set_text(lbl_target, b2);

  if (g_target > 0) {
    uint32_t v = g_count;
    if (v > g_target) v = g_target;
    lv_bar_set_value(bar, (int)v, LV_ANIM_ON);
  }
}

static void ui_set_status(State s) {
  state = s;
  lv_label_set_text(lbl_status, state_text(s));
}

static void save_settings() {
  prefs.begin("bandware", false);
  prefs.putUShort("target", g_target);
  prefs.putUInt("deb_ms", g_debounce_ms);
  prefs.end();
}

static void load_settings() {
  prefs.begin("bandware", true);
  g_target = prefs.getUShort("target", 100);
  if (g_target < TARGET_MIN || g_target > TARGET_MAX) g_target = 100;
  g_debounce_ms = prefs.getUInt("deb_ms", DEFAULT_DEBOUNCE_MS);
  if (g_debounce_ms < 1 || g_debounce_ms > 50) g_debounce_ms = DEFAULT_DEBOUNCE_MS;
  prefs.end();
}

/* Button animation */
static void animate_button_press(lv_obj_t* btn) {
  // LVGL zoom: 256=100%
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, btn);
  lv_anim_set_time(&a, 80);
  lv_anim_set_values(&a, 256, 240);
  lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)[](void* obj, int32_t v){
    lv_obj_set_style_transform_zoom((lv_obj_t*)obj, v, 0);
  });
  lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
  lv_anim_start(&a);

  lv_anim_t b;
  lv_anim_init(&b);
  lv_anim_set_var(&b, btn);
  lv_anim_set_time(&b, 120);
  lv_anim_set_delay(&b, 90);
  lv_anim_set_values(&b, 240, 256);
  lv_anim_set_exec_cb(&b, (lv_anim_exec_xcb_t)[](void* obj, int32_t v){
    lv_obj_set_style_transform_zoom((lv_obj_t*)obj, v, 0);
  });
  lv_anim_set_path_cb(&b, lv_anim_path_ease_in_out);
  lv_anim_start(&b);
}

/* Keypad (LVGL keyboard numeric) */
static void kb_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t* obj = (lv_obj_t*)lv_event_get_target(e);

  if (code == LV_EVENT_READY) {
    const char* t = lv_textarea_get_text(ta_target);
    int v = atoi(t);
    if (v < TARGET_MIN) v = TARGET_MIN;
    if (v > TARGET_MAX) v = TARGET_MAX;
    g_target = (uint16_t)v;
    save_settings();
    ui_update_numbers();

    if (kb) { lv_obj_del(kb); kb = nullptr; }
    lv_obj_clear_state(ta_target, LV_STATE_FOCUSED);
  }
  else if (code == LV_EVENT_CANCEL) {
    if (kb) { lv_obj_del(kb); kb = nullptr; }
    lv_obj_clear_state(ta_target, LV_STATE_FOCUSED);
  }
  (void)obj;
}

static void ta_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_FOCUSED) {
    if (!kb) {
      kb = lv_keyboard_create(lv_scr_act());
      lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_NUMBER);
      lv_keyboard_set_textarea(kb, ta_target);
      lv_obj_set_size(kb, SCREEN_W, 220);
      lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, 0);
      lv_obj_add_event_cb(kb, kb_event_cb, LV_EVENT_ALL, nullptr);
    }
  }
}

/* State machine */
static void start_run() {
  if (g_target < TARGET_MIN || g_target > TARGET_MAX) return;
  g_count = 0;
  g_last_pulse_ms = 0;
  ui_update_numbers();
  ui_set_status(State::RUNNING);
  motor_set(true);
}

static void stop_run(State next) {
  motor_set(false);
  ui_set_status(next);
}

static void btn_start_cb(lv_event_t* e) {
  lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
  animate_button_press(btn);
  if (state != State::RUNNING) start_run();
}

static void btn_stop_cb(lv_event_t* e) {
  lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
  animate_button_press(btn);
  if (state == State::RUNNING) stop_run(State::STOPPED);
}

static void btn_reset_cb(lv_event_t* e) {
  lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
  animate_button_press(btn);
  motor_set(false);
  g_count = 0;
  ui_set_status(State::IDLE);
  ui_update_numbers();
}

/* Sensor ISR */
static void IRAM_ATTR sensor_isr() {
  uint32_t now = (uint32_t)millis();
  uint32_t last = g_last_pulse_ms;
  if ((now - last) >= g_debounce_ms) {
    g_last_pulse_ms = now;
    g_count++;
  }
}

/* Styles & UI */
static void init_styles() {
  lv_style_init(&st_card);
  lv_style_set_radius(&st_card, 14);
  lv_style_set_bg_color(&st_card, lv_color_hex(0xFFFFFF));
  lv_style_set_bg_opa(&st_card, LV_OPA_COVER);
  lv_style_set_pad_all(&st_card, 14);
  lv_style_set_border_width(&st_card, 1);
  lv_style_set_border_color(&st_card, lv_color_hex(0xE6E6E6));

  lv_style_init(&st_title);
  lv_style_set_text_font(&st_title, &lv_font_montserrat_28);
  lv_style_set_text_color(&st_title, lv_color_hex(0x111111));

  lv_style_init(&st_status);
  lv_style_set_text_font(&st_status, &lv_font_montserrat_18);
  lv_style_set_text_color(&st_status, lv_color_hex(0x333333));

  lv_style_init(&st_big);
  lv_style_set_text_font(&st_big, &lv_font_montserrat_48);
  lv_style_set_text_color(&st_big, lv_color_hex(0x111111));

  lv_style_init(&st_mid);
  lv_style_set_text_font(&st_mid, &lv_font_montserrat_22);
  lv_style_set_text_color(&st_mid, lv_color_hex(0x111111));

  // Orange button
  lv_style_init(&st_btn_orange);
  lv_style_set_radius(&st_btn_orange, 18);
  lv_style_set_bg_color(&st_btn_orange, lv_color_hex(0xFF7A00));
  lv_style_set_bg_opa(&st_btn_orange, LV_OPA_COVER);
  lv_style_set_text_color(&st_btn_orange, lv_color_hex(0xFFFFFF));
  lv_style_set_pad_ver(&st_btn_orange, 16);
  lv_style_set_pad_hor(&st_btn_orange, 20);
  lv_style_set_shadow_width(&st_btn_orange, 18);
  lv_style_set_shadow_opa(&st_btn_orange, LV_OPA_20);
  lv_style_set_shadow_ofs_y(&st_btn_orange, 6);

  // White button
  lv_style_init(&st_btn_white);
  lv_style_set_radius(&st_btn_white, 18);
  lv_style_set_bg_color(&st_btn_white, lv_color_hex(0xFFFFFF));
  lv_style_set_bg_opa(&st_btn_white, LV_OPA_COVER);
  lv_style_set_text_color(&st_btn_white, lv_color_hex(0x111111));
  lv_style_set_border_width(&st_btn_white, 2);
  lv_style_set_border_color(&st_btn_white, lv_color_hex(0xDDDDDD));
  lv_style_set_pad_ver(&st_btn_white, 16);
  lv_style_set_pad_hor(&st_btn_white, 20);
  lv_style_set_shadow_width(&st_btn_white, 14);
  lv_style_set_shadow_opa(&st_btn_white, LV_OPA_10);
  lv_style_set_shadow_ofs_y(&st_btn_white, 4);

  // Stop button (white + red accent)
  lv_style_init(&st_btn_stop);
  lv_style_set_radius(&st_btn_stop, 18);
  lv_style_set_bg_color(&st_btn_stop, lv_color_hex(0xFFFFFF));
  lv_style_set_bg_opa(&st_btn_stop, LV_OPA_COVER);
  lv_style_set_text_color(&st_btn_stop, lv_color_hex(0xC62828));
  lv_style_set_border_width(&st_btn_stop, 2);
  lv_style_set_border_color(&st_btn_stop, lv_color_hex(0xC62828));
  lv_style_set_pad_ver(&st_btn_stop, 16);
  lv_style_set_pad_hor(&st_btn_stop, 20);
  lv_style_set_shadow_width(&st_btn_stop, 14);
  lv_style_set_shadow_opa(&st_btn_stop, LV_OPA_10);
  lv_style_set_shadow_ofs_y(&st_btn_stop, 4);
}

static lv_obj_t* make_btn(lv_obj_t* parent, const char* text, lv_style_t* st, lv_event_cb_t cb) {
  lv_obj_t* btn = lv_btn_create(parent);
  lv_obj_add_style(btn, st, 0);
  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* t = lv_label_create(btn);
  lv_label_set_text(t, text);
  lv_obj_center(t);
  lv_obj_set_style_text_font(t, &lv_font_montserrat_24, 0);

  // allow transform zoom animation
  lv_obj_set_style_transform_pivot_x(btn, 0, 0);
  lv_obj_set_style_transform_pivot_y(btn, 0, 0);
  lv_obj_set_style_transform_zoom(btn, 256, 0);
  return btn;
}

static void build_ui() {
  lv_obj_t* scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0xF6F7F8), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  lv_obj_t* card = lv_obj_create(scr);
  lv_obj_add_style(card, &st_card, 0);
  lv_obj_set_size(card, SCREEN_W - 40, SCREEN_H - 40);
  lv_obj_align(card, LV_ALIGN_CENTER, 0, 0);

  lbl_title = lv_label_create(card);
  lv_label_set_text(lbl_title, "Bandware Zähler");
  lv_obj_add_style(lbl_title, &st_title, 0);
  lv_obj_align(lbl_title, LV_ALIGN_TOP_LEFT, 0, 0);

  lbl_status = lv_label_create(card);
  lv_obj_add_style(lbl_status, &st_status, 0);
  lv_obj_align(lbl_status, LV_ALIGN_TOP_RIGHT, 0, 4);

  // Target input
  lv_obj_t* t1 = lv_label_create(card);
  lv_label_set_text(t1, "Zielmenge (1..1000)");
  lv_obj_add_style(t1, &st_mid, 0);
  lv_obj_align(t1, LV_ALIGN_TOP_LEFT, 0, 52);

  ta_target = lv_textarea_create(card);
  lv_obj_set_width(ta_target, 220);
  lv_textarea_set_one_line(ta_target, true);
  lv_textarea_set_text(ta_target, String(g_target).c_str());
  lv_obj_align(ta_target, LV_ALIGN_TOP_LEFT, 0, 92);
  lv_obj_add_event_cb(ta_target, ta_event_cb, LV_EVENT_ALL, nullptr);

  // Big count + target labels
  lbl_count = lv_label_create(card);
  lv_obj_add_style(lbl_count, &st_big, 0);
  lv_obj_align(lbl_count, LV_ALIGN_LEFT_MID, 0, -20);

  lbl_target = lv_label_create(card);
  lv_obj_add_style(lbl_target, &st_mid, 0);
  lv_obj_align(lbl_target, LV_ALIGN_LEFT_MID, 0, 40);

  // Progress bar
  bar = lv_bar_create(card);
  lv_obj_set_size(bar, SCREEN_W - 80, 18);
  lv_obj_align(bar, LV_ALIGN_BOTTOM_MID, 0, -110);
  lv_bar_set_range(bar, 0, TARGET_MAX);
  lv_obj_set_style_bg_color(bar, lv_color_hex(0xEAEAEA), 0);
  lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(bar, 10, 0);
  lv_obj_set_style_bg_color(bar, lv_color_hex(0xFF7A00), LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_radius(bar, 10, LV_PART_INDICATOR);

  // Buttons row
  lv_obj_t* row = lv_obj_create(card);
  lv_obj_remove_style_all(row);
  lv_obj_set_size(row, SCREEN_W - 80, 80);
  lv_obj_align(row, LV_ALIGN_BOTTOM_MID, 0, -20);
  lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  lv_obj_t* b_start = make_btn(row, "START", &st_btn_orange, btn_start_cb);
  lv_obj_set_size(b_start, 220, 80);

  lv_obj_t* b_stop = make_btn(row, "STOP", &st_btn_stop, btn_stop_cb);
  lv_obj_set_size(b_stop, 220, 80);

  lv_obj_t* b_reset = make_btn(row, "RESET", &st_btn_white, btn_reset_cb);
  lv_obj_set_size(b_reset, 220, 80);

  ui_set_status(State::IDLE);
  ui_update_numbers();
}

/* LVGL init */
static void init_lvgl() {
  lv_init();

  lv_disp_draw_buf_init(&draw_buf, buf1, nullptr, SCREEN_W * LV_BUF_LINES);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCREEN_W;
  disp_drv.ver_res = SCREEN_H;
  disp_drv.flush_cb = flush_cb;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = lv_touch_cb;
  lv_indev_drv_register(&indev_drv);
}

/* =========================
   Setup / Loop
   ========================= */
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] Bandware Zaehler (ESP32-S3 + LVGL8 + RGB) ");

  pinMode(GPIO_MOTOR, OUTPUT);
  motor_set(false);

  pinMode(GPIO_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GPIO_SENSOR), sensor_isr, FALLING);

  load_settings();

  lcd.begin();

  // Touch I2C
  Wire.begin(TOUCH_I2C_SDA, TOUCH_I2C_SCL, 400000);
  if (gt911_detect()) {
    Serial.print("[TOUCH] GT911 found at 0x");
    Serial.println(g_gt911_addr, HEX);
  } else {
    Serial.println("[TOUCH] GT911 not detected (check SDA/SCL or address). UI will still run.");
  }

  init_lvgl();
  init_styles();
  build_ui();

  ui_update_numbers();
}

void loop() {
  // UI updates (throttle)
  if (g_count != g_count_last_ui) {
    g_count_last_ui = g_count;
    ui_update_numbers();

    if (state == State::RUNNING && g_target > 0 && g_count >= g_target) {
      stop_run(State::DONE);
      // little message box
      lv_obj_t* m = lv_msgbox_create(nullptr, "Fertig", "Fertig: Bitte Band entnehmen", nullptr, true);
      lv_obj_center(m);
    }
  }

  lv_timer_handler();
  delay(5);
}
