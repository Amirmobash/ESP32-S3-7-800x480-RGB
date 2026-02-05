#define LGFX_USE_V1
#include <Arduino.h>
#include <lvgl.h>

#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>

#include <Wire.h>
#include <Preferences.h>

// =====================================================
// CONFIG (فقط اینجا را اگر لازم شد تغییر بده)
// =====================================================
static const int SCREEN_W = 800;
static const int SCREEN_H = 480;

// GPIOs پروژه
static const int GPIO_SENSOR = 13;   // ورودی سنسور بعد از PC817 (LOW = پالس)
static const int GPIO_MOTOR  = 12;   // خروجی کنترل موتور (HIGH=ON)

// سنسور: debounce
static volatile uint32_t g_lastPulseUs = 0;
static volatile uint32_t g_count_isr   = 0;
static uint32_t g_debounce_us = 3000; // 3ms (قابل تغییر از Settings)

// هدف شمارش
static uint32_t g_target = 100;

// تاچ GT911 (اگر برد شما فرق داشت این پین‌ها را عوض کن)
static const int TOUCH_I2C_SDA = 17;
static const int TOUCH_I2C_SCL = 18;
static const int TOUCH_RST_PIN = -1;   // اگر نداری -1
static const int TOUCH_INT_PIN = -1;   // اگر نداری -1
static const uint8_t GT911_ADDR = 0x5D; // گاهی 0x14 است (اگر تاچ کار نکرد، 0x14 کن)

// =====================================================
// LovyanGFX RGB (همان پین‌مپ شما)
// =====================================================
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[SCREEN_W * 20];

class LGFX : public lgfx::LGFX_Device {
  lgfx::Bus_RGB bus;
  lgfx::Panel_RGB panel;

public:
  LGFX() {
    {
      auto cfg = bus.config();

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

      bus.config(cfg);
      panel.setBus(&bus);
    }

    {
      auto cfg = panel.config();
      cfg.panel_width   = SCREEN_W;
      cfg.panel_height  = SCREEN_H;
      cfg.memory_width  = SCREEN_W;
      cfg.memory_height = SCREEN_H;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      panel.config(cfg);
    }

    setPanel(&panel);
  }
};

static LGFX lcd;

// =====================================================
// GT911 minimal driver (بدون کتابخانه اضافه)
// =====================================================
struct TouchPoint {
  bool touched;
  uint16_t x;
  uint16_t y;
};

static bool i2c_read(uint8_t addr, uint16_t reg, uint8_t* data, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg & 0xFF);
  Wire.write((reg >> 8) & 0xFF);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)len) != (int)len) return false;
  for (size_t i = 0; i < len; i++) data[i] = Wire.read();
  return true;
}

static bool i2c_write8(uint8_t addr, uint16_t reg, uint8_t v) {
  Wire.beginTransmission(addr);
  Wire.write(reg & 0xFF);
  Wire.write((reg >> 8) & 0xFF);
  Wire.write(v);
  return Wire.endTransmission() == 0;
}

static bool gt911_begin() {
  Wire.begin(TOUCH_I2C_SDA, TOUCH_I2C_SCL, 400000);

  // optional reset
  if (TOUCH_RST_PIN >= 0) {
    pinMode(TOUCH_RST_PIN, OUTPUT);
    digitalWrite(TOUCH_RST_PIN, 0);
    delay(10);
    digitalWrite(TOUCH_RST_PIN, 1);
    delay(50);
  }
  if (TOUCH_INT_PIN >= 0) {
    pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
  }

  // check product id
  uint8_t id[4] = {0};
  if (!i2c_read(GT911_ADDR, 0x8140, id, 4)) return false;

  Serial.printf("GT911 ID: %c%c%c%c\n", id[0], id[1], id[2], id[3]);
  return true;
}

static TouchPoint gt911_read() {
  TouchPoint p{false, 0, 0};

  uint8_t status = 0;
  if (!i2c_read(GT911_ADDR, 0x814E, &status, 1)) return p;

  uint8_t n = status & 0x0F;
  if ((status & 0x80) == 0 || n == 0) {
    // clear if needed
    i2c_write8(GT911_ADDR, 0x814E, 0x00);
    return p;
  }

  // first point at 0x8150 (8 bytes per point)
  uint8_t buf[8];
  if (!i2c_read(GT911_ADDR, 0x8150, buf, 8)) {
    i2c_write8(GT911_ADDR, 0x814E, 0x00);
    return p;
  }

  uint16_t x = (uint16_t)buf[1] << 8 | buf[0];
  uint16_t y = (uint16_t)buf[3] << 8 | buf[2];

  // clear "data ready"
  i2c_write8(GT911_ADDR, 0x814E, 0x00);

  // clamp
  if (x >= SCREEN_W) x = SCREEN_W - 1;
  if (y >= SCREEN_H) y = SCREEN_H - 1;

  p.touched = true;
  p.x = x;
  p.y = y;
  return p;
}

// =====================================================
// LVGL glue
// =====================================================
static void flush_cb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;

  lcd.startWrite();
  lcd.setAddrWindow(area->x1, area->y1, w, h);
  lcd.writePixels((lgfx::rgb565_t*)color_p, w * h);
  lcd.endWrite();

  lv_disp_flush_ready(disp);
}

static void touch_cb(lv_indev_drv_t* indev, lv_indev_data_t* data) {
  (void)indev;
  TouchPoint p = gt911_read();

  if (!p.touched) {
    data->state = LV_INDEV_STATE_REL;
    return;
  }
  data->state = LV_INDEV_STATE_PR;
  data->point.x = p.x;
  data->point.y = p.y;
}

// =====================================================
// App logic
// =====================================================
enum class State : uint8_t { IDLE, RUNNING, DONE, STOPPED, ERROR };
static State g_state = State::IDLE;

static Preferences prefs;

// UI objects
static lv_obj_t* label_title;
static lv_obj_t* label_status;
static lv_obj_t* label_count;
static lv_obj_t* label_target;
static lv_obj_t* bar_progress;
static lv_obj_t* btn_start;
static lv_obj_t* btn_stop;
static lv_obj_t* btn_reset;
static lv_obj_t* btn_target;
static lv_obj_t* panel_keypad = nullptr;
static lv_obj_t* panel_settings = nullptr;
static lv_obj_t* slider_debounce = nullptr;

static lv_style_t st_bg;
static lv_style_t st_orange_btn;
static lv_style_t st_white_btn;
static lv_style_t st_red_btn;
static lv_style_t st_text_big;
static lv_style_t st_text_mid;
static lv_style_t st_status;

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static const char* state_text(State s) {
  switch (s) {
    case State::IDLE:    return "Bereit";
    case State::RUNNING: return "Läuft…";
    case State::DONE:    return "Fertig: Bitte Band entnehmen";
    case State::STOPPED: return "Stopp";
    case State::ERROR:   return "Fehler";
    default:             return "";
  }
}

static void motor_set(bool on) {
  digitalWrite(GPIO_MOTOR, on ? HIGH : LOW);
}

static void ui_update_numbers() {
  uint32_t c = g_count_isr;
  char buf[64];

  snprintf(buf, sizeof(buf), "%lu", (unsigned long)c);
  lv_label_set_text(label_count, buf);

  snprintf(buf, sizeof(buf), "Ziel: %lu", (unsigned long)g_target);
  lv_label_set_text(label_target, buf);

  uint32_t pct = (g_target == 0) ? 0 : (uint32_t)((c * 100UL) / g_target);
  pct = clamp_u32(pct, 0, 100);
  lv_bar_set_value(bar_progress, (int)pct, LV_ANIM_ON);
}

static void ui_set_status(State s) {
  g_state = s;
  lv_label_set_text(label_status, state_text(s));

  // color hint
  if (s == State::RUNNING) {
    lv_obj_set_style_text_color(label_status, lv_color_hex(0xFF7A00), 0);
  } else if (s == State::DONE) {
    lv_obj_set_style_text_color(label_status, lv_color_hex(0x2E7D32), 0);
  } else if (s == State::ERROR) {
    lv_obj_set_style_text_color(label_status, lv_color_hex(0xC62828), 0);
  } else {
    lv_obj_set_style_text_color(label_status, lv_color_hex(0x333333), 0);
  }
}

static void stop_run(State next) {
  motor_set(false);
  ui_set_status(next);
}

static void start_run() {
  g_count_isr = 0;
  ui_update_numbers();
  motor_set(true);
  ui_set_status(State::RUNNING);
}

static void animate_button_press(lv_obj_t* btn) {
  if (!btn) return;
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, btn);
  lv_anim_set_time(&a, 90);
  lv_anim_set_values(&a, 100, 96);
  lv_anim_set_exec_cb(&a, [](void* v, int32_t val) {
    lv_obj_t* o = (lv_obj_t*)v;
    lv_obj_set_style_transform_zoom(o, (int16_t)val, 0);
  });
  lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
  lv_anim_start(&a);

  lv_anim_t b;
  lv_anim_init(&b);
  lv_anim_set_var(&b, btn);
  lv_anim_set_time(&b, 120);
  lv_anim_set_delay(&b, 90);
  lv_anim_set_values(&b, 96, 100);
  lv_anim_set_exec_cb(&b, [](void* v, int32_t val) {
    lv_obj_t* o = (lv_obj_t*)v;
    lv_obj_set_style_transform_zoom(o, (int16_t)val, 0);
  });
  lv_anim_set_path_cb(&b, lv_anim_path_ease_in);
  lv_anim_start(&b);
}

static void keypad_close() {
  if (panel_keypad) {
    lv_obj_del(panel_keypad);
    panel_keypad = nullptr;
  }
}

static void keypad_apply_text(const char* txt) {
  uint32_t v = (uint32_t)atoi(txt);
  v = clamp_u32(v, 1, 1000);
  g_target = v;
  prefs.putUInt("target", g_target);
  ui_update_numbers();
}

static void keypad_btn_cb(lv_event_t* e) {
  lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
  const char* t = lv_label_get_text(lv_obj_get_child(btn, 0));

  static char inbuf[8] = {0}; // up to 1000
  size_t len = strlen(inbuf);

  if (!strcmp(t, "OK")) {
    if (len == 0) strcpy(inbuf, "1");
    keypad_apply_text(inbuf);
    inbuf[0] = 0;
    keypad_close();
    return;
  }
  if (!strcmp(t, "C")) {
    inbuf[0] = 0;
  } else if (!strcmp(t, "<")) {
    if (len > 0) inbuf[len - 1] = 0;
  } else {
    if (len < sizeof(inbuf) - 1) {
      inbuf[len] = t[0];
      inbuf[len + 1] = 0;
    }
  }

  // update display
  lv_obj_t* lbl = lv_obj_get_child(panel_keypad, 0);
  char shown[32];
  snprintf(shown, sizeof(shown), "Zielmenge: %s", (strlen(inbuf) ? inbuf : "_"));
  lv_label_set_text(lbl, shown);
}

static lv_obj_t* make_kbtn(lv_obj_t* parent, const char* txt) {
  lv_obj_t* b = lv_btn_create(parent);
  lv_obj_set_size(b, 90, 70);
  lv_obj_add_event_cb(b, keypad_btn_cb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* l = lv_label_create(b);
  lv_label_set_text(l, txt);
  lv_obj_center(l);

  lv_obj_set_style_text_font(l, &lv_font_montserrat_28, 0);
  return b;
}

static void open_keypad() {
  keypad_close();

  panel_keypad = lv_obj_create(lv_scr_act());
  lv_obj_set_size(panel_keypad, 520, 360);
  lv_obj_center(panel_keypad);
  lv_obj_set_style_radius(panel_keypad, 18, 0);
  lv_obj_set_style_bg_color(panel_keypad, lv_color_white(), 0);
  lv_obj_set_style_pad_all(panel_keypad, 14, 0);
  lv_obj_set_style_shadow_width(panel_keypad, 22, 0);
  lv_obj_set_style_shadow_opa(panel_keypad, LV_OPA_30, 0);

  lv_obj_t* top = lv_label_create(panel_keypad);
  lv_label_set_text(top, "Zielmenge: _");
  lv_obj_set_style_text_font(top, &lv_font_montserrat_28, 0);
  lv_obj_align(top, LV_ALIGN_TOP_MID, 0, 0);

  lv_obj_t* grid = lv_obj_create(panel_keypad);
  lv_obj_set_size(grid, 500, 280);
  lv_obj_align(grid, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_opa(grid, LV_OPA_0, 0);
  lv_obj_set_style_border_width(grid, 0, 0);
  lv_obj_set_style_pad_all(grid, 6, 0);

  static lv_coord_t col[] = {100,100,100,100, LV_GRID_TEMPLATE_LAST};
  static lv_coord_t row[] = {80,80,80,80, LV_GRID_TEMPLATE_LAST};
  lv_obj_set_grid_dsc_array(grid, col, row);

  const char* keys[16] = {
    "1","2","3","<",
    "4","5","6","C",
    "7","8","9","OK",
    "0","","",""
  };

  for (int i = 0; i < 16; i++) {
    if (keys[i][0] == 0) continue;
    lv_obj_t* b = make_kbtn(grid, keys[i]);
    lv_obj_set_grid_cell(b, LV_GRID_ALIGN_CENTER, i % 4, 1, LV_GRID_ALIGN_CENTER, i / 4, 1);
  }
}

static void settings_close() {
  if (panel_settings) {
    lv_obj_del(panel_settings);
    panel_settings = nullptr;
  }
}

static void slider_cb(lv_event_t* e) {
  lv_obj_t* sl = (lv_obj_t*)lv_event_get_target(e);
  int v = lv_slider_get_value(sl);
  g_debounce_us = (uint32_t)v * 1000UL;
  prefs.putUInt("deb_ms", (uint32_t)v);

  // update small label
  lv_obj_t* lbl = (lv_obj_t*)lv_event_get_user_data(e);
  char buf[32];
  snprintf(buf, sizeof(buf), "Entprellung: %d ms", v);
  lv_label_set_text(lbl, buf);
}

static void open_settings() {
  settings_close();

  panel_settings = lv_obj_create(lv_scr_act());
  lv_obj_set_size(panel_settings, 520, 240);
  lv_obj_center(panel_settings);
  lv_obj_set_style_radius(panel_settings, 18, 0);
  lv_obj_set_style_bg_color(panel_settings, lv_color_white(), 0);
  lv_obj_set_style_pad_all(panel_settings, 16, 0);
  lv_obj_set_style_shadow_width(panel_settings, 22, 0);
  lv_obj_set_style_shadow_opa(panel_settings, LV_OPA_30, 0);

  lv_obj_t* t = lv_label_create(panel_settings);
  lv_label_set_text(t, "Einstellungen");
  lv_obj_set_style_text_font(t, &lv_font_montserrat_28, 0);
  lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 0);

  uint32_t deb_ms = prefs.getUInt("deb_ms", 3);
  deb_ms = clamp_u32(deb_ms, 1, 20);

  lv_obj_t* lbl = lv_label_create(panel_settings);
  char buf[32];
  snprintf(buf, sizeof(buf), "Entprellung: %lu ms", (unsigned long)deb_ms);
  lv_label_set_text(lbl, buf);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
  lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 0, 60);

  slider_debounce = lv_slider_create(panel_settings);
  lv_obj_set_width(slider_debounce, 480);
  lv_obj_align(slider_debounce, LV_ALIGN_TOP_LEFT, 0, 95);
  lv_slider_set_range(slider_debounce, 1, 20);
  lv_slider_set_value(slider_debounce, (int)deb_ms, LV_ANIM_OFF);
  lv_obj_add_event_cb(slider_debounce, slider_cb, LV_EVENT_VALUE_CHANGED, lbl);

  lv_obj_t* closeb = lv_btn_create(panel_settings);
  lv_obj_set_size(closeb, 150, 55);
  lv_obj_align(closeb, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  lv_obj_add_event_cb(closeb, [](lv_event_t* e){
    (void)e;
    settings_close();
  }, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* cl = lv_label_create(closeb);
  lv_label_set_text(cl, "OK");
  lv_obj_set_style_text_font(cl, &lv_font_montserrat_28, 0);
  lv_obj_center(cl);
}

// Button callbacks
static void btn_start_cb(lv_event_t* e) {
  lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
  animate_button_press(btn);
  if (g_state != State::RUNNING) start_run();
}

static void btn_stop_cb(lv_event_t* e) {
  lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
  animate_button_press(btn);
  if (g_state == State::RUNNING) stop_run(State::STOPPED);
}

static void btn_reset_cb(lv_event_t* e) {
  lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
  animate_button_press(btn);
  motor_set(false);
  g_count_isr = 0;
  ui_set_status(State::IDLE);
  ui_update_numbers();
}

static void btn_target_cb(lv_event_t* e) {
  lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
  animate_button_press(btn);
  open_keypad();
}

static void btn_settings_cb(lv_event_t* e) {
  lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
  animate_button_press(btn);
  open_settings();
}

// Styles + UI
static void init_styles() {
  lv_style_init(&st_bg);
  lv_style_set_bg_color(&st_bg, lv_color_hex(0xF3F4F6));
  lv_style_set_bg_opa(&st_bg, LV_OPA_COVER);

  lv_style_init(&st_orange_btn);
  lv_style_set_bg_color(&st_orange_btn, lv_color_hex(0xFF7A00));
  lv_style_set_bg_opa(&st_orange_btn, LV_OPA_COVER);
  lv_style_set_radius(&st_orange_btn, 16);
  lv_style_set_shadow_width(&st_orange_btn, 16);
  lv_style_set_shadow_opa(&st_orange_btn, LV_OPA_30);
  lv_style_set_shadow_ofs_y(&st_orange_btn, 6);

  lv_style_init(&st_white_btn);
  lv_style_set_bg_color(&st_white_btn, lv_color_white());
  lv_style_set_bg_opa(&st_white_btn, LV_OPA_COVER);
  lv_style_set_border_color(&st_white_btn, lv_color_hex(0xD0D5DD));
  lv_style_set_border_width(&st_white_btn, 2);
  lv_style_set_radius(&st_white_btn, 16);

  lv_style_init(&st_red_btn);
  lv_style_set_bg_color(&st_red_btn, lv_color_hex(0xD32F2F));
  lv_style_set_bg_opa(&st_red_btn, LV_OPA_COVER);
  lv_style_set_radius(&st_red_btn, 16);
  lv_style_set_shadow_width(&st_red_btn, 16);
  lv_style_set_shadow_opa(&st_red_btn, LV_OPA_25);
  lv_style_set_shadow_ofs_y(&st_red_btn, 6);

  lv_style_init(&st_text_big);
  lv_style_set_text_font(&st_text_big, &lv_font_montserrat_28);
  lv_style_set_text_color(&st_text_big, lv_color_hex(0x111827));

  lv_style_init(&st_text_mid);
  lv_style_set_text_font(&st_text_mid, &lv_font_montserrat_16);
  lv_style_set_text_color(&st_text_mid, lv_color_hex(0x374151));

  lv_style_init(&st_status);
  lv_style_set_text_font(&st_status, &lv_font_montserrat_16);
  lv_style_set_text_color(&st_status, lv_color_hex(0x333333));
}

static lv_obj_t* make_btn(lv_obj_t* parent, const char* txt, lv_style_t* st, lv_event_cb_t cb) {
  lv_obj_t* b = lv_btn_create(parent);
  lv_obj_add_style(b, st, 0);
  lv_obj_set_size(b, 220, 80);
  lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* l = lv_label_create(b);
  lv_label_set_text(l, txt);
  lv_obj_center(l);
  lv_obj_set_style_text_font(l, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(l, lv_color_white(), 0);
  return b;
}

static void build_ui() {
  lv_obj_t* scr = lv_scr_act();
  lv_obj_add_style(scr, &st_bg, 0);

  // title
  label_title = lv_label_create(scr);
  lv_label_set_text(label_title, "Bandware Zähler");
  lv_obj_add_style(label_title, &st_text_big, 0);
  lv_obj_align(label_title, LV_ALIGN_TOP_LEFT, 20, 16);

  // status
  label_status = lv_label_create(scr);
  lv_label_set_text(label_status, "Bereit");
  lv_obj_add_style(label_status, &st_status, 0);
  lv_obj_align(label_status, LV_ALIGN_TOP_LEFT, 20, 56);

  // big counter
  label_count = lv_label_create(scr);
  lv_label_set_text(label_count, "0");
  lv_obj_set_style_text_font(label_count, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(label_count, lv_color_hex(0x111827), 0);
  lv_obj_align(label_count, LV_ALIGN_LEFT_MID, 40, -40);

  // target label
  label_target = lv_label_create(scr);
  lv_label_set_text(label_target, "Ziel: 100");
  lv_obj_add_style(label_target, &st_text_mid, 0);
  lv_obj_align(label_target, LV_ALIGN_LEFT_MID, 40, 0);

  // progress
  bar_progress = lv_bar_create(scr);
  lv_obj_set_size(bar_progress, 520, 26);
  lv_obj_align(bar_progress, LV_ALIGN_LEFT_MID, 40, 50);
  lv_bar_set_range(bar_progress, 0, 100);
  lv_bar_set_value(bar_progress, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(bar_progress, lv_color_hex(0xE5E7EB), 0);
  lv_obj_set_style_bg_color(bar_progress, lv_color_hex(0xFF7A00), LV_PART_INDICATOR);

  // buttons (right side)
  btn_start = make_btn(scr, "START", &st_orange_btn, btn_start_cb);
  lv_obj_align(btn_start, LV_ALIGN_RIGHT_MID, -30, -100);

  btn_stop = make_btn(scr, "STOP", &st_red_btn, btn_stop_cb);
  lv_obj_align(btn_stop, LV_ALIGN_RIGHT_MID, -30, 0);

  btn_reset = make_btn(scr, "RESET", &st_white_btn, btn_reset_cb);
  lv_obj_align(btn_reset, LV_ALIGN_RIGHT_MID, -30, 100);
  // reset text black
  lv_obj_t* rlbl = lv_obj_get_child(btn_reset, 0);
  lv_obj_set_style_text_color(rlbl, lv_color_hex(0x111827), 0);

  // target button (bottom left)
  btn_target = lv_btn_create(scr);
  lv_obj_add_style(btn_target, &st_white_btn, 0);
  lv_obj_set_size(btn_target, 240, 60);
  lv_obj_align(btn_target, LV_ALIGN_BOTTOM_LEFT, 20, -20);
  lv_obj_add_event_cb(btn_target, btn_target_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* tt = lv_label_create(btn_target);
  lv_label_set_text(tt, "Zielmenge");
  lv_obj_set_style_text_font(tt, &lv_font_montserrat_16, 0);
  lv_obj_center(tt);

  // settings button
  lv_obj_t* btn_set = lv_btn_create(scr);
  lv_obj_add_style(btn_set, &st_white_btn, 0);
  lv_obj_set_size(btn_set, 240, 60);
  lv_obj_align(btn_set, LV_ALIGN_BOTTOM_LEFT, 280, -20);
  lv_obj_add_event_cb(btn_set, btn_settings_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* st = lv_label_create(btn_set);
  lv_label_set_text(st, "Einstellungen");
  lv_obj_set_style_text_font(st, &lv_font_montserrat_16, 0);
  lv_obj_center(st);

  ui_update_numbers();
  ui_set_status(State::IDLE);
}

// =====================================================
// ISR sensor
// =====================================================
static void IRAM_ATTR sensor_isr() {
  uint32_t now = (uint32_t)micros();
  if (now - g_lastPulseUs < g_debounce_us) return;
  g_lastPulseUs = now;
  g_count_isr++;

  // stop at target (only set flag-ish by checking count)
  // real stop in loop (safe, no heavy things in ISR)
}

// =====================================================
// Setup / Loop
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // GPIO
  pinMode(GPIO_MOTOR, OUTPUT);
  motor_set(false);

  pinMode(GPIO_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GPIO_SENSOR), sensor_isr, FALLING);

  // Preferences
  prefs.begin("bandware", false);
  g_target = prefs.getUInt("target", 100);
  g_target = clamp_u32(g_target, 1, 1000);
  uint32_t deb_ms = prefs.getUInt("deb_ms", 3);
  deb_ms = clamp_u32(deb_ms, 1, 20);
  g_debounce_us = deb_ms * 1000UL;

  // LCD
  lcd.begin();

  // LVGL
  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf1, nullptr, SCREEN_W * 20);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCREEN_W;
  disp_drv.ver_res = SCREEN_H;
  disp_drv.flush_cb = flush_cb;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  // Touch
  bool touch_ok = gt911_begin();
  if (touch_ok) {
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_cb;
    lv_indev_drv_register(&indev_drv);
  } else {
    Serial.println("WARN: GT911 not found. Try GT911_ADDR 0x14 or check SDA/SCL pins.");
  }

  init_styles();
  build_ui();
}

void loop() {
  // UI/task loop
  lv_timer_handler();
  delay(5);

  // RUNNING logic
  if (g_state == State::RUNNING) {
    uint32_t c = g_count_isr;
    if (c >= g_target) {
      motor_set(false);
      ui_set_status(State::DONE);
    }
    ui_update_numbers();
  } else {
    // update sometimes (cheap)
    static uint32_t last = 0;
    uint32_t ms = millis();
    if (ms - last > 150) {
      last = ms;
      ui_update_numbers();
    }
  }
}
