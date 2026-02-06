#define LGFX_USE_V1
#include <Arduino.h>
#include <lvgl.h>
#include <Preferences.h>
#include <Wire.h>

#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>

/* =========================
   CONFIG (ONLY EDIT HERE)
   ========================= */

// Display resolution
static const int SCREEN_W = 800;
static const int SCREEN_H = 480;

// Your proven RGB pinmap (from your working test)
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

static const gpio_num_t PIN_DE    = GPIO_NUM_41; // LovyanGFX uses pin_henable
static const gpio_num_t PIN_VSYNC = GPIO_NUM_40;
static const gpio_num_t PIN_HSYNC = GPIO_NUM_39;
static const gpio_num_t PIN_PCLK  = GPIO_NUM_42;

// RGB timing (you can tweak if black screen/flicker)
static const uint32_t RGB_FREQ_HZ = 12000000;

static const int HSYNC_FRONT = 8;
static const int HSYNC_PULSE = 2;
static const int HSYNC_BACK  = 43;

static const int VSYNC_FRONT = 8;
static const int VSYNC_PULSE = 2;
static const int VSYNC_BACK  = 12;

static const bool PCLK_IDLE_HIGH = true;

// IO for machine
static const int GPIO_SENSOR = 10;   // CHANGE if needed
static const int GPIO_MOTOR  = 11;   // CHANGE if needed
static const bool MOTOR_ON_LEVEL = HIGH;

// GT911 touch (typical)
static const int TOUCH_I2C_SDA = 17; // CHANGE if needed
static const int TOUCH_I2C_SCL = 18; // CHANGE if needed
static const int TOUCH_RST_PIN = -1; // if your board has it, set pin number
static const int TOUCH_INT_PIN = -1; // optional
static const uint8_t GT911_ADDR = 0x5D; // sometimes 0x14, but 0x5D is common

// Debounce default
static const uint16_t DEBOUNCE_MS_DEFAULT = 5;

// LVGL buffer lines (bigger = faster but more RAM)
static const int LVGL_BUF_LINES = 20;

/* =========================
   LovyanGFX (RGB)
   ========================= */
class LGFX : public lgfx::LGFX_Device {
  lgfx::Bus_RGB bus;
  lgfx::Panel_RGB panel;
public:
  LGFX() {
    { // bus
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

      cfg.pin_henable = PIN_DE;     // DE
      cfg.pin_vsync   = PIN_VSYNC;
      cfg.pin_hsync   = PIN_HSYNC;
      cfg.pin_pclk    = PIN_PCLK;

      cfg.freq_write = RGB_FREQ_HZ;

      cfg.hsync_polarity    = 0;
      cfg.hsync_front_porch = HSYNC_FRONT;
      cfg.hsync_pulse_width = HSYNC_PULSE;
      cfg.hsync_back_porch  = HSYNC_BACK;

      cfg.vsync_polarity    = 0;
      cfg.vsync_front_porch = VSYNC_FRONT;
      cfg.vsync_pulse_width = VSYNC_PULSE;
      cfg.vsync_back_porch  = VSYNC_BACK;

      cfg.pclk_idle_high = PCLK_IDLE_HIGH;

      bus.config(cfg);
      panel.setBus(&bus);
    }
    { // panel
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

/* =========================
   GT911 (minimal)
   ========================= */
static bool gt911_ok = false;

static bool i2c_write_u16(uint8_t addr, uint16_t reg, const uint8_t* data, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)(reg & 0xFF));
  Wire.write((uint8_t)((reg >> 8) & 0xFF));
  for (size_t i = 0; i < len; i++) Wire.write(data[i]);
  return Wire.endTransmission() == 0;
}
static bool i2c_read_u16(uint8_t addr, uint16_t reg, uint8_t* data, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)(reg & 0xFF));
  Wire.write((uint8_t)((reg >> 8) & 0xFF));
  if (Wire.endTransmission(false) != 0) return false;
  size_t r = Wire.requestFrom((int)addr, (int)len);
  if (r != len) return false;
  for (size_t i = 0; i < len; i++) data[i] = Wire.read();
  return true;
}

// GT911 regs
static const uint16_t GT_REG_STATUS = 0x814E;
static const uint16_t GT_REG_POINTS = 0x8150;

static void gt911_reset() {
  if (TOUCH_RST_PIN >= 0) {
    pinMode(TOUCH_RST_PIN, OUTPUT);
    digitalWrite(TOUCH_RST_PIN, LOW);
    delay(10);
    digitalWrite(TOUCH_RST_PIN, HIGH);
    delay(60);
  }
}

static bool gt911_init() {
  Wire.begin(TOUCH_I2C_SDA, TOUCH_I2C_SCL, 400000);
  gt911_reset();

  uint8_t tmp[4];
  // Read some bytes from status register to see if device responds
  if (!i2c_read_u16(GT911_ADDR, GT_REG_STATUS, tmp, 1)) {
    // try alternate address
    uint8_t alt = 0x14;
    if (i2c_read_u16(alt, GT_REG_STATUS, tmp, 1)) {
      // If your module uses 0x14, change GT911_ADDR above.
      Serial.println("[TOUCH] GT911 responds on 0x14. Please set GT911_ADDR=0x14.");
      return true; // but we keep ok=false to force you to fix addr
    }
    return false;
  }
  return true;
}

static bool gt911_read(int& x, int& y, bool& pressed) {
  pressed = false;
  uint8_t st = 0;
  if (!i2c_read_u16(GT911_ADDR, GT_REG_STATUS, &st, 1)) return false;

  uint8_t n = st & 0x0F;
  if ((st & 0x80) == 0 || n == 0) {
    // Clear status
    uint8_t z = 0;
    i2c_write_u16(GT911_ADDR, GT_REG_STATUS, &z, 1);
    return true;
  }

  uint8_t buf[8];
  if (!i2c_read_u16(GT911_ADDR, GT_REG_POINTS, buf, 8)) return false;

  // GT911: X = buf[1]<<8 | buf[0]? (commonly little endian inside)
  // Many boards: x = buf[1]<<8 | buf[0], y = buf[3]<<8 | buf[2]
  x = ((int)buf[1] << 8) | buf[0];
  y = ((int)buf[3] << 8) | buf[2];
  pressed = true;

  // Clear status
  uint8_t z = 0;
  i2c_write_u16(GT911_ADDR, GT_REG_STATUS, &z, 1);
  return true;
}

/* =========================
   LVGL glue
   ========================= */
static lv_disp_draw_buf_t draw_buf;
static lv_color_t* buf1 = nullptr;

static void flush_cb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
  uint32_t w = (uint32_t)(area->x2 - area->x1 + 1);
  uint32_t h = (uint32_t)(area->y2 - area->y1 + 1);

  lcd.startWrite();
  lcd.setAddrWindow(area->x1, area->y1, w, h);
  lcd.writePixels((lgfx::rgb565_t*)color_p, w * h);
  lcd.endWrite();

  lv_disp_flush_ready(disp);
}

static void touch_cb(lv_indev_drv_t* indev, lv_indev_data_t* data) {
  (void)indev;
  if (!gt911_ok) { data->state = LV_INDEV_STATE_RELEASED; return; }

  int x, y; bool p;
  if (!gt911_read(x, y, p)) { data->state = LV_INDEV_STATE_RELEASED; return; }

  if (p) {
    // clamp
    if (x < 0) x = 0; if (x >= SCREEN_W) x = SCREEN_W - 1;
    if (y < 0) y = 0; if (y >= SCREEN_H) y = SCREEN_H - 1;
    data->point.x = x;
    data->point.y = y;
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

/* =========================
   App logic
   ========================= */
enum class State : uint8_t { IDLE, RUNNING, DONE, STOPPED, ERROR };

static Preferences prefs;

static volatile uint32_t isr_count = 0;
static volatile uint32_t isr_last_ms = 0;

static uint32_t g_count = 0;
static uint16_t g_target = 100;
static uint16_t g_debounce_ms = DEBOUNCE_MS_DEFAULT;
static State g_state = State::IDLE;

static void motor_set(bool on) {
  digitalWrite(GPIO_MOTOR, on ? MOTOR_ON_LEVEL : !MOTOR_ON_LEVEL);
}

static void IRAM_ATTR sensor_isr() {
  uint32_t now = (uint32_t)millis();
  uint32_t last = isr_last_ms;
  if ((uint32_t)(now - last) >= (uint32_t)g_debounce_ms) {
    isr_last_ms = now;
    isr_count++;
  }
}

/* =========================
   UI (German, Orange/White)
   ========================= */
static lv_obj_t* label_status;
static lv_obj_t* label_count;
static lv_obj_t* label_target;
static lv_obj_t* bar_progress;

static lv_obj_t* keypad_win = nullptr;
static lv_obj_t* keypad_value = nullptr;

static lv_style_t st_btn_orange, st_btn_white, st_btn_red, st_title, st_status, st_big, st_mid, st_box;

static lv_color_t C_ORANGE = lv_color_hex(0xFF7A00);
static lv_color_t C_LIGHT  = lv_color_hex(0xF4F5F7);
static lv_color_t C_WHITE  = lv_color_hex(0xFFFFFF);
static lv_color_t C_DARK   = lv_color_hex(0x1B1F24);

static const char* state_text(State s) {
  switch (s) {
    case State::IDLE:    return "Bereit";
    case State::RUNNING: return "Läuft…";
    case State::DONE:    return "Fertig: Bitte Band entnehmen";
    case State::STOPPED: return "Stopp";
    default:             return "Fehler";
  }
}

static void ui_update_numbers() {
  char b1[32], b2[32];
  snprintf(b1, sizeof(b1), "%lu", (unsigned long)g_count);
  snprintf(b2, sizeof(b2), "%u",  (unsigned)g_target);

  lv_label_set_text(label_count, b1);
  lv_label_set_text(label_target, b2);

  if (g_target > 0) {
    int p = (int)((g_count * 100UL) / (uint32_t)g_target);
    if (p < 0) p = 0; if (p > 100) p = 100;
    lv_bar_set_value(bar_progress, p, LV_ANIM_ON);
  } else {
    lv_bar_set_value(bar_progress, 0, LV_ANIM_OFF);
  }
}

static void ui_set_status(State s) {
  g_state = s;
  lv_label_set_text(label_status, state_text(s));
}

static void animate_press(lv_obj_t* obj) {
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, obj);
  lv_anim_set_values(&a, 100, 96);
  lv_anim_set_time(&a, 80);
  lv_anim_set_playback_time(&a, 80);
  lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_style_transform_zoom);
  lv_anim_start(&a);
}

static void start_run() {
  if (g_target < 1) g_target = 1;
  if (g_target > 1000) g_target = 1000;
  prefs.putUShort("target", g_target);
  motor_set(true);
  ui_set_status(State::RUNNING);
}

static void stop_run(State next) {
  motor_set(false);
  ui_set_status(next);
}

static void reset_all() {
  motor_set(false);
  isr_count = 0;
  g_count = 0;
  ui_set_status(State::IDLE);
  ui_update_numbers();
}

/* ===== Keypad ===== */
static void keypad_close() {
  if (keypad_win) {
    lv_obj_del(keypad_win);
    keypad_win = nullptr;
    keypad_value = nullptr;
  }
}

static void keypad_apply() {
  if (!keypad_value) return;
  const char* t = lv_label_get_text(keypad_value);
  int v = atoi(t);
  if (v < 1) v = 1;
  if (v > 1000) v = 1000;
  g_target = (uint16_t)v;
  prefs.putUShort("target", g_target);
  ui_update_numbers();
  keypad_close();
}

static void keypad_add_digit(char d) {
  if (!keypad_value) return;
  const char* t = lv_label_get_text(keypad_value);
  char buf[16];
  strncpy(buf, t, sizeof(buf));
  buf[sizeof(buf)-1] = 0;

  // avoid leading zeros
  if (strcmp(buf, "0") == 0) buf[0] = 0;

  size_t n = strlen(buf);
  if (n >= 4) return; // max 1000
  buf[n] = d;
  buf[n+1] = 0;
  lv_label_set_text(keypad_value, buf);
}

static void keypad_backspace() {
  if (!keypad_value) return;
  char buf[16];
  const char* t = lv_label_get_text(keypad_value);
  strncpy(buf, t, sizeof(buf));
  buf[sizeof(buf)-1] = 0;
  size_t n = strlen(buf);
  if (n == 0) return;
  buf[n-1] = 0;
  if (buf[0] == 0) strcpy(buf, "0");
  lv_label_set_text(keypad_value, buf);
}

static void keypad_btn_cb(lv_event_t* e) {
  lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
  animate_press(btn);

  const char* txt = lv_label_get_text(lv_obj_get_child(btn, 0));
  if (strcmp(txt, "OK") == 0) { keypad_apply(); return; }
  if (strcmp(txt, "X") == 0) { keypad_close(); return; }
  if (strcmp(txt, "<") == 0) { keypad_backspace(); return; }
  if (txt[0] >= '0' && txt[0] <= '9' && txt[1] == 0) keypad_add_digit(txt[0]);
}

static void open_keypad() {
  if (keypad_win) return;

  keypad_win = lv_obj_create(lv_scr_act());
  lv_obj_set_size(keypad_win, 520, 360);
  lv_obj_center(keypad_win);
  lv_obj_add_style(keypad_win, &st_box, 0);

  lv_obj_t* title = lv_label_create(keypad_win);
  lv_label_set_text(title, "Zielmenge eingeben (1..1000)");
  lv_obj_add_style(title, &st_mid, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);

  keypad_value = lv_label_create(keypad_win);
  lv_label_set_text(keypad_value, "0");
  lv_obj_add_style(keypad_value, &st_big, 0);
  lv_obj_align(keypad_value, LV_ALIGN_TOP_MID, 0, 52);

  // grid of buttons
  static const char* keys[4][4] = {
    {"1","2","3","<"},
    {"4","5","6","X"},
    {"7","8","9","OK"},
    {"0","","",""}
  };

  int x0 = 40, y0 = 150, bw = 100, bh = 55, gap = 16;

  for (int r=0;r<4;r++){
    for(int c=0;c<4;c++){
      const char* k = keys[r][c];
      if (!k[0]) continue;

      lv_obj_t* b = lv_btn_create(keypad_win);
      lv_obj_set_size(b, bw, bh);
      lv_obj_align(b, LV_ALIGN_TOP_LEFT, x0 + c*(bw+gap), y0 + r*(bh+gap), 0, 0);

      if (strcmp(k,"OK")==0) lv_obj_add_style(b, &st_btn_orange, 0);
      else if (strcmp(k,"X")==0) lv_obj_add_style(b, &st_btn_red, 0);
      else lv_obj_add_style(b, &st_btn_white, 0);

      lv_obj_add_event_cb(b, keypad_btn_cb, LV_EVENT_CLICKED, nullptr);

      lv_obj_t* t = lv_label_create(b);
      lv_label_set_text(t, k);
      lv_obj_center(t);
      lv_obj_set_style_text_color(t, C_DARK, 0);
      lv_obj_set_style_text_font(t, &lv_font_montserrat_24, 0);
    }
  }
}

/* ===== Main buttons ===== */
static void btn_start_cb(lv_event_t* e) {
  lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
  animate_press(btn);
  if (g_state != State::RUNNING) start_run();
}

static void btn_stop_cb(lv_event_t* e) {
  lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
  animate_press(btn);
  if (g_state == State::RUNNING) stop_run(State::STOPPED);
}

static void btn_reset_cb(lv_event_t* e) {
  lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
  animate_press(btn);
  reset_all();
}

static void target_box_cb(lv_event_t* e) {
  lv_obj_t* obj = (lv_obj_t*)lv_event_get_target(e);
  animate_press(obj);
  open_keypad();
}

static lv_obj_t* make_btn(lv_obj_t* parent, const char* txt, lv_style_t* st, lv_event_cb_t cb) {
  lv_obj_t* b = lv_btn_create(parent);
  lv_obj_add_style(b, st, 0);
  lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* t = lv_label_create(b);
  lv_label_set_text(t, txt);
  lv_obj_center(t);
  lv_obj_set_style_text_font(t, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(t, C_DARK, 0);
  return b;
}

static void init_styles() {
  lv_style_init(&st_box);
  lv_style_set_bg_color(&st_box, C_WHITE);
  lv_style_set_radius(&st_box, 18);
  lv_style_set_pad_all(&st_box, 14);
  lv_style_set_border_width(&st_box, 2);
  lv_style_set_border_color(&st_box, lv_color_hex(0xE1E5EA));
  lv_style_set_shadow_width(&st_box, 18);
  lv_style_set_shadow_opa(&st_box, LV_OPA_20);
  lv_style_set_shadow_ofs_y(&st_box, 6);

  lv_style_init(&st_btn_orange);
  lv_style_set_bg_color(&st_btn_orange, C_ORANGE);
  lv_style_set_radius(&st_btn_orange, 16);
  lv_style_set_shadow_width(&st_btn_orange, 16);
  lv_style_set_shadow_opa(&st_btn_orange, LV_OPA_20);
  lv_style_set_shadow_ofs_y(&st_btn_orange, 6);

  lv_style_init(&st_btn_white);
  lv_style_set_bg_color(&st_btn_white, C_WHITE);
  lv_style_set_radius(&st_btn_white, 16);
  lv_style_set_border_width(&st_btn_white, 2);
  lv_style_set_border_color(&st_btn_white, lv_color_hex(0xE1E5EA));
  lv_style_set_shadow_width(&st_btn_white, 12);
  lv_style_set_shadow_opa(&st_btn_white, LV_OPA_20);
  lv_style_set_shadow_ofs_y(&st_btn_white, 4);

  lv_style_init(&st_btn_red);
  lv_style_set_bg_color(&st_btn_red, lv_color_hex(0xFF4B4B));
  lv_style_set_radius(&st_btn_red, 16);
  lv_style_set_shadow_width(&st_btn_red, 16);
  lv_style_set_shadow_opa(&st_btn_red, LV_OPA_20);
  lv_style_set_shadow_ofs_y(&st_btn_red, 6);

  lv_style_init(&st_title);
  lv_style_set_text_font(&st_title, &lv_font_montserrat_28);
  lv_style_set_text_color(&st_title, C_DARK);

  lv_style_init(&st_status);
  lv_style_set_text_font(&st_status, &lv_font_montserrat_20);
  lv_style_set_text_color(&st_status, lv_color_hex(0x3A4450));

  lv_style_init(&st_big);
  lv_style_set_text_font(&st_big, &lv_font_montserrat_48);
  lv_style_set_text_color(&st_big, C_DARK);

  lv_style_init(&st_mid);
  lv_style_set_text_font(&st_mid, &lv_font_montserrat_24);
  lv_style_set_text_color(&st_mid, C_DARK);
}

static void build_ui() {
  lv_obj_t* scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, C_LIGHT, 0);

  lv_obj_t* card = lv_obj_create(scr);
  lv_obj_set_size(card, 760, 440);
  lv_obj_center(card);
  lv_obj_add_style(card, &st_box, 0);

  lv_obj_t* title = lv_label_create(card);
  lv_label_set_text(title, "Bandware Zähler");
  lv_obj_add_style(title, &st_title, 0);
  lv_obj_align(title, LV_ALIGN_TOP_LEFT, 18, 12);

  label_status = lv_label_create(card);
  lv_label_set_text(label_status, "Bereit");
  lv_obj_add_style(label_status, &st_status, 0);
  lv_obj_align(label_status, LV_ALIGN_TOP_RIGHT, -18, 18);

  // Count box
  lv_obj_t* box_count = lv_obj_create(card);
  lv_obj_set_size(box_count, 340, 160);
  lv_obj_align(box_count, LV_ALIGN_TOP_LEFT, 18, 60);
  lv_obj_add_style(box_count, &st_box, 0);

  lv_obj_t* l1 = lv_label_create(box_count);
  lv_label_set_text(l1, "IST");
  lv_obj_add_style(l1, &st_mid, 0);
  lv_obj_align(l1, LV_ALIGN_TOP_LEFT, 12, 8);

  label_count = lv_label_create(box_count);
  lv_label_set_text(label_count, "0");
  lv_obj_add_style(label_count, &st_big, 0);
  lv_obj_align(label_count, LV_ALIGN_CENTER, 0, 10);

  // Target box (clickable)
  lv_obj_t* box_target = lv_obj_create(card);
  lv_obj_set_size(box_target, 340, 160);
  lv_obj_align(box_target, LV_ALIGN_TOP_RIGHT, -18, 60);
  lv_obj_add_style(box_target, &st_box, 0);
  lv_obj_add_event_cb(box_target, target_box_cb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* l2 = lv_label_create(box_target);
  lv_label_set_text(l2, "ZIEL (tippen zum Ändern)");
  lv_obj_add_style(l2, &st_mid, 0);
  lv_obj_align(l2, LV_ALIGN_TOP_LEFT, 12, 8);

  label_target = lv_label_create(box_target);
  lv_label_set_text(label_target, "100");
  lv_obj_add_style(label_target, &st_big, 0);
  lv_obj_align(label_target, LV_ALIGN_CENTER, 0, 10);

  // Progress bar
  bar_progress = lv_bar_create(card);
  lv_obj_set_size(bar_progress, 724, 26);
  lv_obj_align(bar_progress, LV_ALIGN_TOP_MID, 0, 235);
  lv_bar_set_range(bar_progress, 0, 100);
  lv_bar_set_value(bar_progress, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(bar_progress, lv_color_hex(0xE9EDF2), 0);
  lv_obj_set_style_bg_color(bar_progress, C_ORANGE, LV_PART_INDICATOR);
  lv_obj_set_style_radius(bar_progress, 12, 0);
  lv_obj_set_style_radius(bar_progress, 12, LV_PART_INDICATOR);

  // Buttons row
  lv_obj_t* btn_start = make_btn(card, "START", &st_btn_orange, btn_start_cb);
  lv_obj_set_size(btn_start, 220, 70);
  lv_obj_align(btn_start, LV_ALIGN_BOTTOM_LEFT, 18, -18);

  lv_obj_t* btn_stop = make_btn(card, "STOP", &st_btn_red, btn_stop_cb);
  lv_obj_set_size(btn_stop, 220, 70);
  lv_obj_align(btn_stop, LV_ALIGN_BOTTOM_MID, 0, -18);

  lv_obj_t* btn_reset = make_btn(card, "RESET", &st_btn_white, btn_reset_cb);
  lv_obj_set_size(btn_reset, 220, 70);
  lv_obj_align(btn_reset, LV_ALIGN_BOTTOM_RIGHT, -18, -18);

  ui_update_numbers();
}

/* =========================
   Setup / Loop
   ========================= */
void setup() {
  Serial.begin(115200);
  delay(200);

  // Motor + sensor IO
  pinMode(GPIO_MOTOR, OUTPUT);
  motor_set(false);

  pinMode(GPIO_SENSOR, INPUT_PULLUP);

  prefs.begin("bandware", false);
  g_target = prefs.getUShort("target", 100);
  g_debounce_ms = prefs.getUShort("debounce", DEBOUNCE_MS_DEFAULT);
  if (g_debounce_ms < 1) g_debounce_ms = 1;
  if (g_debounce_ms > 50) g_debounce_ms = 50;

  // Display
  lcd.begin();

  // Touch
  gt911_ok = gt911_init();
  Serial.printf("[TOUCH] GT911 ok = %s\n", gt911_ok ? "true" : "false");

  // LVGL init
  lv_init();

  // draw buffer in PSRAM (recommended)
  size_t buf_px = (size_t)SCREEN_W * (size_t)LVGL_BUF_LINES;
  buf1 = (lv_color_t*)heap_caps_malloc(buf_px * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!buf1) {
    // fallback internal RAM
    buf1 = (lv_color_t*)malloc(buf_px * sizeof(lv_color_t));
  }
  lv_disp_draw_buf_init(&draw_buf, buf1, nullptr, buf_px);

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
  indev_drv.read_cb = touch_cb;
  lv_indev_drv_register(&indev_drv);

  init_styles();
  build_ui();
  ui_set_status(State::IDLE);

  // Interrupt for sensor
  attachInterrupt(digitalPinToInterrupt(GPIO_SENSOR), sensor_isr, FALLING);

  Serial.println("[APP] Ready.");
}

void loop() {
  // move ISR count to main safely
  static uint32_t last_isr_seen = 0;
  uint32_t now_isr = isr_count;
  if (now_isr != last_isr_seen) {
    uint32_t diff = now_isr - last_isr_seen;
    last_isr_seen = now_isr;

    if (g_state == State::RUNNING) {
      g_count += diff;
      ui_update_numbers();
      if (g_count >= g_target) {
        g_count = g_target;
        ui_update_numbers();
        stop_run(State::DONE);
      }
    }
  }

  lv_timer_handler();
  delay(5);
}
