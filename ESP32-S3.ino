#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>

#include <lvgl.h>
#include <LovyanGFX.hpp>

// =========================
//  CONFIG SECTION (CHANGE HERE)
// =========================

// --- Logic IO ---
static const int GPIO_SENSOR = 6;     // Input from PC817 OUT (open-collector) -> needs pull-up
static const int GPIO_MOTOR  = 7;     // Output to MOSFET/SSR driver (HIGH = ON)

// --- Backlight ---
static const int BL_PIN = 45;         // CHANGE if your board differs
static const bool BL_ACTIVE_HIGH = true;

// --- Touch (GT911 usually) ---
static const int TOUCH_I2C_SDA = 19;  // CHANGE if your board differs
static const int TOUCH_I2C_SCL = 20;  // CHANGE if your board differs
static const int TOUCH_INT_PIN = -1;  // optional
static const int TOUCH_RST_PIN = -1;  // optional
static const uint8_t GT911_ADDR = 0x5D; // sometimes 0x14

// --- Display resolution ---
static const int SCREEN_W = 800;
static const int SCREEN_H = 480;

// LVGL draw buffer lines (more lines = smoother, more RAM)
static const int LVGL_BUF_LINES = 40;

// --- Sensor debounce default (ms) ---
static uint16_t g_debounce_ms_default = 3;

// --- Default target ---
static uint16_t g_target_default = 100;

// =========================
//  DISPLAY (LovyanGFX RGB)  - MUST MATCH YOUR BOARD PINMAP
// =========================
// NOTE: Without correct RGB pinmap and timings, screen can stay black.
// Replace pins below with the pinmap of your specific ESP32-S3 HMI 7" board.
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_RGB _panel;
  lgfx::Bus_RGB   _bus;

public:
  LGFX() {
    {
      auto cfg = _bus.config();
      // -------- RGB DATA PINS D0..D15 (CHANGE!) --------
      cfg.pin_d0  = 8;
      cfg.pin_d1  = 9;
      cfg.pin_d2  = 10;
      cfg.pin_d3  = 11;
      cfg.pin_d4  = 12;
      cfg.pin_d5  = 13;
      cfg.pin_d6  = 14;
      cfg.pin_d7  = 15;
      cfg.pin_d8  = 16;
      cfg.pin_d9  = 17;
      cfg.pin_d10 = 18;
      cfg.pin_d11 = 21;
      cfg.pin_d12 = 47;
      cfg.pin_d13 = 48;
      cfg.pin_d14 = 38;
      cfg.pin_d15 = 39;

      // -------- Control pins (CHANGE!) --------
      cfg.pin_hsync = 40;
      cfg.pin_vsync = 41;
      cfg.pin_pclk  = 42;
      cfg.pin_de    = 46;

      // Pixel clock (Hz) - adjust if unstable/black
      cfg.freq_write = 16000000;

      // Polarity - adjust if needed
      cfg.hsync_polarity = 0;
      cfg.vsync_polarity = 0;
      cfg.pclk_idle_high = 0;
      cfg.de_idle_high   = 1;

      _bus.config(cfg);
      _panel.setBus(&_bus);
    }

    {
      auto cfg = _panel.config();
      cfg.panel_width  = SCREEN_W;
      cfg.panel_height = SCREEN_H;

      // Common 800x480 timing (adjust if needed)
      cfg.hsync_front_porch = 40;
      cfg.hsync_pulse_width = 48;
      cfg.hsync_back_porch  = 40;

      cfg.vsync_front_porch = 13;
      cfg.vsync_pulse_width = 3;
      cfg.vsync_back_porch  = 32;

      cfg.pclk_active_neg = 0;

      _panel.config(cfg);
    }

    setPanel(&_panel);
  }
};

static LGFX lcd;

// =========================
//  GT911 minimal driver (single touch)
// =========================
static bool gt911_read_touch(int16_t &x, int16_t &y, bool &pressed) {
  pressed = false;

  // GT911 status reg 0x814E, points start 0x8150
  Wire.beginTransmission(GT911_ADDR);
  Wire.write(0x81);
  Wire.write(0x4E);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((int)GT911_ADDR, 1) != 1) return false;
  uint8_t status = Wire.read();
  uint8_t n = status & 0x0F;

  if (n == 0) {
    // clear status
    Wire.beginTransmission(GT911_ADDR);
    Wire.write(0x81); Wire.write(0x4E); Wire.write(0x00);
    Wire.endTransmission();
    return true;
  }

  // read first point: 0x8150 .. (8 bytes)
  Wire.beginTransmission(GT911_ADDR);
  Wire.write(0x81);
  Wire.write(0x50);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t buf[8];
  if (Wire.requestFrom((int)GT911_ADDR, 8) != 8) return false;
  for (int i=0;i<8;i++) buf[i] = Wire.read();

  // buf: [id, xL, xH, yL, yH, ...]
  x = (int16_t)(buf[1] | (buf[2] << 8));
  y = (int16_t)(buf[3] | (buf[4] << 8));
  pressed = true;

  // clear status
  Wire.beginTransmission(GT911_ADDR);
  Wire.write(0x81); Wire.write(0x4E); Wire.write(0x00);
  Wire.endTransmission();

  // Some panels are rotated; adjust here if needed:
  // e.g. swap x/y or invert.
  // x = SCREEN_W - x;

  return true;
}

// =========================
//  LVGL glue
// =========================
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1 = nullptr;

static void lv_flush_cb(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  lcd.startWrite();
  lcd.setAddrWindow(area->x1, area->y1, w, h);
  lcd.writePixels((lgfx::rgb565_t*)color_p, w * h);
  lcd.endWrite();

  lv_disp_flush_ready(disp);
}

static void lv_touch_cb(lv_indev_drv_t *indev, lv_indev_data_t *data) {
  (void)indev;
  int16_t x, y; bool pressed;
  if (!gt911_read_touch(x, y, pressed)) {
    data->state = LV_INDEV_STATE_RELEASED;
    return;
  }
  if (pressed) {
    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = x;
    data->point.y = y;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

static void lv_tick_task(void *arg) {
  (void)arg;
  lv_tick_inc(1);
}

// =========================
//  App logic
// =========================
enum class State { IDLE, RUNNING, DONE, STOPPED, ERROR };

static volatile uint32_t g_count_isr = 0;
static volatile uint32_t g_last_isr_us = 0;

static uint16_t g_target = 0;
static uint16_t g_debounce_ms = 3;
static uint8_t  g_brightness = 80; // 0..100
static State    g_state = State::IDLE;

static Preferences prefs;

// UI objects
static lv_obj_t *label_status, *label_count, *label_target;
static lv_obj_t *bar_progress;
static lv_obj_t *ta_target;
static lv_obj_t *kb;
static lv_obj_t *slider_deb, *slider_bl;

static void motor_set(bool on) {
  digitalWrite(GPIO_MOTOR, on ? HIGH : LOW);
}

static const char* state_text(State s) {
  switch (s) {
    case State::IDLE:    return "Bereit";
    case State::RUNNING: return "Läuft…";
    case State::DONE:    return "Fertig: Bitte Band entnehmen";
    case State::STOPPED: return "Stopp";
    case State::ERROR:   return "Fehler";
  }
  return "";
}

static void ui_set_status(State s) {
  g_state = s;
  lv_label_set_text(label_status, state_text(s));
  // small fade animation
  lv_obj_set_style_opa(label_status, LV_OPA_0, 0);
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, label_status);
  lv_anim_set_values(&a, 0, 255);
  lv_anim_set_time(&a, 180);
  lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_style_opa);
  lv_anim_start(&a);
}

static void ui_update_numbers() {
  char buf[64];

  uint32_t c = g_count_isr;
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)c);
  lv_label_set_text(label_count, buf);

  snprintf(buf, sizeof(buf), "Ziel: %u", g_target);
  lv_label_set_text(label_target, buf);

  if (g_target > 0) {
    uint32_t p = (c >= g_target) ? 100 : (uint32_t)((c * 100UL) / g_target);
    lv_bar_set_value(bar_progress, (int)p, LV_ANIM_ON);
  } else {
    lv_bar_set_value(bar_progress, 0, LV_ANIM_OFF);
  }
}

static void start_run() {
  if (g_target < 1) g_target = 1;
  if (g_target > 1000) g_target = 1000;
  g_count_isr = 0;
  ui_update_numbers();
  ui_set_status(State::RUNNING);
  motor_set(true);
}

static void stop_run(State next) {
  motor_set(false);
  ui_set_status(next);
}

static void save_settings() {
  prefs.putUShort("target", g_target);
  prefs.putUShort("deb", g_debounce_ms);
  prefs.putUChar("bl", g_brightness);
}

static void apply_brightness(uint8_t pct) {
  if (pct > 100) pct = 100;
  g_brightness = pct;

  // LEDC PWM
  const int ch = 0;
  const int freq = 20000;
  const int res = 10; // 0..1023
  static bool inited = false;
  if (!inited) {
    ledcSetup(ch, freq, res);
    ledcAttachPin(BL_PIN, ch);
    inited = true;
  }
  uint32_t duty = (pct * 1023UL) / 100UL;
  if (!BL_ACTIVE_HIGH) duty = 1023 - duty;
  ledcWrite(ch, duty);
}

static void IRAM_ATTR sensor_isr() {
  uint32_t now = micros();
  uint32_t min_dt = (uint32_t)g_debounce_ms * 1000UL;
  if ((now - g_last_isr_us) < min_dt) return;
  g_last_isr_us = now;
  g_count_isr++;
}

static void animate_button_press(lv_obj_t *btn) {
  // scale down/up using transform (simple “press” feel)
  lv_obj_set_style_transform_zoom(btn, 240, 0);
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, btn);
  lv_anim_set_values(&a, 240, 256);
  lv_anim_set_time(&a, 120);
  lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_style_transform_zoom);
  lv_anim_start(&a);
}

static void btn_start_cb(lv_event_t *e) {
  lv_obj_t *btn = (lv_obj_t*)lv_event_get_target(e);
  animate_button_press(btn);
  if (g_state != State::RUNNING) start_run();
}

static void btn_stop_cb(lv_event_t *e) {
  lv_obj_t *btn = (lv_obj_t*)lv_event_get_target(e);
  animate_button_press(btn);
  if (g_state == State::RUNNING) stop_run(State::STOPPED);
}

static void btn_reset_cb(lv_event_t *e) {
  lv_obj_t *btn = (lv_obj_t*)lv_event_get_target(e);
  animate_button_press(btn);
  motor_set(false);
  g_count_isr = 0;
  ui_set_status(State::IDLE);
  ui_update_numbers();
}

static void ta_event_cb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t *ta = (lv_obj_t*)lv_event_get_target(e);

  if (code == LV_EVENT_FOCUSED) {
    lv_keyboard_set_textarea(kb, ta);
    lv_obj_clear_flag(kb, LV_OBJ_FLAG_HIDDEN);
  }
  if (code == LV_EVENT_DEFOCUSED || code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
    lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    // parse target
    const char *t = lv_textarea_get_text(ta);
    int v = atoi(t);
    if (v < 1) v = 1;
    if (v > 1000) v = 1000;
    g_target = (uint16_t)v;
    save_settings();
    ui_update_numbers();
  }
}

static void slider_deb_cb(lv_event_t *e) {
  lv_obj_t *sl = (lv_obj_t*)lv_event_get_target(e);
  g_debounce_ms = (uint16_t)lv_slider_get_value(sl);
  save_settings();
}

static void slider_bl_cb(lv_event_t *e) {
  lv_obj_t *sl = (lv_obj_t*)lv_event_get_target(e);
  uint8_t v = (uint8_t)lv_slider_get_value(sl);
  apply_brightness(v);
  save_settings();
}

static void build_ui() {
  lv_obj_t *scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_hex(0xF6F6F6), 0);

  // Title
  lv_obj_t *title = lv_label_create(scr);
  lv_label_set_text(title, "Bandware Zähler");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(title, lv_color_hex(0x222222), 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);

  // Status bar
  label_status = lv_label_create(scr);
  lv_label_set_text(label_status, "Bereit");
  lv_obj_set_style_text_font(label_status, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(label_status, lv_color_hex(0xFF7A00), 0);
  lv_obj_align(label_status, LV_ALIGN_TOP_MID, 0, 48);

  // Count big
  label_count = lv_label_create(scr);
  lv_label_set_text(label_count, "0");
  lv_obj_set_style_text_font(label_count, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(label_count, lv_color_hex(0x111111), 0);
  lv_obj_align(label_count, LV_ALIGN_CENTER, 0, -40);

  // Target label
  label_target = lv_label_create(scr);
  lv_label_set_text(label_target, "Ziel: 100");
  lv_obj_set_style_text_font(label_target, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(label_target, lv_color_hex(0x333333), 0);
  lv_obj_align(label_target, LV_ALIGN_CENTER, 0, 10);

  // Progress bar
  bar_progress = lv_bar_create(scr);
  lv_obj_set_size(bar_progress, 560, 18);
  lv_obj_align(bar_progress, LV_ALIGN_CENTER, 0, 44);
  lv_obj_set_style_bg_color(bar_progress, lv_color_hex(0xDDDDDD), 0);
  lv_obj_set_style_bg_color(bar_progress, lv_color_hex(0xFF7A00), LV_PART_INDICATOR);
  lv_bar_set_range(bar_progress, 0, 100);
  lv_bar_set_value(bar_progress, 0, LV_ANIM_OFF);

  // Target input
  lv_obj_t *lbl_in = lv_label_create(scr);
  lv_label_set_text(lbl_in, "Zielmenge (1..1000):");
  lv_obj_align(lbl_in, LV_ALIGN_LEFT_MID, 40, 90);

  ta_target = lv_textarea_create(scr);
  lv_obj_set_size(ta_target, 160, 40);
  lv_textarea_set_one_line(ta_target, true);
  lv_textarea_set_accepted_chars(ta_target, "0123456789");
  lv_textarea_set_max_length(ta_target, 4);
  lv_obj_align(ta_target, LV_ALIGN_LEFT_MID, 260, 90);
  lv_obj_add_event_cb(ta_target, ta_event_cb, LV_EVENT_ALL, nullptr);

  // Keyboard (numeric)
  kb = lv_keyboard_create(scr);
  lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_NUMBER);
  lv_obj_set_height(kb, 200);
  lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);

  // Buttons row
  lv_obj_t *btn_start = lv_btn_create(scr);
  lv_obj_set_size(btn_start, 180, 60);
  lv_obj_align(btn_start, LV_ALIGN_BOTTOM_LEFT, 40, -20);
  lv_obj_set_style_bg_color(btn_start, lv_color_hex(0xFF7A00), 0);
  lv_obj_set_style_radius(btn_start, 14, 0);
  lv_obj_add_event_cb(btn_start, btn_start_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t *t1 = lv_label_create(btn_start);
  lv_label_set_text(t1, "START");
  lv_obj_center(t1);
  lv_obj_set_style_text_color(t1, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_font(t1, &lv_font_montserrat_20, 0);

  lv_obj_t *btn_stop = lv_btn_create(scr);
  lv_obj_set_size(btn_stop, 180, 60);
  lv_obj_align(btn_stop, LV_ALIGN_BOTTOM_MID, 0, -20);
  lv_obj_set_style_bg_color(btn_stop, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_border_color(btn_stop, lv_color_hex(0xD80000), 0);
  lv_obj_set_style_border_width(btn_stop, 2, 0);
  lv_obj_set_style_radius(btn_stop, 14, 0);
  lv_obj_add_event_cb(btn_stop, btn_stop_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t *t2 = lv_label_create(btn_stop);
  lv_label_set_text(t2, "STOP");
  lv_obj_center(t2);
  lv_obj_set_style_text_color(t2, lv_color_hex(0xD80000), 0);
  lv_obj_set_style_text_font(t2, &lv_font_montserrat_20, 0);

  lv_obj_t *btn_reset = lv_btn_create(scr);
  lv_obj_set_size(btn_reset, 180, 60);
  lv_obj_align(btn_reset, LV_ALIGN_BOTTOM_RIGHT, -40, -20);
  lv_obj_set_style_bg_color(btn_reset, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_border_color(btn_reset, lv_color_hex(0xBBBBBB), 0);
  lv_obj_set_style_border_width(btn_reset, 1, 0);
  lv_obj_set_style_radius(btn_reset, 14, 0);
  lv_obj_add_event_cb(btn_reset, btn_reset_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t *t3 = lv_label_create(btn_reset);
  lv_label_set_text(t3, "RESET");
  lv_obj_center(t3);
  lv_obj_set_style_text_color(t3, lv_color_hex(0x333333), 0);
  lv_obj_set_style_text_font(t3, &lv_font_montserrat_20, 0);

  // Settings (bottom small area)
  lv_obj_t *lbl_set = lv_label_create(scr);
  lv_label_set_text(lbl_set, "Einstellungen");
  lv_obj_align(lbl_set, LV_ALIGN_RIGHT_MID, -220, 90);

  lv_obj_t *lbl_deb = lv_label_create(scr);
  lv_label_set_text(lbl_deb, "Entprellung (ms)");
  lv_obj_align(lbl_deb, LV_ALIGN_RIGHT_MID, -220, 120);

  slider_deb = lv_slider_create(scr);
  lv_obj_set_size(slider_deb, 200, 16);
  lv_slider_set_range(slider_deb, 1, 10);
  lv_slider_set_value(slider_deb, g_debounce_ms, LV_ANIM_OFF);
  lv_obj_align(slider_deb, LV_ALIGN_RIGHT_MID, -40, 120);
  lv_obj_set_style_bg_color(slider_deb, lv_color_hex(0xDDDDDD), 0);
  lv_obj_set_style_bg_color(slider_deb, lv_color_hex(0xFF7A00), LV_PART_INDICATOR);
  lv_obj_add_event_cb(slider_deb, slider_deb_cb, LV_EVENT_VALUE_CHANGED, nullptr);

  lv_obj_t *lbl_bl = lv_label_create(scr);
  lv_label_set_text(lbl_bl, "Helligkeit (%)");
  lv_obj_align(lbl_bl, LV_ALIGN_RIGHT_MID, -220, 150);

  slider_bl = lv_slider_create(scr);
  lv_obj_set_size(slider_bl, 200, 16);
  lv_slider_set_range(slider_bl, 5, 100);
  lv_slider_set_value(slider_bl, g_brightness, LV_ANIM_OFF);
  lv_obj_align(slider_bl, LV_ALIGN_RIGHT_MID, -40, 150);
  lv_obj_set_style_bg_color(slider_bl, lv_color_hex(0xDDDDDD), 0);
  lv_obj_set_style_bg_color(slider_bl, lv_color_hex(0xFF7A00), LV_PART_INDICATOR);
  lv_obj_add_event_cb(slider_bl, slider_bl_cb, LV_EVENT_VALUE_CHANGED, nullptr);
}

// =========================
//  Setup / Loop
// =========================
void setup() {
  Serial.begin(115200);
  delay(200);

  // IO
  pinMode(GPIO_MOTOR, OUTPUT);
  motor_set(false);

  pinMode(GPIO_SENSOR, INPUT_PULLUP); // needed for PC817 open-collector output
  attachInterrupt(digitalPinToInterrupt(GPIO_SENSOR), sensor_isr, FALLING);

  // Preferences
  prefs.begin("band", false);
  g_target = prefs.getUShort("target", g_target_default);
  g_debounce_ms = prefs.getUShort("deb", g_debounce_ms_default);
  g_brightness = prefs.getUChar("bl", 80);

  // I2C for touch
  Wire.begin(TOUCH_I2C_SDA, TOUCH_I2C_SCL, 400000);

  // LCD init
  lcd.init();
  lcd.setRotation(0);

  // Backlight
  pinMode(BL_PIN, OUTPUT);
  apply_brightness(g_brightness);

  // LVGL init
  lv_init();

  // LVGL tick (1ms)
  const esp_timer_create_args_t tcfg = {
    .callback = &lv_tick_task,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "lv_tick"
  };
  esp_timer_handle_t tick_timer;
  esp_timer_create(&tcfg, &tick_timer);
  esp_timer_start_periodic(tick_timer, 1000);

  // Draw buffer
  size_t buf_pixels = (size_t)SCREEN_W * (size_t)LVGL_BUF_LINES;
  buf1 = (lv_color_t*)heap_caps_malloc(buf_pixels * sizeof(lv_color_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!buf1) {
    Serial.println("ERROR: LVGL buffer alloc failed");
    ui_set_status(State::ERROR);
  }

  lv_disp_draw_buf_init(&draw_buf, buf1, nullptr, buf_pixels);

  // Display driver
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCREEN_W;
  disp_drv.ver_res = SCREEN_H;
  disp_drv.flush_cb = lv_flush_cb;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  // Touch input
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = lv_touch_cb;
  lv_indev_drv_register(&indev_drv);

  // UI
  build_ui();

  // Set initial text
  char tmp[16];
  snprintf(tmp, sizeof(tmp), "%u", g_target);
  lv_textarea_set_text(ta_target, tmp);
  ui_set_status(State::IDLE);
  ui_update_numbers();

  Serial.println("Boot OK.");
}

void loop() {
  lv_timer_handler();
  delay(5);

  // Logic updates
  static uint32_t last_ui = 0;
  uint32_t now = millis();
  if (now - last_ui >= 50) {
    last_ui = now;
    ui_update_numbers();

    if (g_state == State::RUNNING) {
      uint32_t c = g_count_isr;
      if (c >= g_target && g_target > 0) {
        stop_run(State::DONE);
      }
    }
  }
}
