#define LGFX_USE_V1
#include <Arduino.h>
#include <lvgl.h>
#include <Preferences.h>

#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <Wire.h>

/* =========================
   CONFIG (easy to change)
   ========================= */
static const int SCREEN_W = 800;
static const int SCREEN_H = 480;

/* Machine State Enum (defined early) */
enum class State : uint8_t { IDLE, RUNNING, DONE, STOPPED, ERROR };

/* Motor & Sensor GPIO */
static const int GPIO_MOTOR  = 2;    // HIGH = Motor ON  (change if needed)
static const int GPIO_SENSOR = 1;    // input from PC817 OUT (change if needed)

/* Touch (GT911 typical) */
static const bool USE_GT911 = true;
static const int TOUCH_I2C_SDA = 19;     // change if needed
static const int TOUCH_I2C_SCL = 20;     // change if needed
static const int TOUCH_RST_PIN = -1;     // -1 if not connected
static const int TOUCH_INT_PIN = -1;     // -1 if not connected
static const uint8_t GT911_ADDR = 0x5D;  // common 0x5D or 0x14

/* Debounce defaults */
static const uint16_t DEBOUNCE_MS_DEFAULT = 3; // 2..10ms typical

/* LVGL draw buffer lines */
static const int LVGL_BUF_LINES = 20;

/* =========================
   Global Variables
   ========================= */
static volatile uint32_t g_count_isr = 0;
static volatile uint32_t g_last_isr_us = 0;
static uint32_t g_count = 0;
static uint16_t g_target = 100;
static uint16_t g_debounce_ms = DEBOUNCE_MS_DEFAULT;
static State g_state = State::IDLE;
static Preferences prefs;

/* =========================
   LovyanGFX RGB
   ========================= */
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[SCREEN_W * LVGL_BUF_LINES];

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

/* =========================
   Touch (GT911 minimal)
   ========================= */
struct TouchPoint {
  bool pressed = false;
  uint16_t x = 0;
  uint16_t y = 0;
};

static TouchPoint g_touch;

static bool i2c_read(uint8_t addr, uint16_t reg, uint8_t* data, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg & 0xFF);
  Wire.write((reg >> 8) & 0xFF);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)len) != (int)len) return false;
  for (size_t i = 0; i < len; i++) data[i] = Wire.read();
  return true;
}

static bool i2c_write(uint8_t addr, uint16_t reg, const uint8_t* data, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg & 0xFF);
  Wire.write((reg >> 8) & 0xFF);
  for (size_t i = 0; i < len; i++) Wire.write(data[i]);
  return Wire.endTransmission() == 0;
}

static bool gt911_read_point(TouchPoint &tp) {
  uint8_t status = 0;
  if (!i2c_read(GT911_ADDR, 0x814E, &status, 1)) return false;

  uint8_t n = status & 0x0F;
  if ((status & 0x80) == 0 || n == 0) {
    tp.pressed = false;
    uint8_t z = 0;
    i2c_write(GT911_ADDR, 0x814E, &z, 1);
    return true;
  }

  uint8_t buf[8] = {0};
  if (!i2c_read(GT911_ADDR, 0x8150, buf, 8)) return false;

  uint16_t x = (uint16_t)buf[1] << 8 | buf[0];
  uint16_t y = (uint16_t)buf[3] << 8 | buf[2];

  uint8_t z = 0;
  i2c_write(GT911_ADDR, 0x814E, &z, 1);

  if (x >= SCREEN_W) x = SCREEN_W - 1;
  if (y >= SCREEN_H) y = SCREEN_H - 1;

  tp.pressed = true;
  tp.x = x;
  tp.y = y;
  return true;
}

/* =========================
   LVGL callbacks
   ========================= */
static void flush_cb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;

  lcd.startWrite();
  lcd.setAddrWindow(area->x1, area->y1, w, h);
  lcd.writePixels((lgfx::rgb565_t*)color_p, w * h);
  lcd.endWrite();

  lv_disp_flush_ready(disp);
}

static void touch_read_cb(lv_indev_drv_t* indev, lv_indev_data_t* data) {
  (void)indev;
  if (USE_GT911) {
    TouchPoint tp;
    if (gt911_read_point(tp)) g_touch = tp;
  }
  data->state = g_touch.pressed ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
  data->point.x = g_touch.x;
  data->point.y = g_touch.y;
}

/* =========================
   Machine logic functions
   ========================= */
static void motor_set(bool on) {
  digitalWrite(GPIO_MOTOR, on ? HIGH : LOW);
}

static void IRAM_ATTR sensor_isr() {
  uint32_t now = (uint32_t)micros();
  uint32_t min_us = (uint32_t)g_debounce_ms * 1000UL;
  if (now - g_last_isr_us < min_us) return;
  g_last_isr_us = now;
  g_count_isr++;
}

/* =========================
   UI Variables
   ========================= */
static lv_obj_t* label_status;
static lv_obj_t* label_count;
static lv_obj_t* label_target;
static lv_obj_t* bar_progress;
static lv_obj_t* btn_start;
static lv_obj_t* btn_stop;
static lv_obj_t* btn_reset;
static lv_obj_t* btn_target;
static lv_obj_t* panel_settings;
static lv_obj_t* slider_deb;

static lv_style_t st_bg, st_title, st_status, st_big, st_mid, st_btn_orange, st_btn_white, st_btn_stop;

static lv_color_t ORANGE = lv_color_hex(0xFF7A00);

/* =========================
   UI Helper Functions
   ========================= */
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

static void ui_set_status(State s) {
  g_state = s;
  lv_label_set_text(label_status, state_text(s));

  lv_obj_set_style_opa(label_status, LV_OPA_0, 0);
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, label_status);
  lv_anim_set_values(&a, 0, 255);
  lv_anim_set_time(&a, 220);
  lv_anim_set_exec_cb(&a, [](void* obj, int32_t v){
    lv_obj_set_style_opa((lv_obj_t*)obj, (lv_opa_t)v, 0);
  });
  lv_anim_start(&a);
}

static void ui_update_numbers() {
  static char buf[64];

  snprintf(buf, sizeof(buf), "%lu", (unsigned long)g_count);
  lv_label_set_text(label_count, buf);

  snprintf(buf, sizeof(buf), "Ziel: %u", (unsigned)g_target);
  lv_label_set_text(label_target, buf);

  int32_t pct = 0;
  if (g_target > 0) pct = (int32_t)((g_count * 100UL) / g_target);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;

  lv_bar_set_value(bar_progress, pct, LV_ANIM_ON);
}

static void animate_button(lv_obj_t* btn) {
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, btn);
  lv_anim_set_values(&a, 12, 6);
  lv_anim_set_time(&a, 90);
  lv_anim_set_playback_time(&a, 90);
  lv_anim_set_exec_cb(&a, [](void* obj, int32_t v){
    lv_obj_set_style_pad_all((lv_obj_t*)obj, v, 0);
  });
  lv_anim_start(&a);
}

/* =========================
   Keypad Functions
   ========================= */
static lv_obj_t* kb_win = nullptr;
static lv_obj_t* kb_label = nullptr;

static const char* kb_map[] = {
  "1","2","3","\n",
  "4","5","6","\n",
  "7","8","9","\n",
  "C","0","OK",""
};

static void kb_close() {
  if (kb_win) {
    lv_obj_del(kb_win);
    kb_win = nullptr;
    kb_label = nullptr;
  }
}

static void kb_event_cb(lv_event_t* e) {
  lv_obj_t* m = lv_event_get_target(e);
  uint32_t id = lv_btnmatrix_get_selected_btn(m);
  const char* txt = lv_btnmatrix_get_btn_text(m, id);
  if (!txt) return;

  static char inbuf[8] = "100";

  if (strcmp(txt, "C") == 0) {
    strcpy(inbuf, "");
  } else if (strcmp(txt, "OK") == 0) {
    int v = atoi(inbuf);
    if (v < 1) v = 1;
    if (v > 1000) v = 1000;
    g_target = (uint16_t)v;
    prefs.putUShort("target", g_target);
    ui_update_numbers();
    kb_close();
    return;
  } else {
    if (strlen(inbuf) < 4) {
      size_t L = strlen(inbuf);
      inbuf[L] = txt[0];
      inbuf[L+1] = 0;
    }
  }
  if (kb_label) lv_label_set_text(kb_label, inbuf);
}

static void open_keypad() {
  if (kb_win) return;

  kb_win = lv_obj_create(lv_scr_act());
  lv_obj_set_size(kb_win, 420, 360);
  lv_obj_center(kb_win);
  lv_obj_set_style_radius(kb_win, 16, 0);
  lv_obj_set_style_pad_all(kb_win, 14, 0);

  lv_obj_t* title = lv_label_create(kb_win);
  lv_label_set_text(title, "Zielmenge eingeben (1..1000)");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);

  kb_label = lv_label_create(kb_win);
  lv_label_set_text(kb_label, "100");
  lv_obj_set_style_text_font(kb_label, &lv_font_montserrat_36, 0);
  lv_obj_align(kb_label, LV_ALIGN_TOP_MID, 0, 34);

  lv_obj_t* m = lv_btnmatrix_create(kb_win);
  lv_btnmatrix_set_map(m, kb_map);
  lv_obj_set_size(m, 380, 240);
  lv_obj_align(m, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_add_event_cb(m, kb_event_cb, LV_EVENT_VALUE_CHANGED, nullptr);
}

/* =========================
   Control Functions
   ========================= */
static void start_run() {
  g_count = 0;
  g_count_isr = 0;
  motor_set(true);
  ui_set_status(State::RUNNING);
  ui_update_numbers();
}

static void stop_run(State next) {
  motor_set(false);
  ui_set_status(next);
}

static void btn_start_cb(lv_event_t* e) {
  (void)e;
  animate_button(btn_start);
  if (g_state != State::RUNNING) start_run();
}

static void btn_stop_cb(lv_event_t* e) {
  (void)e;
  animate_button(btn_stop);
  if (g_state == State::RUNNING) stop_run(State::STOPPED);
}

static void btn_reset_cb(lv_event_t* e) {
  (void)e;
  animate_button(btn_reset);
  motor_set(false);
  g_count = 0;
  g_count_isr = 0;
  ui_set_status(State::IDLE);
  ui_update_numbers();
}

static void btn_target_cb(lv_event_t* e) {
  (void)e;
  animate_button(btn_target);
  open_keypad();
}

static void slider_event_cb(lv_event_t* e) {
  lv_obj_t* s = lv_event_get_target(e);
  int v = lv_slider_get_value(s);
  if (v < 2) v = 2;
  if (v > 10) v = 10;
  g_debounce_ms = (uint16_t)v;
  prefs.putUShort("deb", g_debounce_ms);
}

/* =========================
   UI Creation
   ========================= */
static void init_styles() {
  lv_style_init(&st_bg);
  lv_style_set_bg_color(&st_bg, lv_color_hex(0xF6F6F6));
  lv_style_set_bg_opa(&st_bg, LV_OPA_COVER);

  lv_style_init(&st_title);
  lv_style_set_text_font(&st_title, &lv_font_montserrat_28);
  lv_style_set_text_color(&st_title, lv_color_hex(0x222222));

  lv_style_init(&st_status);
  lv_style_set_text_font(&st_status, &lv_font_montserrat_18);
  lv_style_set_text_color(&st_status, lv_color_hex(0x444444));

  lv_style_init(&st_big);
  lv_style_set_text_font(&st_big, &lv_font_montserrat_48);
  lv_style_set_text_color(&st_big, lv_color_hex(0x111111));

  lv_style_init(&st_mid);
  lv_style_set_text_font(&st_mid, &lv_font_montserrat_22);
  lv_style_set_text_color(&st_mid, lv_color_hex(0x333333));

  lv_style_init(&st_btn_orange);
  lv_style_set_bg_color(&st_btn_orange, ORANGE);
  lv_style_set_bg_opa(&st_btn_orange, LV_OPA_COVER);
  lv_style_set_radius(&st_btn_orange, 18);
  lv_style_set_shadow_width(&st_btn_orange, 20);
  lv_style_set_shadow_opa(&st_btn_orange, LV_OPA_20);

  lv_style_init(&st_btn_white);
  lv_style_set_bg_color(&st_btn_white, lv_color_hex(0xFFFFFF));
  lv_style_set_bg_opa(&st_btn_white, LV_OPA_COVER);
  lv_style_set_radius(&st_btn_white, 18);
  lv_style_set_border_width(&st_btn_white, 2);
  lv_style_set_border_color(&st_btn_white, lv_color_hex(0xDDDDDD));
  lv_style_set_shadow_width(&st_btn_white, 16);
  lv_style_set_shadow_opa(&st_btn_white, LV_OPA_10);

  lv_style_init(&st_btn_stop);
  lv_style_set_bg_color(&st_btn_stop, lv_color_hex(0xFFFFFF));
  lv_style_set_bg_opa(&st_btn_stop, LV_OPA_COVER);
  lv_style_set_radius(&st_btn_stop, 18);
  lv_style_set_border_width(&st_btn_stop, 3);
  lv_style_set_border_color(&st_btn_stop, lv_color_hex(0xD03030));
}

static lv_obj_t* make_btn(lv_obj_t* parent, const char* text, lv_style_t* st, lv_event_cb_t cb) {
  lv_obj_t* b = lv_btn_create(parent);
  lv_obj_add_style(b, st, 0);
  lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_set_style_pad_all(b, 12, 0);

  lv_obj_t* t = lv_label_create(b);
  lv_label_set_text(t, text);
  lv_obj_center(t);
  lv_obj_set_style_text_font(t, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(t, lv_color_hex(0x111111), 0);
  return b;
}

static void build_ui() {
  lv_obj_t* scr = lv_scr_act();
  lv_obj_add_style(scr, &st_bg, 0);

  // Title
  lv_obj_t* title = lv_label_create(scr);
  lv_label_set_text(title, "Bandware Zähler");
  lv_obj_add_style(title, &st_title, 0);
  lv_obj_align(title, LV_ALIGN_TOP_LEFT, 24, 16);

  // Status
  label_status = lv_label_create(scr);
  lv_label_set_text(label_status, "Bereit");
  lv_obj_add_style(label_status, &st_status, 0);
  lv_obj_align(label_status, LV_ALIGN_TOP_LEFT, 24, 56);

  // Count big
  label_count = lv_label_create(scr);
  lv_label_set_text(label_count, "0");
  lv_obj_add_style(label_count, &st_big, 0);
  lv_obj_align(label_count, LV_ALIGN_LEFT_MID, 40, -60);

  // Target label
  label_target = lv_label_create(scr);
  lv_label_set_text(label_target, "Ziel: 100");
  lv_obj_add_style(label_target, &st_mid, 0);
  lv_obj_align(label_target, LV_ALIGN_LEFT_MID, 44, 10);

  // Target button
  btn_target = make_btn(scr, "Zielmenge", &st_btn_white, btn_target_cb);
  lv_obj_set_size(btn_target, 210, 70);
  lv_obj_align(btn_target, LV_ALIGN_LEFT_MID, 40, 80);

  // Progress bar
  bar_progress = lv_bar_create(scr);
  lv_obj_set_size(bar_progress, 720, 18);
  lv_obj_align(bar_progress, LV_ALIGN_BOTTOM_MID, 0, -24);
  lv_bar_set_range(bar_progress, 0, 100);
  lv_bar_set_value(bar_progress, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(bar_progress, lv_color_hex(0xEAEAEA), 0);
  lv_obj_set_style_bg_color(bar_progress, ORANGE, LV_PART_INDICATOR);

  // Buttons right
  btn_start = make_btn(scr, "START", &st_btn_orange, btn_start_cb);
  lv_obj_set_size(btn_start, 240, 90);
  lv_obj_align(btn_start, LV_ALIGN_RIGHT_MID, -40, -80);

  btn_stop = make_btn(scr, "STOP", &st_btn_stop, btn_stop_cb);
  lv_obj_set_size(btn_stop, 240, 90);
  lv_obj_align(btn_stop, LV_ALIGN_RIGHT_MID, -40, 20);

  btn_reset = make_btn(scr, "RESET", &st_btn_white, btn_reset_cb);
  lv_obj_set_size(btn_reset, 240, 90);
  lv_obj_align(btn_reset, LV_ALIGN_RIGHT_MID, -40, 120);

  // Settings panel (debounce)
  panel_settings = lv_obj_create(scr);
  lv_obj_set_size(panel_settings, 300, 80);
  lv_obj_align(panel_settings, LV_ALIGN_TOP_RIGHT, -24, 14);
  lv_obj_set_style_radius(panel_settings, 16, 0);
  lv_obj_set_style_bg_color(panel_settings, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_shadow_width(panel_settings, 14, 0);
  lv_obj_set_style_shadow_opa(panel_settings, LV_OPA_10, 0);
  lv_obj_set_style_pad_all(panel_settings, 12, 0);

  lv_obj_t* s_title = lv_label_create(panel_settings);
  lv_label_set_text(s_title, "Entprellung (ms)");
  lv_obj_set_style_text_font(s_title, &lv_font_montserrat_18, 0);
  lv_obj_align(s_title, LV_ALIGN_TOP_LEFT, 0, 0);

  slider_deb = lv_slider_create(panel_settings);
  lv_obj_set_size(slider_deb, 260, 16);
  lv_obj_align(slider_deb, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_slider_set_range(slider_deb, 2, 10);
  lv_slider_set_value(slider_deb, g_debounce_ms, LV_ANIM_OFF);
  lv_obj_add_event_cb(slider_deb, slider_event_cb, LV_EVENT_VALUE_CHANGED, nullptr);
  lv_obj_set_style_bg_color(slider_deb, lv_color_hex(0xEAEAEA), 0);
  lv_obj_set_style_bg_color(slider_deb, ORANGE, LV_PART_INDICATOR);
}

/* =========================
   LVGL Initialization
   ========================= */
static void init_lvgl() {
  lv_init();

  lv_disp_draw_buf_init(&draw_buf, buf1, nullptr, SCREEN_W * LVGL_BUF_LINES);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCREEN_W;
  disp_drv.ver_res = SCREEN_H;
  disp_drv.flush_cb = flush_cb;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_t* disp = lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touch_read_cb;
  lv_indev_drv_register(&indev_drv);
}

/* =========================
   Setup / Loop
   ========================= */
void setup() {
  Serial.begin(115200);
  delay(200);

  // Load preferences
  prefs.begin("bandware", false);
  g_target = prefs.getUShort("target", 100);
  g_debounce_ms = prefs.getUShort("deb", DEBOUNCE_MS_DEFAULT);
  
  if (g_target < 1) g_target = 1;
  if (g_target > 1000) g_target = 1000;
  if (g_debounce_ms < 2) g_debounce_ms = 2;
  if (g_debounce_ms > 10) g_debounce_ms = 10;

  // GPIO Setup
  pinMode(GPIO_MOTOR, OUTPUT);
  motor_set(false);

  pinMode(GPIO_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GPIO_SENSOR), sensor_isr, FALLING);

  // LCD Initialize
  lcd.begin();

  // Touch I2C
  if (USE_GT911) {
    Wire.begin(TOUCH_I2C_SDA, TOUCH_I2C_SCL);
    Wire.setClock(400000);
  }

  // LVGL Initialize
  init_lvgl();
  init_styles();
  build_ui();

  ui_set_status(State::IDLE);
  ui_update_numbers();

  Serial.println("[OK] BandwareZaehler gestartet.");
}

void loop() {
  static uint32_t last_tick = millis();
  uint32_t now = millis();
  uint32_t diff = now - last_tick;
  last_tick = now;

  // Update LVGL tick
  lv_tick_inc(diff);
  
  // Read ISR count
  uint32_t c = g_count_isr;
  if (c != g_count) {
    g_count = c;
    ui_update_numbers();
    if (g_state == State::RUNNING && g_target > 0 && g_count >= g_target) {
      motor_set(false);
      ui_set_status(State::DONE);
    }
  }

  // Handle LVGL tasks
  lv_timer_handler();
  delay(5);
}
