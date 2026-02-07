#define LGFX_USE_V1
#include <Arduino.h>

#include <lvgl.h>
#if (LVGL_VERSION_MAJOR != 8)
  #error "This sketch requires LVGL v8.x. Remove LVGL v9 and install LVGL 8.x from Arduino Library Manager."
#endif

#include <Preferences.h>
#include <Wire.h>

#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>

// ======================
// 0) USER CONFIG
// ======================
static const int SCREEN_W = 800;
static const int SCREEN_H = 480;

// I/O
static const int GPIO_SENSOR = 10;     // Opto OUT -> ESP32 (INPUT_PULLUP) / interrupt
static const int GPIO_MOTOR  = 11;     // Motor driver input (HIGH = ON)

// Optional Backlight pin (اگر بردت BL جدا دارد مقدار بده، وگرنه -1 بگذار)
static const int GPIO_BL = -1;         // مثال: 2 یا 38 یا 45 ... (اگر نمی‌دانی -1)

// Debounce default (ms)
static const uint16_t DEFAULT_DEBOUNCE_MS = 4;

// Touch I2C (اگر بردت فرق دارد تغییر بده)
static const int TOUCH_I2C_SDA = 19;
static const int TOUCH_I2C_SCL = 20;
static const int TOUCH_INT_PIN = -1;
static const int TOUCH_RST_PIN = -1;

// LVGL buffer lines (20 خوبه)
static const int LVGL_BUF_LINES = 20;

// ======================
// 1) DISPLAY (LovyanGFX RGB) - YOUR WORKING CONFIG
// ======================
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[SCREEN_W * LVGL_BUF_LINES];

class LGFX : public lgfx::LGFX_Device {
  lgfx::Bus_RGB bus;
  lgfx::Panel_RGB panel;

public:
  LGFX() {
    { // bus
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

    { // panel
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

// ======================
// 2) APP STATE + LOGIC
// ======================
enum class AppState : uint8_t { IDLE, RUNNING, DONE, STOPPED, ERROR };

static volatile uint32_t g_count = 0;
static volatile uint32_t g_last_isr_ms = 0;
static volatile uint16_t g_debounce_ms_isr = DEFAULT_DEBOUNCE_MS;

static Preferences prefs;
static uint16_t target = 50;
static uint16_t debounce_ms = DEFAULT_DEBOUNCE_MS;
static AppState app_state = AppState::IDLE;

// ======================
// 3) UI objects
// ======================
static lv_obj_t* lbl_title  = nullptr;
static lv_obj_t* lbl_status = nullptr;
static lv_obj_t* lbl_count  = nullptr;
static lv_obj_t* lbl_target = nullptr;
static lv_obj_t* bar_prog   = nullptr;

static lv_obj_t* btn_start  = nullptr;
static lv_obj_t* btn_stop   = nullptr;
static lv_obj_t* btn_reset  = nullptr;
static lv_obj_t* btn_target = nullptr;
static lv_obj_t* btn_settings = nullptr;

static lv_obj_t* settings_panel = nullptr;
static lv_obj_t* slider_debounce = nullptr;
static lv_obj_t* lbl_deb_val = nullptr;

static lv_obj_t* keypad_win = nullptr;
static lv_obj_t* keypad_lbl = nullptr;
static String keypad_value;

// Styles
static lv_style_t st_bg, st_title, st_status, st_big, st_mid;
static lv_style_t st_btn_orange, st_btn_white, st_btn_red;

// ======================
// 4) Motor control
// ======================
static void motor_set(bool on) {
  digitalWrite(GPIO_MOTOR, on ? HIGH : LOW);
}

// ======================
// 5) Sensor ISR (FALLING, opto open-collector + pullup)
// ======================
void IRAM_ATTR sensor_isr() {
  uint32_t now = millis();
  uint16_t db = g_debounce_ms_isr;
  if ((uint32_t)(now - g_last_isr_ms) < (uint32_t)db) return;
  g_last_isr_ms = now;
  g_count++;
}

// ======================
// 6) LVGL flush
// ======================
static void flush_cb(lv_disp_drv_t* disp, const lv_area_t* area, lv_color_t* color_p) {
  uint32_t w = (uint32_t)(area->x2 - area->x1 + 1);
  uint32_t h = (uint32_t)(area->y2 - area->y1 + 1);

  lcd.startWrite();
  lcd.setAddrWindow(area->x1, area->y1, w, h);
  lcd.writePixels((lgfx::rgb565_t*)color_p, w * h);
  lcd.endWrite();

  lv_disp_flush_ready(disp);
}

// ======================
// 7) Touch GT911 (simple, robust)
// ======================
static bool touch_ok = false;
static uint8_t gt_addr = 0x00;
static int16_t touch_x = 0, touch_y = 0;
static bool touch_pressed = false;

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
  return Wire.endTransmission() == 0;
}

static bool gt911_try(uint8_t addr) {
  uint8_t id[4];
  if (!i2c_read(addr, 0x8140, id, 4)) return false;
  // بعضی بردها "911\0" یا مشابه برمی‌گردانند. همین که خوانده شد یعنی هست.
  uint8_t zero = 0;
  i2c_write(addr, 0x814E, &zero, 1);
  gt_addr = addr;
  return true;
}

static void touch_init() {
  Wire.begin(TOUCH_I2C_SDA, TOUCH_I2C_SCL, 400000);

  if (TOUCH_RST_PIN >= 0) {
    pinMode(TOUCH_RST_PIN, OUTPUT);
    digitalWrite(TOUCH_RST_PIN, LOW);
    delay(10);
    digitalWrite(TOUCH_RST_PIN, HIGH);
    delay(50);
  }
  if (TOUCH_INT_PIN >= 0) pinMode(TOUCH_INT_PIN, INPUT);

  if (gt911_try(0x5D)) { touch_ok = true; return; }
  if (gt911_try(0x14)) { touch_ok = true; return; }
  touch_ok = false;
}

static void touch_read_once() {
  if (!touch_ok) { touch_pressed = false; return; }

  uint8_t status = 0;
  if (!i2c_read(gt_addr, 0x814E, &status, 1)) { touch_pressed = false; return; }

  uint8_t points = status & 0x0F;
  if ((status & 0x80) == 0 || points == 0) {
    touch_pressed = false;
    return;
  }

  uint8_t buf[8];
  if (!i2c_read(gt_addr, 0x8150, buf, 8)) { touch_pressed = false; return; }

  uint16_t x = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
  uint16_t y = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);

  uint8_t zero = 0;
  i2c_write(gt_addr, 0x814E, &zero, 1);

  touch_x = (int16_t)x;
  touch_y = (int16_t)y;
  touch_pressed = true;
}

static void indev_read_cb(lv_indev_drv_t* indev, lv_indev_data_t* data) {
  (void)indev;
  touch_read_once();
  data->state = touch_pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
  data->point.x = touch_x;
  data->point.y = touch_y;
}

// ======================
// 8) UI helper
// ======================
static const char* state_text(AppState s) {
  switch (s) {
    case AppState::IDLE:    return "Bereit";
    case AppState::RUNNING: return "Läuft…";
    case AppState::DONE:    return "Fertig: Bitte Band entnehmen";
    case AppState::STOPPED: return "Stopp";
    case AppState::ERROR:   return "Fehler";
    default:                return "—";
  }
}

static void ui_set_status(AppState s) {
  app_state = s;
  if (lbl_status) lv_label_set_text(lbl_status, state_text(s));
}

static void ui_update_numbers() {
  if (!lbl_count || !lbl_target || !bar_prog) return;

  char cbuf[32];
  snprintf(cbuf, sizeof(cbuf), "%lu", (unsigned long)g_count);
  lv_label_set_text(lbl_count, cbuf);

  char tbuf[32];
  snprintf(tbuf, sizeof(tbuf), "Ziel: %u", (unsigned)target);
  lv_label_set_text(lbl_target, tbuf);

  uint16_t pct = 0;
  if (target > 0) {
    uint32_t c = g_count;
    pct = (c >= target) ? 100 : (uint16_t)((c * 100UL) / target);
  }
  lv_bar_set_value(bar_prog, pct, LV_ANIM_ON);
}

static void animate_press(lv_obj_t* obj) {
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, obj);
  lv_anim_set_time(&a, 70);
  lv_anim_set_values(&a, 100, 94);
  lv_anim_set_exec_cb(&a, [](void* v, int32_t val){
    lv_obj_set_style_transform_zoom((lv_obj_t*)v, val, 0);
  });
  lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
  lv_anim_start(&a);

  lv_anim_t b;
  lv_anim_init(&b);
  lv_anim_set_var(&b, obj);
  lv_anim_set_delay(&b, 70);
  lv_anim_set_time(&b, 110);
  lv_anim_set_values(&b, 94, 100);
  lv_anim_set_exec_cb(&b, [](void* v, int32_t val){
    lv_obj_set_style_transform_zoom((lv_obj_t*)v, val, 0);
  });
  lv_anim_set_path_cb(&b, lv_anim_path_ease_in_out);
  lv_anim_start(&b);
}

// ======================
// 9) Keypad (Target)
// ======================
static void keypad_close() {
  if (keypad_win) {
    lv_obj_del(keypad_win);
    keypad_win = nullptr;
  }
}

static void keypad_apply() {
  int v = keypad_value.toInt();
  if (v < 1) v = 1;
  if (v > 1000) v = 1000;
  target = (uint16_t)v;
  prefs.putUShort("target", target);
  ui_update_numbers();
  keypad_close();
}

static void keypad_btn_cb(lv_event_t* e) {
  lv_obj_t* m = lv_event_get_target(e);
  const char* txt = lv_btnmatrix_get_btn_text(m, lv_btnmatrix_get_selected_btn(m));
  if (!txt) return;

  if (strcmp(txt, "OK") == 0) { keypad_apply(); return; }
  if (strcmp(txt, "C") == 0)  { keypad_value = ""; lv_label_set_text(keypad_lbl, ""); return; }
  if (strcmp(txt, "<") == 0)  {
    if (keypad_value.length()) keypad_value.remove(keypad_value.length() - 1);
    lv_label_set_text(keypad_lbl, keypad_value.c_str());
    return;
  }

  if (keypad_value.length() < 4 && txt[0] >= '0' && txt[0] <= '9') {
    keypad_value += txt;
    lv_label_set_text(keypad_lbl, keypad_value.c_str());
  }
}

static void open_keypad() {
  if (keypad_win) return;

  keypad_value = String(target);

  keypad_win = lv_obj_create(lv_scr_act());
  lv_obj_set_size(keypad_win, 420, 340);
  lv_obj_center(keypad_win);
  lv_obj_set_style_bg_color(keypad_win, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_radius(keypad_win, 16, 0);
  lv_obj_set_style_shadow_width(keypad_win, 20, 0);
  lv_obj_set_style_shadow_opa(keypad_win, LV_OPA_20, 0);

  lv_obj_t* title = lv_label_create(keypad_win);
  lv_label_set_text(title, "Zielmenge eingeben");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);

  keypad_lbl = lv_label_create(keypad_win);
  lv_label_set_text(keypad_lbl, keypad_value.c_str());
  lv_obj_set_style_text_font(keypad_lbl, &lv_font_montserrat_36, 0);
  lv_obj_align(keypad_lbl, LV_ALIGN_TOP_MID, 0, 44);

  static const char* map[] = {
    "1","2","3","\n",
    "4","5","6","\n",
    "7","8","9","\n",
    "C","0","<","\n",
    "OK","", ""
  };

  lv_obj_t* bm = lv_btnmatrix_create(keypad_win);
  lv_btnmatrix_set_map(bm, map);
  lv_obj_set_size(bm, 380, 220);
  lv_obj_align(bm, LV_ALIGN_BOTTOM_MID, 0, -14);
  lv_obj_add_event_cb(bm, keypad_btn_cb, LV_EVENT_VALUE_CHANGED, nullptr);
  lv_obj_set_style_text_font(bm, &lv_font_montserrat_24, 0);
}

// ======================
// 10) Settings (Debounce)
// ======================
static void settings_close() {
  if (settings_panel) {
    lv_obj_del(settings_panel);
    settings_panel = nullptr;
  }
}

static void settings_toggle() {
  if (settings_panel) { settings_close(); return; }

  settings_panel = lv_obj_create(lv_scr_act());
  lv_obj_set_size(settings_panel, 360, 220);
  lv_obj_align(settings_panel, LV_ALIGN_TOP_RIGHT, -14, 70);
  lv_obj_set_style_radius(settings_panel, 16, 0);
  lv_obj_set_style_shadow_width(settings_panel, 20, 0);
  lv_obj_set_style_shadow_opa(settings_panel, LV_OPA_20, 0);

  lv_obj_t* t = lv_label_create(settings_panel);
  lv_label_set_text(t, "Einstellungen");
  lv_obj_set_style_text_font(t, &lv_font_montserrat_20, 0);
  lv_obj_align(t, LV_ALIGN_TOP_LEFT, 12, 10);

  lv_obj_t* l = lv_label_create(settings_panel);
  lv_label_set_text(l, "Entprellung (ms)");
  lv_obj_set_style_text_font(l, &lv_font_montserrat_16, 0);
  lv_obj_align(l, LV_ALIGN_TOP_LEFT, 12, 56);

  lbl_deb_val = lv_label_create(settings_panel);
  lv_label_set_text_fmt(lbl_deb_val, "%u", (unsigned)debounce_ms);
  lv_obj_set_style_text_font(lbl_deb_val, &lv_font_montserrat_16, 0);
  lv_obj_align(lbl_deb_val, LV_ALIGN_TOP_RIGHT, -12, 56);

  slider_debounce = lv_slider_create(settings_panel);
  lv_obj_set_width(slider_debounce, 320);
  lv_obj_align(slider_debounce, LV_ALIGN_TOP_LEFT, 12, 84);
  lv_slider_set_range(slider_debounce, 1, 20);
  lv_slider_set_value(slider_debounce, debounce_ms, LV_ANIM_OFF);

  lv_obj_add_event_cb(slider_debounce, [](lv_event_t* e){
    lv_obj_t* sld = lv_event_get_target(e);
    debounce_ms = (uint16_t)lv_slider_get_value(sld);
    g_debounce_ms_isr = debounce_ms;             // ISR-safe copy
    prefs.putUShort("deb", debounce_ms);
    if (lbl_deb_val) lv_label_set_text_fmt(lbl_deb_val, "%u", (unsigned)debounce_ms);
  }, LV_EVENT_VALUE_CHANGED, nullptr);

  lv_obj_t* hint = lv_label_create(settings_panel);
  lv_label_set_text(hint, "Wert wird automatisch gespeichert.");
  lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
  lv_obj_align(hint, LV_ALIGN_BOTTOM_LEFT, 12, -12);
}

// ======================
// 11) Button callbacks
// ======================
static void start_run() {
  g_count = 0;
  ui_update_numbers();
  motor_set(true);
  ui_set_status(AppState::RUNNING);
}

static void stop_run(AppState next) {
  motor_set(false);
  ui_set_status(next);
}

static void btn_start_cb(lv_event_t* e) {
  lv_obj_t* btn = lv_event_get_target(e);
  animate_press(btn);
  if (app_state != AppState::RUNNING) start_run();
}

static void btn_stop_cb(lv_event_t* e) {
  lv_obj_t* btn = lv_event_get_target(e);
  animate_press(btn);
  if (app_state == AppState::RUNNING) stop_run(AppState::STOPPED);
  else motor_set(false);
}

static void btn_reset_cb(lv_event_t* e) {
  lv_obj_t* btn = lv_event_get_target(e);
  animate_press(btn);
  motor_set(false);
  g_count = 0;
  ui_set_status(AppState::IDLE);
  ui_update_numbers();
}

static void btn_target_cb(lv_event_t* e) {
  lv_obj_t* btn = lv_event_get_target(e);
  animate_press(btn);
  open_keypad();
}

static void btn_settings_cb(lv_event_t* e) {
  lv_obj_t* btn = lv_event_get_target(e);
  animate_press(btn);
  settings_toggle();
}

// ======================
// 12) Styles + UI build
// ======================
static void init_styles() {
  // background
  lv_style_init(&st_bg);
  lv_style_set_bg_color(&st_bg, lv_color_hex(0xF4F6F8));
  lv_style_set_bg_opa(&st_bg, LV_OPA_COVER);

  // title
  lv_style_init(&st_title);
  lv_style_set_text_color(&st_title, lv_color_hex(0x111827));
  lv_style_set_text_font(&st_title, &lv_font_montserrat_28);

  // status
  lv_style_init(&st_status);
  lv_style_set_text_color(&st_status, lv_color_hex(0x374151));
  lv_style_set_text_font(&st_status, &lv_font_montserrat_18);

  // big count
  lv_style_init(&st_big);
  lv_style_set_text_color(&st_big, lv_color_hex(0x111827));
  lv_style_set_text_font(&st_big, &lv_font_montserrat_48);

  // mid
  lv_style_init(&st_mid);
  lv_style_set_text_color(&st_mid, lv_color_hex(0x111827));
  lv_style_set_text_font(&st_mid, &lv_font_montserrat_22);

  // buttons
  lv_style_init(&st_btn_orange);
  lv_style_set_bg_color(&st_btn_orange, lv_color_hex(0xFF7A00));
  lv_style_set_bg_opa(&st_btn_orange, LV_OPA_COVER);
  lv_style_set_radius(&st_btn_orange, 16);
  lv_style_set_shadow_width(&st_btn_orange, 18);
  lv_style_set_shadow_opa(&st_btn_orange, LV_OPA_20);
  lv_style_set_shadow_spread(&st_btn_orange, 2);
  lv_style_set_text_color(&st_btn_orange, lv_color_hex(0xFFFFFF));

  lv_style_init(&st_btn_white);
  lv_style_set_bg_color(&st_btn_white, lv_color_hex(0xFFFFFF));
  lv_style_set_bg_opa(&st_btn_white, LV_OPA_COVER);
  lv_style_set_radius(&st_btn_white, 16);
  lv_style_set_shadow_width(&st_btn_white, 14);
  lv_style_set_shadow_opa(&st_btn_white, LV_OPA_20);
  lv_style_set_text_color(&st_btn_white, lv_color_hex(0x111827));

  lv_style_init(&st_btn_red);
  lv_style_set_bg_color(&st_btn_red, lv_color_hex(0xE11D48));
  lv_style_set_bg_opa(&st_btn_red, LV_OPA_COVER);
  lv_style_set_radius(&st_btn_red, 16);
  lv_style_set_shadow_width(&st_btn_red, 18);
  lv_style_set_shadow_opa(&st_btn_red, LV_OPA_20);
  lv_style_set_text_color(&st_btn_red, lv_color_hex(0xFFFFFF));
}

static lv_obj_t* make_btn(lv_obj_t* parent, const char* text, lv_style_t* st, lv_event_cb_t cb) {
  lv_obj_t* b = lv_btn_create(parent);
  lv_obj_add_style(b, st, 0);
  lv_obj_set_style_transform_zoom(b, 100, 0);
  lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* t = lv_label_create(b);
  lv_label_set_text(t, text);
  lv_obj_center(t);
  lv_obj_set_style_text_font(t, &lv_font_montserrat_24, 0);
  return b;
}

static void build_ui() {
  lv_obj_t* scr = lv_scr_act();
  lv_obj_add_style(scr, &st_bg, 0);

  lbl_title = lv_label_create(scr);
  lv_label_set_text(lbl_title, "Bandware Zähler");
  lv_obj_add_style(lbl_title, &st_title, 0);
  lv_obj_align(lbl_title, LV_ALIGN_TOP_LEFT, 20, 14);

  btn_settings = make_btn(scr, "⚙", &st_btn_white, btn_settings_cb);
  lv_obj_set_size(btn_settings, 70, 56);
  lv_obj_align(btn_settings, LV_ALIGN_TOP_RIGHT, -20, 10);

  btn_target = make_btn(scr, "ZIEL", &st_btn_white, btn_target_cb);
  lv_obj_set_size(btn_target, 120, 56);
  lv_obj_align(btn_target, LV_ALIGN_TOP_LEFT, 260, 10);

  lbl_status = lv_label_create(scr);
  lv_obj_add_style(lbl_status, &st_status, 0);
  lv_label_set_text(lbl_status, "Bereit");
  lv_obj_align(lbl_status, LV_ALIGN_TOP_LEFT, 22, 54);

  lbl_count = lv_label_create(scr);
  lv_obj_add_style(lbl_count, &st_big, 0);
  lv_label_set_text(lbl_count, "0");
  lv_obj_align(lbl_count, LV_ALIGN_LEFT_MID, 40, -30);

  lbl_target = lv_label_create(scr);
  lv_obj_add_style(lbl_target, &st_mid, 0);
  lv_label_set_text(lbl_target, "Ziel: 50");
  lv_obj_align(lbl_target, LV_ALIGN_LEFT_MID, 42, 40);

  bar_prog = lv_bar_create(scr);
  lv_obj_set_size(bar_prog, 520, 18);
  lv_obj_align(bar_prog, LV_ALIGN_BOTTOM_LEFT, 20, -20);
  lv_bar_set_range(bar_prog, 0, 100);
  lv_bar_set_value(bar_prog, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(bar_prog, lv_color_hex(0xE5E7EB), 0);
  lv_obj_set_style_bg_color(bar_prog, lv_color_hex(0xFF7A00), LV_PART_INDICATOR);

  btn_start = make_btn(scr, "START", &st_btn_orange, btn_start_cb);
  lv_obj_set_size(btn_start, 210, 90);
  lv_obj_align(btn_start, LV_ALIGN_RIGHT_MID, -40, -90);

  btn_stop = make_btn(scr, "STOP", &st_btn_red, btn_stop_cb);
  lv_obj_set_size(btn_stop, 210, 90);
  lv_obj_align(btn_stop, LV_ALIGN_RIGHT_MID, -40, 10);

  btn_reset = make_btn(scr, "RESET", &st_btn_white, btn_reset_cb);
  lv_obj_set_size(btn_reset, 210, 90);
  lv_obj_align(btn_reset, LV_ALIGN_RIGHT_MID, -40, 110);

  ui_update_numbers();
}

// ======================
// 13) LVGL init
// ======================
static void init_lvgl() {
  lv_init();

  lv_disp_draw_buf_init(&draw_buf, buf1, nullptr, SCREEN_W * LVGL_BUF_LINES);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCREEN_W;
  disp_drv.ver_res = SCREEN_H;
  disp_drv.flush_cb = flush_cb;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  touch_init();
  if (touch_ok) {
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = indev_read_cb;
    lv_indev_drv_register(&indev_drv);
    Serial.printf("[TOUCH] GT911 OK addr=0x%02X\n", gt_addr);
  } else {
    Serial.println("[TOUCH] GT911 not found (UI still works without touch)");
  }
}

// ======================
// 14) Setup/Loop
// ======================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Backlight optional
  if (GPIO_BL >= 0) {
    pinMode(GPIO_BL, OUTPUT);
    digitalWrite(GPIO_BL, HIGH);
  }

  // NVS
  prefs.begin("bandware", false);
  target = prefs.getUShort("target", 50);
  debounce_ms = prefs.getUShort("deb", DEFAULT_DEBOUNCE_MS);
  g_debounce_ms_isr = debounce_ms;

  // GPIO
  pinMode(GPIO_MOTOR, OUTPUT);
  motor_set(false);

  pinMode(GPIO_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GPIO_SENSOR), sensor_isr, FALLING);

  // Display
  lcd.begin();

  // LVGL + UI
  init_lvgl();
  init_styles();
  build_ui();

  ui_set_status(AppState::IDLE);
  ui_update_numbers();

  Serial.println("[APP] Ready");
}

void loop() {
  lv_timer_handler();
  delay(5);

  if (app_state == AppState::RUNNING) {
    if (g_count >= target) {
      motor_set(false);
      ui_set_status(AppState::DONE);
    }
  }

  static uint32_t last_ui_ms = 0;
  uint32_t now = millis();
  if (now - last_ui_ms >= 50) {
    last_ui_ms = now;
    ui_update_numbers();
  }
}
