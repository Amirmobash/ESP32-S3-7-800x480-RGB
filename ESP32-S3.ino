/****************************************************
 * Sunton ESP32-S3 7" RGB 800x480 + GT911 (Cap Touch)
 * LVGL 8.x + LovyanGFX
 * Project: Bandware Counter + Motor Control
 ****************************************************/

#define LGFX_USE_V1

// اگر lv_conf.h را کنار اسکچ گذاشتی:
#define LV_CONF_INCLUDE_SIMPLE 1

#include <Arduino.h>
#include <Preferences.h>
#include <lvgl.h>

#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <lgfx/v1/light/Light_PWM.hpp>
#include <lgfx/v1/touch/Touch_GT911.hpp>

// ======================
// 0) USER CONFIG (فقط اینجا)
// ======================
static const int SCREEN_W = 800;
static const int SCREEN_H = 480;

// سنسور + موتور (به دلخواهت عوض کن)
static const int GPIO_SENSOR = 10;   // ورودی سنسور
static const int GPIO_MOTOR  = 11;   // خروجی موتور (HIGH=ON)

// debounce پیش‌فرض (ms)
static const uint16_t DEFAULT_DEBOUNCE_MS = 4;

// LVGL buffer lines (کم/زیاد = RAM کمتر/بیشتر + سرعت)
static const int LVGL_BUF_LINES = 10;   // 10 خط معمولاً خوبه

// بک‌لایت این بردها غالباً GPIO2 است
static const int BL_PIN = 2;

// ریست تاچ (GT911) روی خیلی از مدل‌ها GPIO38 است
static const int TOUCH_RST_PIN = 38;

// ======================
// 1) DISPLAY + TOUCH (LovyanGFX RGB)
// ======================
class LGFX : public lgfx::LGFX_Device {
public:
  lgfx::Bus_RGB     _bus_instance;
  lgfx::Panel_RGB   _panel_instance;
  lgfx::Light_PWM   _light_instance;
  lgfx::Touch_GT911 _touch_instance;

  LGFX() {
    // Panel basic
    {
      auto cfg = _panel_instance.config();
      cfg.memory_width  = SCREEN_W;
      cfg.memory_height = SCREEN_H;
      cfg.panel_width   = SCREEN_W;
      cfg.panel_height  = SCREEN_H;
      cfg.offset_x      = 0;
      cfg.offset_y      = 0;
      _panel_instance.config(cfg);
    }

    // Panel detail
    {
      auto cfg = _panel_instance.config_detail();
      cfg.use_psram = 1; // بهتر برای DMA/بافر
      _panel_instance.config_detail(cfg);
    }

    // RGB bus pins + timings
    {
      auto cfg = _bus_instance.config();
      cfg.panel = &_panel_instance;

      cfg.pin_d0  = GPIO_NUM_15;  // B0
      cfg.pin_d1  = GPIO_NUM_7;   // B1
      cfg.pin_d2  = GPIO_NUM_6;   // B2
      cfg.pin_d3  = GPIO_NUM_5;   // B3
      cfg.pin_d4  = GPIO_NUM_4;   // B4
      cfg.pin_d5  = GPIO_NUM_9;   // G0
      cfg.pin_d6  = GPIO_NUM_46;  // G1
      cfg.pin_d7  = GPIO_NUM_3;   // G2
      cfg.pin_d8  = GPIO_NUM_8;   // G3
      cfg.pin_d9  = GPIO_NUM_16;  // G4
      cfg.pin_d10 = GPIO_NUM_1;   // G5
      cfg.pin_d11 = GPIO_NUM_14;  // R0
      cfg.pin_d12 = GPIO_NUM_21;  // R1
      cfg.pin_d13 = GPIO_NUM_47;  // R2
      cfg.pin_d14 = GPIO_NUM_48;  // R3
      cfg.pin_d15 = GPIO_NUM_45;  // R4

      cfg.pin_henable = GPIO_NUM_41; // DE
      cfg.pin_vsync   = GPIO_NUM_40;
      cfg.pin_hsync   = GPIO_NUM_39;
      cfg.pin_pclk    = GPIO_NUM_42;

      // خیلی مهم: روی این بردها 12MHz پایدارتره
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

      _bus_instance.config(cfg);
    }

    _panel_instance.setBus(&_bus_instance);

    // Backlight
    {
      auto cfg = _light_instance.config();
      cfg.pin_bl = (gpio_num_t)BL_PIN;
      _light_instance.config(cfg);
      _panel_instance.light(&_light_instance);
    }

    // Touch (GT911)
    {
      auto cfg = _touch_instance.config();
      cfg.x_min = 0;
      cfg.y_min = 0;
      cfg.x_max = SCREEN_W;
      cfg.y_max = SCREEN_H;

      cfg.bus_shared = false;
      cfg.offset_rotation = 0;

      cfg.i2c_port = I2C_NUM_0;
      cfg.pin_sda  = GPIO_NUM_19;
      cfg.pin_scl  = GPIO_NUM_20;
      cfg.pin_int  = GPIO_NUM_NC;
      cfg.pin_rst  = (gpio_num_t)TOUCH_RST_PIN;
      cfg.freq     = 100000;

      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }

    setPanel(&_panel_instance);
  }
};

static LGFX gfx;

// ======================
// 2) LVGL display buffer + driver
// ======================
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1 = nullptr;
static lv_color_t *buf2 = nullptr;

static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  // DMA push (بهتر)
  if (gfx.getStartCount() == 0) gfx.startWrite();

  const int32_t w = (area->x2 - area->x1 + 1);
  const int32_t h = (area->y2 - area->y1 + 1);

  gfx.pushImageDMA(area->x1, area->y1, w, h, (lgfx::rgb565_t*)&color_p->full);
  lv_disp_flush_ready(disp);
}

static void my_touch_read(lv_indev_drv_t * indev, lv_indev_data_t * data) {
  (void)indev;
  uint16_t x, y;
  bool touched = gfx.getTouch(&x, &y);

  data->state = touched ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
  data->point.x = (lv_coord_t)x;
  data->point.y = (lv_coord_t)y;
}

// ======================
// 3) APP STATE + LOGIC
// ======================
enum class State : uint8_t { IDLE, RUNNING, DONE, STOPPED, ERROR };

static Preferences prefs;
static volatile uint32_t g_count = 0;
static volatile uint32_t g_last_isr_ms = 0;

static uint16_t target = 50;
static uint16_t debounce_ms = DEFAULT_DEBOUNCE_MS;
static State state = State::IDLE;

// ======================
// 4) UI objects + styles
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

static lv_obj_t* keypad_win = nullptr;
static lv_obj_t* keypad_lbl = nullptr;
static String keypad_value;

static lv_style_t st_bg, st_title, st_status, st_big, st_mid;
static lv_style_t st_btn_orange, st_btn_white, st_btn_red;

// ======================
// 5) Motor + Sensor
// ======================
static void motor_set(bool on) {
  digitalWrite(GPIO_MOTOR, on ? HIGH : LOW);
}

void IRAM_ATTR sensor_isr() {
  uint32_t now = millis();
  if (now - g_last_isr_ms < debounce_ms) return;
  g_last_isr_ms = now;
  g_count++;
}

// ======================
// 6) UI helpers
// ======================
static const char* state_text(State s) {
  switch (s) {
    case State::IDLE:    return "Bereit";
    case State::RUNNING: return "Läuft…";
    case State::DONE:    return "Fertig: Bitte Band entnehmen";
    case State::STOPPED: return "Stopp";
    case State::ERROR:   return "Fehler";
    default:             return "—";
  }
}

static void ui_set_status(State s) {
  state = s;
  if (lbl_status) lv_label_set_text(lbl_status, state_text(s));
}

static void ui_update_numbers() {
  char b1[32], b2[32];
  snprintf(b1, sizeof(b1), "%lu", (unsigned long)g_count);
  snprintf(b2, sizeof(b2), "Ziel: %u", (unsigned)target);

  if (lbl_count)  lv_label_set_text(lbl_count, b1);
  if (lbl_target) lv_label_set_text(lbl_target, b2);

  uint16_t pct = 0;
  if (target > 0) {
    uint32_t c = g_count;
    pct = (c >= target) ? 100 : (uint16_t)((c * 100UL) / target);
  }
  if (bar_prog) lv_bar_set_value(bar_prog, pct, LV_ANIM_ON);
}

static void animate_press(lv_obj_t* obj) {
  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, obj);
  lv_anim_set_time(&a, 80);
  lv_anim_set_values(&a, 100, 95);
  lv_anim_set_exec_cb(&a, [](void* v, int32_t val){
    lv_obj_set_style_transform_zoom((lv_obj_t*)v, val, 0);
  });
  lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
  lv_anim_start(&a);

  lv_anim_t b;
  lv_anim_init(&b);
  lv_anim_set_var(&b, obj);
  lv_anim_set_time(&b, 120);
  lv_anim_set_delay(&b, 80);
  lv_anim_set_values(&b, 95, 100);
  lv_anim_set_exec_cb(&b, [](void* v, int32_t val){
    lv_obj_set_style_transform_zoom((lv_obj_t*)v, val, 0);
  });
  lv_anim_set_path_cb(&b, lv_anim_path_ease_in_out);
  lv_anim_start(&b);
}

// ======================
// 7) Keypad (target input)
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
  lv_obj_t* m = (lv_obj_t*)lv_event_get_target(e);
  const char* txt = lv_btnmatrix_get_btn_text(m, lv_btnmatrix_get_selected_btn(m));
  if (!txt) return;

  if (strcmp(txt, "OK") == 0) { keypad_apply(); return; }
  if (strcmp(txt, "C") == 0)  { keypad_value = ""; lv_label_set_text(keypad_lbl, ""); return; }
  if (strcmp(txt, "<") == 0)  {
    if (keypad_value.length()) keypad_value.remove(keypad_value.length()-1);
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
// 8) Button callbacks + logic
// ======================
static void start_run() {
  g_count = 0;
  ui_update_numbers();
  motor_set(true);
  ui_set_status(State::RUNNING);
}

static void stop_run(State next) {
  motor_set(false);
  ui_set_status(next);
}

static void btn_start_cb(lv_event_t* e) {
  lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
  animate_press(btn);
  if (state != State::RUNNING) start_run();
}

static void btn_stop_cb(lv_event_t* e) {
  lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
  animate_press(btn);
  if (state == State::RUNNING) stop_run(State::STOPPED);
  else motor_set(false);
}

static void btn_reset_cb(lv_event_t* e) {
  lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
  animate_press(btn);
  motor_set(false);
  g_count = 0;
  ui_set_status(State::IDLE);
  ui_update_numbers();
}

static void btn_target_cb(lv_event_t* e) {
  lv_obj_t* btn = (lv_obj_t*)lv_event_get_target(e);
  animate_press(btn);
  open_keypad();
}

// ======================
// 9) Styles + UI build
// ======================
static void init_styles() {
  lv_style_init(&st_bg);
  lv_style_set_bg_color(&st_bg, lv_color_hex(0xF4F6F8));
  lv_style_set_bg_opa(&st_bg, LV_OPA_COVER);

  lv_style_init(&st_title);
  lv_style_set_text_color(&st_title, lv_color_hex(0x111827));
  lv_style_set_text_font(&st_title, &lv_font_montserrat_28);

  lv_style_init(&st_status);
  lv_style_set_text_color(&st_status, lv_color_hex(0x374151));
  lv_style_set_text_font(&st_status, &lv_font_montserrat_18);

  lv_style_init(&st_big);
  lv_style_set_text_color(&st_big, lv_color_hex(0x111827));
  lv_style_set_text_font(&st_big, &lv_font_montserrat_48);

  lv_style_init(&st_mid);
  lv_style_set_text_color(&st_mid, lv_color_hex(0x111827));
  lv_style_set_text_font(&st_mid, &lv_font_montserrat_22);

  lv_style_init(&st_btn_orange);
  lv_style_set_bg_color(&st_btn_orange, lv_color_hex(0xFF7A00));
  lv_style_set_bg_opa(&st_btn_orange, LV_OPA_COVER);
  lv_style_set_radius(&st_btn_orange, 16);
  lv_style_set_shadow_width(&st_btn_orange, 18);
  lv_style_set_shadow_opa(&st_btn_orange, LV_OPA_20);
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

  btn_target = make_btn(scr, "ZIEL", &st_btn_white, btn_target_cb);
  lv_obj_set_size(btn_target, 120, 56);
  lv_obj_align(btn_target, LV_ALIGN_TOP_LEFT, 260, 10);

  ui_update_numbers();
}

// ======================
// 10) LVGL init
// ======================
static void init_lvgl() {
  lv_init();

  // بافرها را در PSRAM بگیر (برای ESP32-S3 بهتر)
  size_t buf_pixels = SCREEN_W * LVGL_BUF_LINES;
  buf1 = (lv_color_t*)heap_caps_malloc(buf_pixels * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  buf2 = (lv_color_t*)heap_caps_malloc(buf_pixels * sizeof(lv_color_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, buf_pixels);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCREEN_W;
  disp_drv.ver_res = SCREEN_H;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touch_read;
  lv_indev_drv_register(&indev_drv);
}

// ======================
// 11) Setup / Loop
// ======================
void setup() {
  Serial.begin(115200);
  delay(200);

  // NVS
  prefs.begin("bandware", false);
  target = prefs.getUShort("target", 50);
  debounce_ms = prefs.getUShort("deb", DEFAULT_DEBOUNCE_MS);

  // GPIO
  pinMode(GPIO_MOTOR, OUTPUT);
  motor_set(false);

  pinMode(GPIO_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GPIO_SENSOR), sensor_isr, FALLING);

  // Display
  gfx.begin();
  gfx.setBrightness(255);  // اگر این نباشد، ممکن است صفحه سیاه بماند

  // LVGL + UI
  init_lvgl();
  init_styles();
  build_ui();

  ui_set_status(State::IDLE);
  ui_update_numbers();

  Serial.println("[APP] Ready");
}

void loop() {
  lv_timer_handler();
  delay(5);

  // منطق دستگاه
  if (state == State::RUNNING) {
    if (g_count >= target) {
      motor_set(false);
      ui_set_status(State::DONE);
    }
  }

  // رفرش عددها
  static uint32_t last_ui_ms = 0;
  uint32_t now = millis();
  if (now - last_ui_ms >= 50) {
    last_ui_ms = now;
    ui_update_numbers();
  }
}
