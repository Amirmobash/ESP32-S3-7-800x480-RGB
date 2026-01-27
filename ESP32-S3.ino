/****************************************************
 * ESP32-S3 + 7" 800x480 RGB (Sunton ESP32-8048S070) + LVGL
 * Touch: GT911 (I2C)
 *
 * UI: target input + on-screen keyboard + Start/Stop/Reset
 * Count pulses on GPIO_SENSOR, drive motor/relay on GPIO_MOTOR
 ****************************************************/

#include <Arduino.h>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>
#include <lvgl.h>

// ---------------- USER IO (change if you want) ----------------
static const int GPIO_MOTOR  = 17;   // output to motor driver / relay input
static const int GPIO_SENSOR = 18;   // input pulses from PC817 OUT (open-collector)

// debounce for sensor pulses
static const uint32_t SENSOR_DEBOUNCE_MS = 3;

// screen
static const int SCREEN_W = 800;
static const int SCREEN_H = 480;
static const int LVGL_BUF_LINES = 40;
// --------------------------------------------------------------

// ---------------- Display driver (pins from Sunton 7" TN board) ----------------
// Pin mapping and timing based on known Sunton ESP32-S3 7" (ESP32-8048S070) config. :contentReference[oaicite:4]{index=4}
class LGFX : public lgfx::LGFX_Device {
public:
  lgfx::Bus_RGB     _bus;
  lgfx::Panel_RGB   _panel;
  lgfx::Light_PWM   _light;
  lgfx::Touch_GT911 _touch;

  LGFX() {
    // Panel basic
    {
      auto cfg = _panel.config();
      cfg.memory_width  = SCREEN_W;
      cfg.memory_height = SCREEN_H;
      cfg.panel_width   = SCREEN_W;
      cfg.panel_height  = SCREEN_H;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      _panel.config(cfg);
    }
    // Use PSRAM
    {
      auto cfg = _panel.config_detail();
      cfg.use_psram = 1;
      _panel.config_detail(cfg);
    }
    // RGB bus pins
    {
      auto cfg = _bus.config();
      cfg.panel = &_panel;

      // D0..D15 (RGB565 parallel)
      cfg.pin_d0  = GPIO_NUM_15; // B0
      cfg.pin_d1  = GPIO_NUM_7;  // B1
      cfg.pin_d2  = GPIO_NUM_6;  // B2
      cfg.pin_d3  = GPIO_NUM_5;  // B3
      cfg.pin_d4  = GPIO_NUM_4;  // B4
      cfg.pin_d5  = GPIO_NUM_9;  // G0
      cfg.pin_d6  = GPIO_NUM_46; // G1
      cfg.pin_d7  = GPIO_NUM_3;  // G2
      cfg.pin_d8  = GPIO_NUM_8;  // G3
      cfg.pin_d9  = GPIO_NUM_16; // G4
      cfg.pin_d10 = GPIO_NUM_1;  // G5
      cfg.pin_d11 = GPIO_NUM_14; // R0
      cfg.pin_d12 = GPIO_NUM_21; // R1
      cfg.pin_d13 = GPIO_NUM_47; // R2
      cfg.pin_d14 = GPIO_NUM_48; // R3
      cfg.pin_d15 = GPIO_NUM_45; // R4

      // Sync / DE / PCLK
      cfg.pin_henable = GPIO_NUM_41; // DE
      cfg.pin_vsync   = GPIO_NUM_40; // VSYNC
      cfg.pin_hsync   = GPIO_NUM_39; // HSYNC
      cfg.pin_pclk    = GPIO_NUM_42; // PCLK

      // Pixel clock: 12MHz recommended to avoid flicker on this board. :contentReference[oaicite:5]{index=5}
      cfg.freq_write  = 12000000;

      // Timings
      cfg.hsync_polarity    = 0;
      cfg.hsync_front_porch = 8;
      cfg.hsync_pulse_width = 2;
      cfg.hsync_back_porch  = 43;

      cfg.vsync_polarity    = 0;
      cfg.vsync_front_porch = 8;
      cfg.vsync_pulse_width = 2;
      cfg.vsync_back_porch  = 12;

      cfg.pclk_idle_high    = 1;

      _bus.config(cfg);
    }
    _panel.setBus(&_bus);

    // Backlight
    {
      auto cfg = _light.config();
      cfg.pin_bl = GPIO_NUM_2;
      _light.config(cfg);
      _panel.light(&_light);
    }

    // Touch GT911 (I2C)
    {
      auto cfg = _touch.config();
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
      cfg.pin_rst  = GPIO_NUM_38;
      cfg.freq     = 100000;

      _touch.config(cfg);
      _panel.setTouch(&_touch);
    }

    setPanel(&_panel);
  }
};

static LGFX gfx;

// ---------------- LVGL glue ----------------
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

// ---------------- App logic ----------------
volatile uint32_t g_count = 0;
volatile uint32_t g_last_ms = 0;

static int target = 100;
static bool running = false;

static lv_obj_t* label_count;
static lv_obj_t* label_target;
static lv_obj_t* ta_target;
static lv_obj_t* kb;

static void motor_set(bool on) {
  digitalWrite(GPIO_MOTOR, on ? HIGH : LOW);
}

static void IRAM_ATTR isr_sensor() {
  uint32_t now = millis();
  if (now - g_last_ms < SENSOR_DEBOUNCE_MS) return;
  g_last_ms = now;

  if (running) g_count++;
}

static void ui_update() {
  char a[64], b[64];
  snprintf(a, sizeof(a), "Count: %lu", (unsigned long)g_count);
  snprintf(b, sizeof(b), "Target: %d", target);
  lv_label_set_text(label_count, a);
  lv_label_set_text(label_target, b);
}

static void stop_run() {
  running = false;
  motor_set(false);
}

static void start_run() {
  const char* txt = lv_textarea_get_text(ta_target);
  int v = atoi(txt);
  if (v < 1) v = 1;
  if (v > 1000) v = 1000;
  target = v;

  g_count = 0;
  running = true;
  motor_set(true);
  ui_update();
}

// UI callbacks
static void btn_start_cb(lv_event_t* e) { (void)e; start_run(); }
static void btn_stop_cb(lv_event_t* e)  { (void)e; stop_run(); ui_update(); }
static void btn_reset_cb(lv_event_t* e) { (void)e; stop_run(); g_count = 0; ui_update(); }

static void kb_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t* keyboard = (lv_obj_t*)lv_event_get_target(e);
  if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
    lv_obj_add_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
  }
}

static void ta_event_cb(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_FOCUSED) {
    lv_keyboard_set_textarea(kb, ta_target);
    lv_obj_clear_flag(kb, LV_OBJ_FLAG_HIDDEN);
  }
}

static void create_ui() {
  lv_obj_t* scr = lv_scr_act();

  lv_obj_t* title = lv_label_create(scr);
  lv_label_set_text(title, "Bandware Counter");
  lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

  lv_obj_t* lbl = lv_label_create(scr);
  lv_label_set_text(lbl, "Target (1..1000):");
  lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 30, 70);

  ta_target = lv_textarea_create(scr);
  lv_textarea_set_one_line(ta_target, true);
  lv_textarea_set_text(ta_target, "100");
  lv_textarea_set_max_length(ta_target, 4);
  lv_textarea_set_accepted_chars(ta_target, "0123456789");
  lv_obj_set_size(ta_target, 220, 50);
  lv_obj_align(ta_target, LV_ALIGN_TOP_LEFT, 30, 100);
  lv_obj_add_event_cb(ta_target, ta_event_cb, LV_EVENT_ALL, nullptr);

  lv_obj_t* btnStart = lv_btn_create(scr);
  lv_obj_set_size(btnStart, 180, 70);
  lv_obj_align(btnStart, LV_ALIGN_TOP_LEFT, 30, 170);
  lv_obj_add_event_cb(btnStart, btn_start_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* t1 = lv_label_create(btnStart);
  lv_label_set_text(t1, "START");
  lv_obj_center(t1);

  lv_obj_t* btnStop = lv_btn_create(scr);
  lv_obj_set_size(btnStop, 180, 70);
  lv_obj_align(btnStop, LV_ALIGN_TOP_LEFT, 230, 170);
  lv_obj_add_event_cb(btnStop, btn_stop_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* t2 = lv_label_create(btnStop);
  lv_label_set_text(t2, "STOP");
  lv_obj_center(t2);

  lv_obj_t* btnReset = lv_btn_create(scr);
  lv_obj_set_size(btnReset, 180, 70);
  lv_obj_align(btnReset, LV_ALIGN_TOP_LEFT, 430, 170);
  lv_obj_add_event_cb(btnReset, btn_reset_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t* t3 = lv_label_create(btnReset);
  lv_label_set_text(t3, "RESET");
  lv_obj_center(t3);

  label_count = lv_label_create(scr);
  lv_obj_set_style_text_font(label_count, &lv_font_montserrat_28, 0);
  lv_obj_align(label_count, LV_ALIGN_TOP_LEFT, 30, 270);

  label_target = lv_label_create(scr);
  lv_obj_set_style_text_font(label_target, &lv_font_montserrat_28, 0);
  lv_obj_align(label_target, LV_ALIGN_TOP_LEFT, 30, 320);

  // On-screen keyboard
  kb = lv_keyboard_create(scr);
  lv_obj_set_size(kb, 780, 200);
  lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_obj_add_event_cb(kb, kb_event_cb, LV_EVENT_ALL, nullptr);
  lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);

  ui_update();
}

void setup() {
  Serial.begin(115200);

  pinMode(GPIO_MOTOR, OUTPUT);
  motor_set(false);

  // PC817 output is open-collector -> use pull-up
  pinMode(GPIO_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GPIO_SENSOR), isr_sensor, FALLING);

  gfx.init();
  gfx.setRotation(0);

  lv_init();

  // LVGL buffer (use PSRAM if available)
  lv_buf1 = (lv_color_t*)heap_caps_malloc(SCREEN_W * LVGL_BUF_LINES * sizeof(lv_color_t),
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!lv_buf1) {
    // fallback internal RAM
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

  create_ui();
}

void loop() {
  lv_timer_handler();
  delay(5);

  if (running && (int)g_count >= target) {
    stop_run();
    ui_update();
  }

  static uint32_t lastUi = 0;
  uint32_t now = millis();
  if (now - lastUi > 100) {
    lastUi = now;
    ui_update();
  }
}
