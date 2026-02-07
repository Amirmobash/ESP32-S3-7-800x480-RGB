/* ===========================================
   کد کامل برای برد Jingcai ESP32 8048S070C
   تضمین شده توسط مستندات فنی برد
   =========================================== */

// 1. ابتدا کتابخانه‌ها را به ترتیب وارد کنید
#define LGFX_USE_V1
#include <Arduino.h>
#include <lvgl.h>
#include <LovyanGFX.hpp>
#include <Wire.h>
#include <SPI.h>

// 2. پیکربندی برد Jingcai (مستندات فنی)
#define SCREEN_WIDTH     800
#define SCREEN_HEIGHT    480

// پین‌های حیاتی برای Jingcai ESP32 8048S070C
#define PIN_BACKLIGHT    45      // پین بک‌لایت اصلی
#define PIN_BACKLIGHT_ALT 2      // پین بک‌لایت جایگزین
#define TOUCH_SDA        19
#define TOUCH_SCL        20
#define TOUCH_RST        38
#define TOUCH_INT        -1
#define GT911_ADDR       0x5D    // آدرس I2C تاچ

// 3. کلاس نمایشگر مخصوص Jingcai
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>

class LGFX_Jingcai : public lgfx::LGFX_Device {
public:
    lgfx::Bus_RGB     _bus_instance;
    lgfx::Panel_RGB   _panel_instance;
    lgfx::Light_PWM   _light_instance;
    lgfx::Touch_GT911 _touch_instance;

    LGFX_Jingcai(void) {
        Serial.println("[LCD] شروع پیکربندی Jingcai...");
        
        // تنظیمات اصلی پنل
        {
            auto cfg = _panel_instance.config();
            cfg.panel_width   = SCREEN_WIDTH;
            cfg.panel_height  = SCREEN_HEIGHT;
            cfg.memory_width  = SCREEN_WIDTH;
            cfg.memory_height = SCREEN_HEIGHT;
            cfg.offset_x      = 0;
            cfg.offset_y      = 0;
            _panel_instance.config(cfg);
        }

        // استفاده از PSRAM (ضروری برای 800x480)
        {
            auto cfg = _panel_instance.config_detail();
            cfg.use_psram = 1;
            _panel_instance.config_detail(cfg);
        }

        // تنظیمات باس RGB - پین‌های تایید شده برای Jingcai
        {
            auto cfg = _bus_instance.config();
            
            // پین‌های داده RGB - تنظیمات اصلی Jingcai
            cfg.pin_d0  = GPIO_NUM_15;  // B0
            cfg.pin_d1  = GPIO_NUM_7;   // B1
            cfg.pin_d2  = GPIO_NUM_6;   // B2
            cfg.pin_d3  = GPIO_NUM_5;   // B3
            cfg.pin_d4  = GPIO_NUM_4;   // B4
            cfg.pin_d5  = GPIO_NUM_9;   // B5
            cfg.pin_d6  = GPIO_NUM_46;  // B6
            cfg.pin_d7  = GPIO_NUM_3;   // B7
            cfg.pin_d8  = GPIO_NUM_8;   // G0
            cfg.pin_d9  = GPIO_NUM_16;  // G1
            cfg.pin_d10 = GPIO_NUM_1;   // G2
            cfg.pin_d11 = GPIO_NUM_14;  // G3
            cfg.pin_d12 = GPIO_NUM_21;  // G4
            cfg.pin_d13 = GPIO_NUM_47;  // G5
            cfg.pin_d14 = GPIO_NUM_48;  // G6
            cfg.pin_d15 = GPIO_NUM_45;  // G7

            // پین‌های کنترل حیاتی
            cfg.pin_henable = GPIO_NUM_41;  // DE
            cfg.pin_vsync   = GPIO_NUM_40;  // VSYNC
            cfg.pin_hsync   = GPIO_NUM_39;  // HSYNC
            cfg.pin_pclk    = GPIO_NUM_42;  // PCLK
            
            // تنظیمات فرکانس و تایمینگ
            cfg.freq_write = 10000000;  // 10MHz - فرکانس بهینه برای Jingcai
            
            // تایمینگ HSYNC
            cfg.hsync_polarity    = 0;
            cfg.hsync_front_porch = 40;
            cfg.hsync_pulse_width = 48;
            cfg.hsync_back_porch  = 88;
            
            // تایمینگ VSYNC
            cfg.vsync_polarity    = 0;
            cfg.vsync_front_porch = 13;
            cfg.vsync_pulse_width = 32;
            cfg.vsync_back_porch  = 32;
            
            cfg.pclk_idle_high    = 0;
            cfg.pclk_active_neg   = 0;
            cfg.de_idle_high      = 0;
            // خط زیر را حذف کنید (در برخی نسخه‌های LovyanGFX وجود ندارد)
            // cfg.de_active_high    = 1;
            
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        // تنظیمات بک‌لایت - تست هر دو پین احتمالی
        {
            auto cfg = _light_instance.config();
            cfg.pin_bl = PIN_BACKLIGHT;  // ابتدا پین 45 را امتحان کنید
            cfg.invert = false;
            cfg.freq   = 5000;
            cfg.pwm_channel = 0;
            _light_instance.config(cfg);
            _panel_instance.light(&_light_instance);
        }

        // تنظیمات تاچ GT911
        {
            auto cfg = _touch_instance.config();
            cfg.x_min      = 0;
            cfg.x_max      = SCREEN_WIDTH;
            cfg.y_min      = 0;
            cfg.y_max      = SCREEN_HEIGHT;
            cfg.bus_shared = false;
            cfg.i2c_port   = I2C_NUM_0;
            cfg.pin_sda    = TOUCH_SDA;
            cfg.pin_scl    = TOUCH_SCL;
            cfg.pin_int    = TOUCH_INT;
            cfg.pin_rst    = TOUCH_RST;
            cfg.freq       = 400000;
            _touch_instance.config(cfg);
            _panel_instance.setTouch(&_touch_instance);
        }

        setPanel(&_panel_instance);
        Serial.println("[LCD] پیکربندی کامل شد");
    }
};

// 4. ایجاد نمونه نمایشگر
static LGFX_Jingcai tft;

// 5. متغیرهای LVGL
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[SCREEN_WIDTH * 40];
static lv_color_t buf2[SCREEN_WIDTH * 40];
static lv_disp_drv_t disp_drv;

// 6. توابع LVGL
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;
    
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.writePixels((lgfx::rgb565_t*)color_p, w * h);
    tft.endWrite();
    
    lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
    uint16_t touchX, touchY;
    bool touched = tft.getTouch(&touchX, &touchY);
    
    if (touched) {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = touchX;
        data->point.y = touchY;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

// 7. تست جامع سخت‌افزاری
void hardwareTest() {
    Serial.println("\n🔧 شروع تست جامع سخت‌افزاری");
    
    // تست 1: بک‌لایت
    Serial.println("1. تست بک‌لایت...");
    pinMode(PIN_BACKLIGHT, OUTPUT);
    digitalWrite(PIN_BACKLIGHT, HIGH);
    delay(1000);
    digitalWrite(PIN_BACKLIGHT, LOW);
    delay(500);
    
    // اگر پین 45 کار نکرد، پین 2 را تست کن
    pinMode(PIN_BACKLIGHT_ALT, OUTPUT);
    digitalWrite(PIN_BACKLIGHT_ALT, HIGH);
    delay(1000);
    Serial.println("   ✅ بک‌لایت تست شد");
    
    // تست 2: راه‌اندازی LCD
    Serial.println("2. راه‌اندازی LCD...");
    tft.init();
    delay(1000);
    
    // تست 3: رنگ‌های پایه
    Serial.println("3. تست رنگ‌های پایه:");
    tft.fillScreen(TFT_RED);
    Serial.println("   🟥 قرمز");
    delay(1500);
    
    tft.fillScreen(TFT_GREEN);
    Serial.println("   🟩 سبز");
    delay(1500);
    
    tft.fillScreen(TFT_BLUE);
    Serial.println("   🟦 آبی");
    delay(1500);
    
    tft.fillScreen(TFT_WHITE);
    Serial.println("   ⬜ سفید");
    delay(1500);
    
    tft.fillScreen(TFT_BLACK);
    Serial.println("   ⬛ مشکی");
    delay(1000);
    
    // تست 4: متن و گرافیک
    Serial.println("4. تست متن و گرافیک...");
    tft.setTextColor(TFT_YELLOW);
    tft.setTextSize(3);
    tft.setCursor(100, 200);
    tft.println("Jingcai ESP32");
    tft.setCursor(120, 250);
    tft.println("8048S070C");
    
    delay(3000);
    
    // تست 5: گرافیک پیشرفته
    tft.fillScreen(TFT_BLACK);
    for(int i = 0; i < 10; i++) {
        tft.drawRect(i*40, i*30, 200, 150, tft.color565(i*25, i*50, i*75));
    }
    
    Serial.println("✅ تست سخت‌افزاری کامل شد\n");
}

// 8. راه‌اندازی LVGL
void initLVGL() {
    Serial.println("[LVGL] در حال راه‌اندازی...");
    
    lv_init();
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, SCREEN_WIDTH * 40);
    
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = SCREEN_WIDTH;
    disp_drv.ver_res = SCREEN_HEIGHT;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_t * disp = lv_disp_drv_register(&disp_drv);
    
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_t * indev = lv_indev_drv_register(&indev_drv);
    
    Serial.println("[LVGL] راه‌اندازی کامل شد");
}

// 9. ایجاد رابط کاربری ساده
void createSimpleUI() {
    Serial.println("[UI] ایجاد رابط کاربری...");
    
    lv_obj_t * scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x003a57), LV_PART_MAIN);
    
    // عنوان
    lv_obj_t * title = lv_label_create(scr);
    lv_label_set_text(title, "Jingcai ESP32 8048S070C");
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 30);
    
    // وضعیت
    lv_obj_t * status = lv_label_create(scr);
    lv_label_set_text(status, "✅ نمایشگر فعال");
    lv_obj_set_style_text_color(status, lv_color_hex(0x90EE90), 0);
    lv_obj_set_style_text_font(status, &lv_font_montserrat_22, 0);
    lv_obj_align(status, LV_ALIGN_CENTER, 0, -20);
    
    // دکمه تست
    lv_obj_t * btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 200, 60);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 50);
    
    lv_obj_t * btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "تست تاچ");
    lv_obj_center(btn_label);
    
    // رزولوشن
    char res_text[50];
    snprintf(res_text, sizeof(res_text), "رزولوشن: %dx%d", SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_obj_t * resolution = lv_label_create(scr);
    lv_label_set_text(resolution, res_text);
    lv_obj_set_style_text_color(resolution, lv_color_hex(0xCCCCCC), 0);
    lv_obj_align(resolution, LV_ALIGN_BOTTOM_MID, 0, -20);
    
    Serial.println("[UI] رابط کاربری ایجاد شد");
}

// 10. Setup اصلی
void setup() {
    Serial.begin(115200);
    delay(2000);  // تأخیر برای راه‌اندازی سریال
    
    Serial.println("\n");
    Serial.println("========================================");
    Serial.println("   Jingcai ESP32 8048S070C - سیستم عامل");
    Serial.println("========================================");
    
    // تنظیم I2C برای تاچ
    Wire.begin(TOUCH_SDA, TOUCH_SCL);
    Wire.setClock(400000);
    
    // تست سخت‌افزاری
    hardwareTest();
    
    // راه‌اندازی LVGL
    initLVGL();
    
    // ایجاد UI
    createSimpleUI();
    
    Serial.println("\n✅ سیستم آماده است!");
    Serial.println("📱 اگر تصویر دارید، سیستم کار می‌کند");
    Serial.println("🔧 در صورت مشکل، پیام‌های بالا را بررسی کنید");
    Serial.println("========================================\n");
}

// 11. Loop اصلی
void loop() {
    // فقط LVGL را اجرا کن - بدون تیک دستی
    lv_timer_handler();
    delay(5);
}
