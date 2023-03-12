#include <Arduino.h>
#include <stdlib.h>
#include <Wire.h>
#include "lv_png.h"

#include <SPI.h>
#include <TFT_eSPI.h> 
#include "FT62XXTouchScreen.h"

TFT_eSPI lcd = TFT_eSPI();
FT62XXTouchScreen touchScreen = FT62XXTouchScreen(TFT_WIDTH, PIN_SDA, PIN_SCL);

#include "lvgl.h"
#include "esp_freertos_hooks.h"

#define SLAVE_ADDRESS 0x08

static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];

lv_disp_drv_t disp_drv;
lv_indev_drv_t indev_drv;

// LVGL Objects
lv_obj_t *settings;
lv_obj_t *settings_label;
lv_obj_t *ph_btn;
lv_obj_t *ec_btn;
lv_obj_t *settings_btn;
lv_obj_t *refresh_btn;
lv_obj_t *settings_cls_btn;

lv_obj_t *screen;
lv_obj_t *body_main;
lv_obj_t *splashscreen_main;
lv_obj_t *body_settings;

lv_obj_t *pH_text;
lv_obj_t *pH_text_target;
lv_obj_t *EC_text;
lv_obj_t *EC_text_target;
lv_obj_t *WL_text;
lv_obj_t *temp_text;

lv_obj_t *phBody;
lv_obj_t *ecBody;
lv_obj_t *tabBody;
lv_obj_t *settingsParam_ph;
lv_obj_t *settingsParam_ec;
lv_obj_t *settingsParam_delay;
lv_obj_t *settingsParam_brightness;
lv_obj_t *tabBody_st;

lv_obj_t *spinbox_ph;
lv_obj_t *spinbox_ec;
lv_obj_t *dropdown_delay;
lv_obj_t *slider_brightness;
lv_obj_t *switch_pumps;

lv_obj_t *pH_text_val;
lv_obj_t *pH_text_target_val;
lv_obj_t *EC_text_val;
lv_obj_t *EC_text_target_val;
lv_obj_t *WL_text_val;
lv_obj_t *temp_text_val;

lv_obj_t *settings_text;
lv_obj_t *settings_ph_text;
lv_obj_t *settings_ec_text;
lv_obj_t *settings_delay_text;
lv_obj_t *settings_brightness_text;

lv_obj_t *setting_window;

lv_obj_t *phChart;
lv_obj_t *ecChart;

lv_obj_t *splashscreen_img;
LV_IMG_DECLARE(icon);

// Values
double total_ph = 0; // To take average value, or use in chart 
double total_ec = 0; // (ie, if chart delay set to 10 mins, chart will update avg val of 10 mins)

struct Threshold
{
    double target_ph, target_ec;
    bool PUMP;
};

static Threshold THRESHOLD{.target_ph = 6.0, .target_ec = 2.0, .PUMP = false};

struct SystemMeasurements
{
    double ph;
    double ec;
    double temp;
    bool wl;
};

static SystemMeasurements system_measurements{.ph = 0.0, .ec = 0.0, .temp = 0.0, .wl = false};

// Chart Y value
int ph_max = 8;
int ph_min = 4;
int ec_max = 4;
int ec_min = 0;
int points = 24; // Chart point amount

// Chart Delay Option
int delay_option = 0; // Delay option
int delay_ms = 1000; // Chart delay in ms
int delay_current = 0; // Current delay count

// Status
bool settings_btn_pressed = false;
bool settings_cls_btn_pressed = false;
int spl_scrn = 0;

// Chart Object
lv_chart_series_t * ph_ser;
lv_chart_series_t * ec_ser;

// Style Object
static lv_style_t screen_st;
static lv_style_t body_st;
static lv_style_t setting_st;
static lv_style_t tab_st;

// Brightness Settings
static const uint8_t backlightChannel = 1;
uint32_t currentBrightness = 96;

// LCD Setup
void disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
bool input_read(lv_indev_drv_t * drv, lv_indev_data_t*data) ;

void update_chart_val(int val, lv_chart_series_t *ser);

// Task Function
void task_update_settings_button_status(lv_task_t *task);
void task_update_settings_cls_button_status(lv_task_t *task);
static void task_check_I2C(lv_task_t *task);
static void task_read_I2C(lv_task_t *task);
static void task_write_I2C(lv_task_t *task);
static void task_update_chart(lv_task_t *task);
static void task_update_values(lv_task_t *task) ;
static void task_rand_value_readings(lv_task_t *task);
static void task_update_brightness(lv_task_t *task);
static void lv_tick_task(void);

//Style Function
static void build_style_mainscreen();

// Build Function
static void build_text_mainscreen();
static void build_screen();
static void build_text_settings();
static void build_chart_mainscreen();
static void build_body_mainscreen();
static void build_buttons_mainscreen();
static void build_widgets_mainscreen();
static void build_buttons_settings();
static void build_body_settings();
static void build_body_splashscreen();

//Text Function
void create_text(lv_obj_t *text, int posx, int posy, const char *word);
void create_val_text(lv_obj_t *text, lv_obj_t *body, int posx, int posy);

//Event Function
static void spinbox_ph_increment_event(lv_obj_t * btn, lv_event_t e);
static void spinbox_ph_decrement_event(lv_obj_t * btn, lv_event_t e);
static void spinbox_ec_increment_event(lv_obj_t * btn, lv_event_t e);
static void spinbox_ec_decrement_event(lv_obj_t * btn, lv_event_t e);
static void slider_change_brightness_event(lv_obj_t * obj, lv_event_t event);
static void dropdown_change_delay_event(lv_obj_t * obj, lv_event_t event);
static void button_ph_change_chart_event(lv_obj_t * btn, lv_event_t e);
static void button_ec_change_chart_event(lv_obj_t * btn, lv_event_t e);
static void button_chart_refresh_event(lv_obj_t * btn, lv_event_t e);

static void task_splash_screen(lv_task_t *task);

//Widget Function
static void build_widgets_settings();

static void initialize();

void setup() {

  initialize();
  lv_png_init();

  build_screen();
  build_style_mainscreen();

  // Build order: Style -> Body -> Buttons -> Text -> Others

  // Mainscreen
  build_body_mainscreen();
  build_buttons_mainscreen();
  build_text_mainscreen();
  build_chart_mainscreen();
  build_widgets_mainscreen();

  // Settings
  build_body_settings();
  build_buttons_settings();
  build_text_settings();
  build_widgets_settings();

  //Splashscreen
  build_body_splashscreen();

  // Tasks
  lv_task_create(task_rand_value_readings, 500, LV_TASK_PRIO_MID, NULL);
  lv_task_create(task_update_chart, 1000, LV_TASK_PRIO_MID, NULL);
  lv_task_create(task_update_values, 500, LV_TASK_PRIO_MID, NULL);
  lv_task_create(task_update_brightness, 100, LV_TASK_PRIO_MID, NULL);
  lv_task_create(task_check_I2C, 500, LV_TASK_PRIO_MID, NULL);
  lv_task_create(task_read_I2C, 500, LV_TASK_PRIO_MID, NULL);
  lv_task_create(task_write_I2C, 500, LV_TASK_PRIO_MID, NULL);
  
  lv_task_t * splash_screen = lv_task_create(task_splash_screen, 3000, LV_TASK_PRIO_MID, NULL);
  lv_task_set_repeat_count(splash_screen, 1);
  lv_task_create(task_update_settings_button_status, 10, LV_TASK_PRIO_HIGH, NULL);
  lv_task_create(task_update_settings_cls_button_status, 10, LV_TASK_PRIO_HIGH, NULL);

  // Screen load
  lv_scr_load(screen);

}

void loop() {

  lv_task_handler();
  delay(5);

}

void disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    lcd.startWrite();
    lcd.setAddrWindow(area->x1, area->y1, w, h);
    lcd.pushColors((uint16_t *)&color_p->full, w * h, true);
    lcd.endWrite();

    lv_disp_flush_ready(disp);
}

uint16_t lastx = 0;
uint16_t lasty = 0;

bool input_read(lv_indev_drv_t * drv, lv_indev_data_t*data) {
  Serial.println("#");
  TouchPoint touchPos = touchScreen.read();
  if (touchPos.touched) {
    Serial.println(String(touchPos.xPos) + ": " + String(touchPos.yPos));
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touchPos.xPos;
    data->point.y = touchPos.yPos;
    lastx = touchPos.xPos;
    lasty = touchPos.yPos;
  } else {
    data->state = LV_INDEV_STATE_REL;
     data->point.x = lastx;
     data->point.y = lasty;
     
  }
  return false;
}

void create_text(lv_obj_t *text, int posx, int posy, const char *word){
  lv_obj_set_size(text, 64, 16);
  lv_obj_set_pos(text, posx, posy);
  lv_obj_set_style_local_text_color(text, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);
  lv_label_set_text(text, word);
  lv_label_set_align(text, LV_LABEL_ALIGN_CENTER);
}

void create_val_text(lv_obj_t *text, lv_obj_t *body, int posx, int posy){
  lv_obj_set_size(text, 64, 16);
  lv_obj_set_pos(text, posx, posy);
  lv_obj_set_style_local_text_color(text, LV_LABEL_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);
  lv_label_set_align(text, LV_LABEL_ALIGN_CENTER);
}

void update_chart_val(int val, lv_chart_series_t *ser){
  for(int i=0; i<points-1; i++){
    ser->points[i] = ser->points[i+1];
  }
  ser->points[points-1] = val;
  
}

void task_check_I2C(lv_task_t *task){
  Wire.beginTransmission(SLAVE_ADDRESS);  // Set the slave address to 0x50
  if (Wire.endTransmission() == 0) {
    Serial.println("I2C connection established with slave!");
  } else {
    Serial.println("Failed to establish I2C connection with slave.");
  }
}

void task_read_I2C(lv_task_t *task){
  if (Wire.requestFrom(SLAVE_ADDRESS, sizeof system_measurements)) {
    Wire.readBytes((byte*) &system_measurements, sizeof system_measurements);

    Serial.println("I2C Read:");
    Serial.println(system_measurements.ph);
    Serial.println(system_measurements.ec);
    Serial.println(system_measurements.temp);
    Serial.println(system_measurements.wl);
  } 
  else {
    Serial.println("could not connect");
  }
}

void task_write_I2C(lv_task_t *task){
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write((uint8_t*)&THRESHOLD, sizeof(THRESHOLD));
  Wire.endTransmission();
}

void task_update_settings_button_status(lv_task_t *task){
  lv_btn_state_t st = lv_btn_get_state(settings_btn);
  if (st == LV_BTN_STATE_PRESSED){
    settings_btn_pressed = true;
  }
  else if (st == LV_BTN_STATE_RELEASED && settings_btn_pressed){
    lv_obj_set_pos(body_main, -4, -(lcd.height() + 8) - 4);
    lv_obj_set_pos(body_settings, -4, -4);
    settings_btn_pressed = false;
  }
}

void task_splash_screen(lv_task_t *task){

  lv_obj_del(splashscreen_main);
}

void task_splashscreen(lv_task_t *task){
  lv_btn_state_t st = lv_btn_get_state(settings_btn);
  if (st == LV_BTN_STATE_PRESSED){
    settings_btn_pressed = true;
  }
  else if (st == LV_BTN_STATE_RELEASED && settings_btn_pressed){
    lv_obj_set_pos(body_main, -4, -(lcd.height() + 8) - 4);
    lv_obj_set_pos(body_settings, -4, -4);
    settings_btn_pressed = false;
  }
}

void task_update_settings_cls_button_status(lv_task_t *task){
  lv_btn_state_t st = lv_btn_get_state(settings_cls_btn);
  if (st == LV_BTN_STATE_PRESSED){
    settings_cls_btn_pressed = true;
  }
  else if (st == LV_BTN_STATE_RELEASED && settings_cls_btn_pressed){
    lv_obj_set_pos(body_main, -4, -4);
    lv_obj_set_pos(body_settings, -4, -(lcd.height() + 8) - 4);
    settings_cls_btn_pressed = false;
  }
  
}

static void task_update_chart(lv_task_t *task){

  delay_current += 1000;
  total_ph += system_measurements.ph;
  total_ec += system_measurements.ec;

  if(delay_current >= delay_ms){

    update_chart_val(total_ph / delay_ms * 10000, ph_ser);
    update_chart_val(total_ec / delay_ms * 10000, ec_ser);

    total_ph = 0;
    total_ec = 0;

    delay_current = 0;
  }
  lv_chart_refresh(phChart);
  lv_chart_refresh(ecChart);
}

static void task_update_values(lv_task_t *task) {

  lv_label_set_text_fmt(pH_text_val, "%.2f" , system_measurements.ph);
  lv_label_set_text_fmt(EC_text_val, "%.2fS" , system_measurements.ec);
  lv_label_set_text_fmt(pH_text_target_val, "%.1f" , THRESHOLD.target_ph);
  lv_label_set_text_fmt(EC_text_target_val, "%.1fS" , THRESHOLD.target_ec);
  lv_label_set_text_fmt(temp_text_val, "%.1f°C" , system_measurements.temp);
  lv_label_set_text(WL_text_val, (system_measurements.wl ? "OK" : "LOW"));
  
}

static void task_rand_value_readings(lv_task_t *task) {
  // Temporary test values
  system_measurements.ph = system_measurements.ph + ((double)(rand() % 2) / 100) - ((double)(rand() % 2) / 100);
  if (system_measurements.ph > THRESHOLD.target_ph + 0.05 && THRESHOLD.PUMP){
    system_measurements.ph = system_measurements.ph - ((double)(rand() % 3) / 100);
  }
  else if (system_measurements.ph < THRESHOLD.target_ph - 0.05 && THRESHOLD.PUMP){
    system_measurements.ph = system_measurements.ph + ((double)(rand() % 3) / 100);
  }

  system_measurements.ec = system_measurements.ec - ((double)(rand() % 2) / 100);
  if (system_measurements.ec < THRESHOLD.target_ec && THRESHOLD.PUMP){
    system_measurements.ec = system_measurements.ec + ((double)(rand() % 4) / 100);
  }
  if (system_measurements.ec < 0){
    system_measurements.ec = 0;
  }

  system_measurements.temp = 60 + ((double)(rand() % 500) / 100);
  
  double wl_threshold = rand() % 100;
  system_measurements.wl = false;
  if (wl_threshold > 10){
    system_measurements.wl = true;
  }
}

static void task_update_brightness(lv_task_t *task) {

  ledcWrite(backlightChannel, currentBrightness);
}

static void lv_tick_task(void)
{
 lv_tick_inc(portTICK_RATE_MS);
}

static void build_screen() {
  // Screen Object
  screen = lv_obj_create(NULL, NULL);
  lv_obj_add_style(screen, 0, &screen_st);
}

static void build_text_mainscreen() {
  // Water Level text
  WL_text = lv_label_create(tabBody, NULL);
  create_text(WL_text, 70, 10, "Water Level Status: ");

  // Current Water Level value text
  WL_text_val = lv_label_create(tabBody, NULL);
  create_val_text(WL_text_val, tabBody, 212, 10);

  // Temperature text
  temp_text = lv_label_create(tabBody, NULL);
  create_text(temp_text, 70, 30, "Temperature: ");

  // Current Temperature value text
  temp_text_val = lv_label_create(tabBody, NULL);
  create_val_text(temp_text_val, tabBody, 178, 30);

  // Current pH Text
  pH_text = lv_label_create(phBody, NULL);
  create_text(pH_text, 8, 8, "Current pH Value: ");

  // Current pH Value Text 
  pH_text_val = lv_label_create(phBody, NULL);
  create_val_text(pH_text_val, phBody, 142, 8);

  // Target pH Text
  pH_text_target = lv_label_create(phBody, NULL);
  create_text(pH_text_target, 224, 8, "Target pH Value: ");

  // Target pH Value Text 
  pH_text_target_val = lv_label_create(phBody, NULL);
  create_val_text(pH_text_target_val, phBody, 352, 8);

  // Current EC Text
  EC_text = lv_label_create(ecBody, NULL);
  create_text(EC_text, 8, 8, "Current EC Value: ");

  // Current EC Value Text
  EC_text_val = lv_label_create(ecBody, NULL);
  create_val_text(EC_text_val, ecBody, 142, 8);

  // Target EC Text
  EC_text_target = lv_label_create(ecBody, NULL);
  create_text(EC_text_target, 224, 8, "Target EC Value: ");
  
  // Target EC Value Text
  EC_text_target_val = lv_label_create(ecBody, NULL);
  create_val_text(EC_text_target_val, ecBody, 352, 8);
}

static void build_text_settings() {
  // Water Level text
  settings_text = lv_label_create(tabBody_st, NULL);
  create_text(settings_text, 20, 20, "Settings");

  // Set pH Level text
  settings_ph_text = lv_label_create(settingsParam_ph, NULL);
  create_text(settings_ph_text, 16, 16, "Set Target pH");

  // Set EC Level text
  settings_ec_text = lv_label_create(settingsParam_ec, NULL);
  create_text(settings_ec_text, 16, 16, "Set Target EC");

  // Set Delay Level text
  settings_delay_text = lv_label_create(settingsParam_delay, NULL);
  create_text(settings_delay_text, 16, 16, "Set Chart Delay");

  // Set Brightness Level text
  settings_brightness_text = lv_label_create(settingsParam_brightness, NULL);
  create_text(settings_brightness_text, 16, 16, "Brightness");
}

static void build_chart_mainscreen() {
  // pH chart
  phChart = lv_chart_create(phBody, NULL);
  lv_obj_set_size(phChart, 444, 80);
  lv_obj_align(phChart, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -4, -4);
  lv_chart_set_type(phChart, LV_CHART_TYPE_COLUMN);

  lv_chart_set_range(phChart, ph_min * 10, ph_max * 10);
  lv_chart_set_y_tick_texts(phChart, "4\n5\n6\n7\n8", 0, LV_CHART_AXIS_DRAW_LAST_TICK | LV_CHART_AXIS_INVERSE_LABELS_ORDER);
  lv_chart_set_y_tick_length(phChart, 4, 0);

  // Stylelizing Chart
  lv_obj_set_style_local_bg_opa(phChart, LV_CHART_PART_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);
  lv_obj_set_style_local_border_opa(phChart, LV_CHART_PART_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);
  lv_chart_set_div_line_count(phChart, ph_max - ph_min - 1, 0);
  lv_obj_set_style_local_pad_left(phChart, LV_CHART_PART_BG, LV_STATE_DEFAULT, 32);
  lv_obj_set_style_local_line_opa(phChart, LV_CHART_PART_SERIES_BG, LV_STATE_DEFAULT, LV_OPA_20);

  // Fade effect 
  lv_obj_set_style_local_bg_opa(phChart, LV_CHART_PART_SERIES, LV_STATE_DEFAULT, LV_OPA_50);
  lv_obj_set_style_local_bg_grad_dir(phChart, LV_CHART_PART_SERIES, LV_STATE_DEFAULT, LV_GRAD_DIR_VER);
  lv_obj_set_style_local_bg_main_stop(phChart, LV_CHART_PART_SERIES, LV_STATE_DEFAULT, 255);   
  lv_obj_set_style_local_bg_grad_stop(phChart, LV_CHART_PART_SERIES, LV_STATE_DEFAULT, 32);     

  lv_chart_set_point_count(phChart, points);
  lv_obj_set_state(phChart, LV_STATE_DISABLED);

  ph_ser = lv_chart_add_series(phChart, LV_COLOR_WHITE);

  for(int i=0; i<points; i++){
    ph_ser->points[i] = ph_min  * 10;
  }

  // EC chart
  ecChart = lv_chart_create(ecBody, NULL);
  lv_obj_set_size(ecChart, 444, 80);
  lv_obj_align(ecChart, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -4, -4);
  lv_chart_set_type(ecChart, LV_CHART_TYPE_COLUMN);

  lv_chart_set_range(ecChart, ec_min, ec_max * 10);
  lv_chart_set_y_tick_texts(ecChart, "0\n1\n2\n3\n4", 0, LV_CHART_AXIS_DRAW_LAST_TICK | LV_CHART_AXIS_INVERSE_LABELS_ORDER);
  lv_chart_set_y_tick_length(ecChart, 4, 0);

  // Stylelizing Chart
  lv_obj_set_style_local_bg_opa(ecChart, LV_CHART_PART_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);
  lv_obj_set_style_local_border_opa(ecChart, LV_CHART_PART_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);
  lv_chart_set_div_line_count(ecChart, ec_max - ec_min - 1, 0);
  lv_obj_set_style_local_pad_left(ecChart, LV_CHART_PART_BG, LV_STATE_DEFAULT, 32);

  // Fade effect 
  lv_obj_set_style_local_bg_opa(ecChart, LV_CHART_PART_SERIES, LV_STATE_DEFAULT, LV_OPA_50);
  lv_obj_set_style_local_bg_grad_dir(ecChart, LV_CHART_PART_SERIES, LV_STATE_DEFAULT, LV_GRAD_DIR_VER);
  lv_obj_set_style_local_bg_main_stop(ecChart, LV_CHART_PART_SERIES, LV_STATE_DEFAULT, 255);   
  lv_obj_set_style_local_bg_grad_stop(ecChart, LV_CHART_PART_SERIES, LV_STATE_DEFAULT, 32);     

  lv_obj_set_style_local_line_opa(ecChart, LV_CHART_PART_SERIES_BG, LV_STATE_DEFAULT, LV_OPA_20);

  lv_chart_set_point_count(ecChart, points);
  lv_obj_set_state(ecChart, LV_STATE_DISABLED);

  ec_ser = lv_chart_add_series(ecChart, LV_COLOR_WHITE);
  lv_chart_refresh(phChart);

  for(int i=0; i<points; i++){
    ec_ser->points[i] = ec_min;
  }
}

static void build_body_mainscreen() {

  // Main body object
  body_main = lv_obj_create(screen, NULL);
  lv_obj_add_style(body_main, 0, &screen_st);
  lv_obj_set_size(body_main, lcd.width() + 8, lcd.height() + 8);
  lv_obj_align(body_main, screen, LV_ALIGN_IN_TOP_LEFT, -4, -4);
  lv_obj_set_state(body_main, LV_STATE_DISABLED);

  // pH Body Object
  phBody = lv_obj_create(body_main, NULL);
  lv_obj_add_style(phBody, 0, &body_st);
  lv_obj_set_size(phBody, lcd.width() - 32, 112);
  lv_obj_align(phBody, body_main, LV_ALIGN_IN_BOTTOM_MID, 0, -144);
  lv_obj_set_state(phBody, LV_STATE_DISABLED);

  // EC Body Object
  ecBody = lv_obj_create(body_main, NULL);
  lv_obj_add_style(ecBody, 0, &body_st);
  lv_obj_set_size(ecBody, lcd.width() - 32, 112);
  lv_obj_align(ecBody, body_main, LV_ALIGN_IN_BOTTOM_MID, 0, -16);
  lv_obj_set_state(ecBody, LV_STATE_DISABLED);

  // Tab Object
  tabBody = lv_obj_create(body_main, NULL);
  lv_obj_add_style(tabBody, 0, &body_st);
  lv_obj_set_size(tabBody, lcd.width(), 56);
  lv_obj_align(tabBody, body_main, LV_ALIGN_IN_TOP_MID, 0, 0);
  lv_obj_set_state(tabBody, LV_STATE_DISABLED);
}

static void build_body_splashscreen() {
  
  // Splashscreen object
  splashscreen_main = lv_obj_create(screen, NULL);
  lv_obj_add_style(splashscreen_main, 0, &screen_st);
  lv_obj_set_size(splashscreen_main, lcd.width() + 8, lcd.height() + 8);
  lv_obj_align(splashscreen_main, screen, LV_ALIGN_IN_TOP_LEFT, -4, -4);
  lv_obj_set_state(splashscreen_main, LV_STATE_DISABLED);

  LV_IMG_DECLARE(icon);
  splashscreen_img = lv_img_create(splashscreen_main, NULL);
  lv_img_set_src(splashscreen_img, &icon);
  lv_obj_align(splashscreen_img, NULL, LV_ALIGN_IN_TOP_LEFT, 4, 4);
}

static void switch_pumps_event(lv_obj_t * btn, lv_event_t e){
if(e == LV_EVENT_VALUE_CHANGED) {
    THRESHOLD.PUMP = !(THRESHOLD.PUMP);
  }
}

static void build_widgets_mainscreen(){
  /*Create a switch and apply the styles*/
  switch_pumps = lv_switch_create(tabBody, NULL);
  lv_obj_set_size(switch_pumps, 64, 36);
  lv_obj_align(switch_pumps, NULL, LV_ALIGN_CENTER, 144, 2);
  lv_obj_set_event_cb(switch_pumps, switch_pumps_event);
  lv_switch_set_anim_time(switch_pumps, 100);
}

static void button_ph_change_chart_event(lv_obj_t * btn, lv_event_t e)
{
  if(e == LV_EVENT_SHORT_CLICKED || e == LV_EVENT_LONG_PRESSED_REPEAT) {
    if (lv_chart_get_type(phChart) == LV_CHART_TYPE_COLUMN){
      lv_chart_set_type(phChart, LV_CHART_TYPE_LINE);
    }
    else {
      lv_chart_set_type(phChart, LV_CHART_TYPE_COLUMN);
    }
  }
}

static void button_ec_change_chart_event(lv_obj_t * btn, lv_event_t e)
{
  if(e == LV_EVENT_SHORT_CLICKED || e == LV_EVENT_LONG_PRESSED_REPEAT) {
    if (lv_chart_get_type(ecChart) == LV_CHART_TYPE_COLUMN){
      lv_chart_set_type(ecChart, LV_CHART_TYPE_LINE);
    }
    else {
      lv_chart_set_type(ecChart, LV_CHART_TYPE_COLUMN);
    }
  }
}

static void button_chart_refresh_event(lv_obj_t * btn, lv_event_t e)
{
  if(e == LV_EVENT_SHORT_CLICKED || e == LV_EVENT_LONG_PRESSED_REPEAT) {
    for(int i=0; i<points; i++){
      ph_ser->points[i] = ph_min  * 10;
    }
    
    for(int i=0; i<points; i++){
      ec_ser->points[i] = ec_min  * 10;
    }
  }

  delay_current = 0;
}

static void build_buttons_mainscreen(){

  // Settings Button 
  settings_btn = lv_btn_create(tabBody, NULL);
  lv_obj_set_size(settings_btn, 40, 40);
  lv_obj_align(settings_btn, NULL, LV_ALIGN_IN_RIGHT_MID, -8, 2);

  lv_obj_t *settings_btn_st = lv_label_create(settings_btn, NULL); 
  lv_label_set_text(settings_btn_st, LV_SYMBOL_SETTINGS); 

  // Settings Button 
  refresh_btn = lv_btn_create(tabBody, NULL);
  lv_obj_set_size(refresh_btn, 40, 40);
  lv_obj_align(refresh_btn, NULL, LV_ALIGN_IN_LEFT_MID, 8, 2);
  lv_obj_set_event_cb(refresh_btn, button_chart_refresh_event);

  lv_obj_t *refresh_btn_st = lv_label_create(refresh_btn, NULL); 
  lv_label_set_text(refresh_btn_st, LV_SYMBOL_REFRESH); 

  // pH Chart Button
  ph_btn = lv_btn_create(phBody, NULL);
  lv_obj_set_size(ph_btn, 32, 16);
  lv_obj_align(ph_btn, NULL, LV_ALIGN_IN_TOP_RIGHT, -8, 8);
  lv_obj_set_ext_click_area(ph_btn, 8, 8, 16, 16);
  lv_obj_set_event_cb(ph_btn, button_ph_change_chart_event);

  lv_obj_t *ph_btn_st = lv_label_create(ph_btn, NULL); 
  lv_label_set_text(ph_btn_st, LV_SYMBOL_LOOP); 

  // EC Chart Button 
  ec_btn = lv_btn_create(ecBody, NULL);
  lv_obj_set_size(ec_btn, 32, 16);
  lv_obj_align(ec_btn, NULL, LV_ALIGN_IN_TOP_RIGHT, -8, 8);
  lv_obj_set_ext_click_area(ec_btn, 8, 8, 16, 16);
  lv_obj_set_event_cb(ec_btn, button_ec_change_chart_event);

  lv_obj_t *ec_btn_st = lv_label_create(ec_btn, NULL); 
  lv_label_set_text(ec_btn_st, LV_SYMBOL_LOOP); 
}

static void build_buttons_settings(){

  // Settings Close Button
  settings_cls_btn = lv_btn_create(tabBody_st, NULL);
  lv_obj_set_size(settings_cls_btn, 40, 40);
  lv_obj_align(settings_cls_btn, NULL, LV_ALIGN_IN_RIGHT_MID, -10, 2);

  lv_obj_t *label_cls_st = lv_label_create(settings_cls_btn, NULL); 
  lv_label_set_text(label_cls_st, LV_SYMBOL_HOME); 
}

static void build_body_settings() {

  // Main body object
  body_settings = lv_obj_create(screen, NULL);
  lv_obj_add_style(body_settings, 0, &screen_st);
  lv_obj_set_size(body_settings, lcd.width() + 8, lcd.height() + 8);
  lv_obj_align(body_settings, screen, LV_ALIGN_IN_TOP_LEFT, -4, -4);
  lv_obj_set_state(body_settings, LV_STATE_DISABLED);
  lv_obj_set_pos(body_settings, 0, -(lcd.height() + 8));

  // Settings parameters Object
  settingsParam_ph = lv_obj_create(body_settings, NULL);
  lv_obj_add_style(settingsParam_ph, 0, &body_st);
  lv_obj_set_size(settingsParam_ph, lcd.width() - 32, 48);
  lv_obj_align(settingsParam_ph, body_settings, LV_ALIGN_IN_BOTTOM_MID, 0, -210);
  lv_obj_set_state(settingsParam_ph, LV_STATE_DISABLED);

  // Settings parameters Object
  settingsParam_ec = lv_obj_create(body_settings, NULL);
  lv_obj_add_style(settingsParam_ec, 0, &body_st);
  lv_obj_set_size(settingsParam_ec, lcd.width() - 32, 48);
  lv_obj_align(settingsParam_ec, body_settings, LV_ALIGN_IN_BOTTOM_MID, 0, -144);
  lv_obj_set_state(settingsParam_ec, LV_STATE_DISABLED);

  // Settings parameters Object
  settingsParam_delay = lv_obj_create(body_settings, NULL);
  lv_obj_add_style(settingsParam_delay, 0, &body_st);
  lv_obj_set_size(settingsParam_delay, lcd.width() - 32, 48);
  lv_obj_align(settingsParam_delay, body_settings, LV_ALIGN_IN_BOTTOM_MID, 0, -80);
  lv_obj_set_state(settingsParam_delay, LV_STATE_DISABLED);

  // Settings parameters Object
  settingsParam_brightness = lv_obj_create(body_settings, NULL);
  lv_obj_add_style(settingsParam_brightness, 0, &body_st);
  lv_obj_set_size(settingsParam_brightness, lcd.width() - 32, 48);
  lv_obj_align(settingsParam_brightness, body_settings, LV_ALIGN_IN_BOTTOM_MID, 0, -16);
  lv_obj_set_state(settingsParam_brightness, LV_STATE_DISABLED);

  // Tab Object
  tabBody_st = lv_obj_create(body_settings, NULL);
  lv_obj_add_style(tabBody_st, 0, &body_st);
  lv_obj_set_size(tabBody_st, lcd.width(), 56);
  lv_obj_align(tabBody_st, body_settings, LV_ALIGN_IN_TOP_MID, 0, 0);
  lv_obj_set_state(tabBody_st, LV_STATE_DISABLED);
}

static void spinbox_ph_increment_event(lv_obj_t * btn, lv_event_t e)
{
    if(e == LV_EVENT_SHORT_CLICKED || e == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_increment(spinbox_ph);
        THRESHOLD.target_ph = ((double)lv_spinbox_get_value(spinbox_ph)) / 10;
    }
}

static void spinbox_ph_decrement_event(lv_obj_t * btn, lv_event_t e)
{
    if(e == LV_EVENT_SHORT_CLICKED || e == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_decrement(spinbox_ph);
        THRESHOLD.target_ph = ((double)lv_spinbox_get_value(spinbox_ph)) / 10;
    }
}

static void spinbox_ec_increment_event(lv_obj_t * btn, lv_event_t e)
{
    if(e == LV_EVENT_SHORT_CLICKED || e == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_increment(spinbox_ec);
        THRESHOLD.target_ec = ((double)lv_spinbox_get_value(spinbox_ec)) / 10;
    }
}

static void spinbox_ec_decrement_event(lv_obj_t * btn, lv_event_t e)
{
    if(e == LV_EVENT_SHORT_CLICKED || e == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_decrement(spinbox_ec);
        THRESHOLD.target_ec = ((double)lv_spinbox_get_value(spinbox_ec)) / 10; 
    }
}

static void slider_change_brightness_event(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_VALUE_CHANGED) {
      currentBrightness = lv_slider_get_value(slider_brightness);
    }
}

static void dropdown_change_delay_event(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_VALUE_CHANGED) {
        delay_option = lv_dropdown_get_selected(obj);
        delay_current = 0;
        total_ph = 0;
        total_ec = 0;

        if (delay_option == 0){
          delay_ms = 1000;
        }
        else if (delay_option == 1){
          delay_ms = 10000;
        }
        else if (delay_option == 2){
          delay_ms = 60000;
        }
        else if (delay_option == 3){
          delay_ms = 600000;
        }
        else if (delay_option == 4){
          delay_ms = 1800000;
        }
        else if (delay_option == 5){
          delay_ms = 3600000;
        }
        else {
          delay_ms = 7200000;
        }
    }
}

static void build_widgets_settings(){
  // Spinbox for pH
  spinbox_ph = lv_spinbox_create(settingsParam_ph, NULL);
  lv_spinbox_set_range(spinbox_ph, ph_min * 10, ph_max * 10);
  lv_spinbox_set_value(spinbox_ph, THRESHOLD.target_ph*10);
  lv_spinbox_set_digit_format(spinbox_ph, 2, 1);
  lv_spinbox_step_prev(spinbox_ph);
  lv_obj_set_width(spinbox_ph, 40);
  lv_obj_align(spinbox_ph, NULL, LV_ALIGN_IN_RIGHT_MID, -48, 0);

  lv_coord_t h = lv_obj_get_height(spinbox_ph);

  lv_obj_t * btn_ph = lv_btn_create(settingsParam_ph, NULL);
  lv_obj_set_size(btn_ph, h, h);
  lv_obj_align(btn_ph, spinbox_ph, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_theme_apply(btn_ph, LV_THEME_SPINBOX_BTN);
  lv_obj_set_style_local_value_str(btn_ph, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_SYMBOL_PLUS);
  lv_obj_set_event_cb(btn_ph, spinbox_ph_increment_event);

  btn_ph = lv_btn_create(settingsParam_ph, btn_ph);
  lv_obj_align(btn_ph, spinbox_ph, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_event_cb(btn_ph, spinbox_ph_decrement_event);
  lv_obj_set_style_local_value_str(btn_ph, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_SYMBOL_MINUS);

  // Spinbox for EC
  spinbox_ec = lv_spinbox_create(settingsParam_ec, NULL);
  lv_spinbox_set_range(spinbox_ec, ec_min * 10, ec_max * 10);
  lv_spinbox_set_value(spinbox_ec, THRESHOLD.target_ec*10);
  lv_spinbox_set_digit_format(spinbox_ec, 2, 1);
  lv_spinbox_step_prev(spinbox_ec);
  lv_obj_set_width(spinbox_ec, 40);
  lv_obj_align(spinbox_ec, NULL, LV_ALIGN_IN_RIGHT_MID, -48, 0);

  lv_obj_t * btn_ec = lv_btn_create(settingsParam_ec, NULL);
  lv_obj_set_size(btn_ec, h, h);
  lv_obj_align(btn_ec, spinbox_ec, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_theme_apply(btn_ec, LV_THEME_SPINBOX_BTN);
  lv_obj_set_style_local_value_str(btn_ec, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_SYMBOL_PLUS);
  lv_obj_set_event_cb(btn_ec, spinbox_ec_increment_event);

  btn_ec = lv_btn_create(settingsParam_ec, btn_ec);
  lv_obj_align(btn_ec, spinbox_ec, LV_ALIGN_OUT_LEFT_MID, -5, 0);
  lv_obj_set_event_cb(btn_ec, spinbox_ec_decrement_event);
  lv_obj_set_style_local_value_str(btn_ec, LV_BTN_PART_MAIN, LV_STATE_DEFAULT, LV_SYMBOL_MINUS);

  // Drop down list for chart delay
  lv_obj_t * dropdown_delay = lv_dropdown_create(settingsParam_delay, NULL);
  lv_dropdown_set_options(dropdown_delay,
          "1 second\n"
          "10 seconds\n"
          "1 minute\n"
          "10 minutes\n"
          "30 minutes\n"
          "1 hour\n"
          "2 hours");

  lv_dropdown_set_dir(dropdown_delay, LV_DROPDOWN_DIR_LEFT);
  lv_dropdown_set_symbol(dropdown_delay, NULL);
  lv_dropdown_set_selected(dropdown_delay, delay_option);
  lv_obj_align(dropdown_delay, NULL, LV_ALIGN_IN_RIGHT_MID, -16, 0);
  lv_obj_set_ext_click_area(dropdown_delay, 8,8,8,8);
  lv_obj_set_event_cb(dropdown_delay, dropdown_change_delay_event);
  
  // Slider for Brightness
  slider_brightness = lv_slider_create(settingsParam_brightness, NULL);
  lv_obj_align(slider_brightness, NULL, LV_ALIGN_IN_RIGHT_MID, -16, 0);
  lv_slider_set_range(slider_brightness, 4, 256);
  lv_slider_set_value(slider_brightness, currentBrightness, LV_ANIM_ON);
  lv_obj_set_event_cb(slider_brightness, slider_change_brightness_event);
  lv_obj_set_ext_click_area(slider_brightness, 4, 4, 16, 16);
}

static void build_style_mainscreen() {
  // Screen style
  lv_style_init(&screen_st);
  lv_style_set_bg_color(&screen_st, LV_STATE_DEFAULT, lv_color_hex(0x1A1A2C));
  lv_style_set_border_width(&screen_st, LV_STATE_DEFAULT, 4);
  lv_style_set_border_color(&screen_st, LV_STATE_DEFAULT, lv_color_hex(0x1A1A2C));
  //lv_style_set_text_font(&screen_st, LV_STATE_DEFAULT, LV_FONT_MONTSERRAT_28);
  
  // Settings style
  lv_style_init(&setting_st);
  lv_style_set_border_width(&setting_st, LV_STATE_DEFAULT, 4);
  lv_style_set_border_color(&setting_st, LV_STATE_DEFAULT, lv_color_hex(0xEEEEEE));
  lv_style_set_bg_color(&setting_st, LV_STATE_DEFAULT, lv_color_hex(0xEEEEEE));
  //lv_style_set_text_font(&setting_st, LV_STATE_DEFAULT, LV_FONT_MONTSERRAT_28);

  // Body style
  lv_style_init(&body_st);
  lv_style_set_border_width(&body_st, LV_STATE_DEFAULT, 4);
  lv_style_set_border_color(&body_st, LV_STATE_DEFAULT, lv_color_hex(0x26264C));
  lv_style_set_bg_color(&body_st, LV_STATE_DEFAULT, lv_color_hex(0x26264C));
  //lv_style_set_text_font(&body_st, LV_STATE_DEFAULT, LV_FONT_MONTSERRAT_28);
}

static void initialize() {
  Wire.begin(18, 19);
  Serial.begin(115200);
  lv_init();
  // Setup tick hook for lv_tick_task
  esp_err_t err = esp_register_freertos_tick_hook((esp_freertos_tick_cb_t)lv_tick_task); 

  // Enable TFT
  lcd.begin();
  lcd.setRotation(1);

  // Enable Backlight
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL,1);
  ledcSetup(backlightChannel, 100, 8);
  ledcAttachPin(TFT_BL, backlightChannel);
  ledcWrite(backlightChannel, currentBrightness);

  // Start TouchScreen
  touchScreen.begin();

  // Display Buffer
  lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);

  // Init Display
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 480;
  disp_drv.ver_res = 320;
  disp_drv.flush_cb = disp_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);

  // Init Touchscreen
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = input_read;
  lv_indev_drv_register(&indev_drv);
}
