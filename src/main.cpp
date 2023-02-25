#include <Arduino.h>
#include <stdlib.h>
#include <Wire.h>

#include <SPI.h>
#include <TFT_eSPI.h> 
#include "FT62XXTouchScreen.h"

TFT_eSPI lcd = TFT_eSPI();
FT62XXTouchScreen touchScreen = FT62XXTouchScreen(TFT_WIDTH, PIN_SDA, PIN_SCL);

#include "lvgl.h"
#include "esp_freertos_hooks.h"

static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];

lv_disp_drv_t disp_drv;
lv_indev_drv_t indev_drv;

// LVGL Objects
lv_obj_t *settings;
lv_obj_t *settings_label;
lv_obj_t *settings_btn;
lv_obj_t *settings_cls_btn;

lv_obj_t *screen;
lv_obj_t *body_main;
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

// Values
double ph = 0;
double ec = 0;
double target_ph = 6;
double target_ec = 5;
bool wl = false;
double temp = 0;

// Chart Y value
int ph_max = 7;
int ph_min = 5;
int ec_max = 10;
int ec_min = 0;

// Chart Delay Option
int delay_option = 2;
int delay_ms = 1000;
int delay_current = 0;

// Button Status
bool settings_btn_pressed = false;
bool settings_cls_btn_pressed = false;

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
static void task_update_chart(lv_task_t *task);
static void task_update_values(lv_task_t *task) ;
static void task_value_readings(lv_task_t *task);
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
static void build_buttons_settings();
static void build_body_settings();

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

//Widget Function
static void build_widgets_settings();

static void initialize();

void setup() {

  initialize();

  build_screen();

  // Build order: Style -> Body -> Buttons -> Text -> Others

  // Mainscreen
  build_style_mainscreen();
  build_body_mainscreen();
  build_buttons_mainscreen();
  build_text_mainscreen();
  build_chart_mainscreen();

  // Settings
  build_body_settings();
  build_buttons_settings();
  build_text_settings();
  build_widgets_settings();

  // Tasks
  lv_task_create(task_value_readings, 100, LV_TASK_PRIO_MID, NULL);
  lv_task_create(task_update_chart, 100, LV_TASK_PRIO_MID, NULL);
  lv_task_create(task_update_values, 100, LV_TASK_PRIO_MID, NULL);
  lv_task_create(task_update_brightness, 100, LV_TASK_PRIO_MID, NULL);

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
  for(int i=0; i<19; i++){
    ser->points[i] = ser->points[i+1];
  }
  ser->points[19] = val;
  
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

  delay_current += 100;

  if(delay_current > delay_ms){
    update_chart_val(ph * 10, ph_ser);
    update_chart_val(ec * 10, ec_ser);

    delay_current = 0;
  }
  lv_chart_refresh(phChart);
  lv_chart_refresh(ecChart);
}

static void task_update_values(lv_task_t *task) {

  lv_label_set_text_fmt(pH_text_val, "%.2f" , ph);
  lv_label_set_text_fmt(EC_text_val, "%.2f" , ec);
  lv_label_set_text_fmt(pH_text_target_val, "%.1f" , target_ph);
  lv_label_set_text_fmt(EC_text_target_val, "%.1f" , target_ec);
  lv_label_set_text_fmt(temp_text_val, "%.1f" , temp);

  if (wl){
    lv_label_set_text(WL_text_val, "OK");
  }
  else {
    lv_label_set_text(WL_text_val, "LOW");
  }
  
}

static void task_value_readings(lv_task_t *task) {

  ph = 5 + ((double)(rand() % 200) / 100);
  ec = ((double)(rand() % 1000) / 100);
  temp = 60 + ((double)(rand() % 500) / 100);
  
  double wl_threshold = rand() % 100;
  wl = false;
  if (wl_threshold > 10){
    wl = true;
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
  create_text(WL_text, 22, 20, "Water Level Status: ");

  // Current Water Level value text
  WL_text_val = lv_label_create(tabBody, NULL);
  create_val_text(WL_text_val, tabBody, 164, 20);

  // Temperature text
  temp_text = lv_label_create(tabBody, NULL);
  create_text(temp_text, 240, 20, "Temperature: ");

  // Current Temperature value text
  temp_text_val = lv_label_create(tabBody, NULL);
  create_val_text(temp_text_val, tabBody, 352, 20);

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
  lv_obj_set_size(phChart, 448, 80);
  lv_obj_align(phChart, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -4);
  lv_chart_set_type(phChart, LV_CHART_TYPE_COLUMN);
  lv_chart_set_range(phChart, 50, 70);
  lv_obj_set_style_local_bg_opa(phChart, LV_CHART_PART_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);
  lv_obj_set_style_local_border_opa(phChart, LV_CHART_PART_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);
  lv_obj_set_style_local_line_opa(phChart, LV_CHART_PART_SERIES_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);
  lv_chart_set_point_count(phChart, 20);
  lv_obj_set_state(phChart, LV_STATE_DISABLED);

  ph_ser = lv_chart_add_series(phChart, LV_COLOR_WHITE);

  for(int i=0; i<20; i++){
    ph_ser->points[i] = 6;
  }

  // EC chart
  ecChart = lv_chart_create(ecBody, NULL);
  lv_obj_set_size(ecChart, 448, 80);
  lv_obj_align(ecChart, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -4);
  lv_chart_set_type(ecChart, LV_CHART_TYPE_COLUMN);
  lv_chart_set_range(ecChart, 0, 100);
  lv_obj_set_style_local_bg_opa(ecChart, LV_CHART_PART_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);
  lv_obj_set_style_local_border_opa(ecChart, LV_CHART_PART_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);
  lv_obj_set_style_local_line_opa(ecChart, LV_CHART_PART_SERIES_BG, LV_STATE_DEFAULT, LV_OPA_TRANSP);
  lv_chart_set_point_count(ecChart, 20);
  lv_obj_set_state(ecChart, LV_STATE_DISABLED);

  ec_ser = lv_chart_add_series(ecChart, LV_COLOR_WHITE);
  lv_chart_refresh(phChart);

  for(int i=0; i<20; i++){
    ec_ser->points[i] = 6;
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

static void build_buttons_mainscreen(){

  // Settings Button 
  settings_btn = lv_btn_create(tabBody, NULL);
  lv_obj_set_size(settings_btn, 40, 40);
  lv_obj_align(settings_btn, NULL, LV_ALIGN_IN_RIGHT_MID, -10, 2);

  lv_obj_t *label_st = lv_label_create(settings_btn, NULL); 
  lv_label_set_text(label_st, LV_SYMBOL_SETTINGS); 
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
        target_ph = ((double)lv_spinbox_get_value(spinbox_ph)) / 10;
    }
}

static void spinbox_ph_decrement_event(lv_obj_t * btn, lv_event_t e)
{
    if(e == LV_EVENT_SHORT_CLICKED || e == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_decrement(spinbox_ph);
        target_ph = ((double)lv_spinbox_get_value(spinbox_ph)) / 10;
    }
}

static void spinbox_ec_increment_event(lv_obj_t * btn, lv_event_t e)
{
    if(e == LV_EVENT_SHORT_CLICKED || e == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_increment(spinbox_ec);
        target_ec = ((double)lv_spinbox_get_value(spinbox_ec)) / 10;
    }
}

static void spinbox_ec_decrement_event(lv_obj_t * btn, lv_event_t e)
{
    if(e == LV_EVENT_SHORT_CLICKED || e == LV_EVENT_LONG_PRESSED_REPEAT) {
        lv_spinbox_decrement(spinbox_ec);
        target_ec = ((double)lv_spinbox_get_value(spinbox_ec)) / 10; 
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

        if (delay_option == 0){
          delay_ms = 100;
        }
        else if (delay_option == 1){
          delay_ms = 1000;
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
        else {
          delay_ms = 3600000;
        }
    }
}

static void build_widgets_settings(){
  // Spinbox for pH
  spinbox_ph = lv_spinbox_create(settingsParam_ph, NULL);
  lv_spinbox_set_range(spinbox_ph, 50, 70);
  lv_spinbox_set_value(spinbox_ph, target_ph*10);
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
  lv_spinbox_set_range(spinbox_ec, 0, 99);
  lv_spinbox_set_value(spinbox_ec, target_ec*10);
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
  lv_dropdown_set_options(dropdown_delay, "0.1 seconds\n"
          "1 second\n"
          "1 minute\n"
          "10 minutes\n"
          "30 minutes\n"
          "1 hour");

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

