#pragma once

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern "C" {
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
}

// ---------- Defaults ----------
#ifndef OLED_PORT
#define OLED_PORT I2C_NUM_0
#endif
#ifndef OLED_ADDR
#define OLED_ADDR 0x3C
#endif
#ifndef OLED_SDA
#define OLED_SDA 21
#endif
#ifndef OLED_SCL
#define OLED_SCL 22
#endif
#ifndef OLED_FREQ
#define OLED_FREQ 100000
#endif
#ifndef OLED_RST_GPIO
#define OLED_RST_GPIO 16
#endif

#define OLED_W 128
#define OLED_H 64
#define OLED_PAGES (OLED_H/8)

// ---------- State ----------
static uint8_t _oled_col = 0;
static uint8_t _oled_page = 0;
static bool _oled_alive = true;

static inline void _oled_delay_ms(int ms){
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// ---------- GPIO ----------
static inline void _oled_gpio_out(int pin,int lvl){
    gpio_set_direction((gpio_num_t)pin,GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)pin,lvl);
}

static inline void _oled_reset_if_needed(){
    if(OLED_RST_GPIO<0) return;

    _oled_gpio_out(OLED_RST_GPIO,1);
    _oled_delay_ms(2);
    _oled_gpio_out(OLED_RST_GPIO,0);
    _oled_delay_ms(50);
    _oled_gpio_out(OLED_RST_GPIO,1);
    _oled_delay_ms(10);
}

// ---------- I2C ----------
static inline esp_err_t _oled_i2c_write(uint8_t ctrl,const uint8_t* data,size_t n)
{
    if(!_oled_alive) return ESP_FAIL;

    i2c_cmd_handle_t cmd=i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,(OLED_ADDR<<1)|I2C_MASTER_WRITE,true);
    i2c_master_write_byte(cmd,ctrl,true);
    if(n && data) i2c_master_write(cmd,(uint8_t*)data,n,true);
    i2c_master_stop(cmd);

    esp_err_t e=i2c_master_cmd_begin(OLED_PORT,cmd,pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(cmd);

    if(e!=ESP_OK) _oled_alive=false;
    return e;
}

static inline esp_err_t _oled_cmd(uint8_t c){
    return _oled_i2c_write(0x00,&c,1);
}

static inline esp_err_t _oled_cmds(const uint8_t* c,size_t n){
    return _oled_i2c_write(0x00,c,n);
}

// ---------- FONT ----------
#include "font6x8.inc"

// ---------- INIT ----------
static inline esp_err_t oled_tty_init(void)
{
    _oled_alive=true;

    i2c_config_t conf{};
    conf.mode=I2C_MODE_MASTER;
    conf.sda_io_num=(gpio_num_t)OLED_SDA;
    conf.scl_io_num=(gpio_num_t)OLED_SCL;
    conf.sda_pullup_en=GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en=GPIO_PULLUP_ENABLE;
    conf.master.clk_speed=OLED_FREQ;

    esp_err_t e=i2c_param_config(OLED_PORT,&conf);
    if(e!=ESP_OK) return e;

    e=i2c_driver_install(OLED_PORT,conf.mode,0,0,0);
    if(e!=ESP_OK && e!=ESP_ERR_INVALID_STATE) return e;

    _oled_reset_if_needed();

    const uint8_t init_cmds[]={
        0xAE,
        0xD5,0x80,
        0xA8,0x3F,
        0xD3,0x00,
        0x40,
        0x8D,0x14,
        0x20,0x02,   // PAGE MODE
        0xA1,
        0xC8,
        0xDA,0x12,
        0x81,0xFF,
        0xD9,0xF1,
        0xDB,0x40,
        0xA4,
        0xA6,
        0xAF
    };

    e=_oled_cmds(init_cmds,sizeof(init_cmds));
    if(e!=ESP_OK){
        _oled_alive=false;
        return e;
    }

    return ESP_OK;
}

// ---------- CURSOR ----------
static inline void oled_tty_cursor(uint8_t col,uint8_t page)
{
    if(!_oled_alive) return;

    _oled_col=col;
    _oled_page=page;

    _oled_cmd(0xB0 | (_oled_page&0x0F));
    _oled_cmd(0x00 | (_oled_col&0x0F));
    _oled_cmd(0x10 | ((_oled_col>>4)&0x0F));
}

// ---------- CLEAR ----------
static inline void oled_tty_clear(void)
{
    if(!_oled_alive) return;

    uint8_t z[16]={0};

    for(uint8_t p=0;p<OLED_PAGES;p++){
        oled_tty_cursor(0,p);
        for(int i=0;i<OLED_W;i+=16){
            if(_oled_i2c_write(0x40,z,16)!=ESP_OK) return;
        }
    }
}

// ---------- PRINT ----------
static inline void oled_tty_putc(char c)
{
    if(!_oled_alive) return;

    if(c=='\n'){
        _oled_col=0;
        _oled_page=(_oled_page+1)%OLED_PAGES;
        oled_tty_cursor(_oled_col,_oled_page);
        return;
    }

    if(c<32||c>127) c='?';

    if(_oled_col+6>=OLED_W){
        _oled_col=0;
        _oled_page=(_oled_page+1)%OLED_PAGES;
        oled_tty_cursor(_oled_col,_oled_page);
    }

    if(_oled_i2c_write(0x40,_f6x8[(uint8_t)c-32],6)!=ESP_OK) return;
    _oled_col+=6;
}

static inline void oled_tty_print(const char* s){
    while(s && *s) oled_tty_putc(*s++);
}

static inline void oled_tty_printf(const char* fmt,...){
    char b[128];
    va_list ap;
    va_start(ap,fmt);
    vsnprintf(b,sizeof(b),fmt,ap);
    va_end(ap);
    oled_tty_print(b);
}

// ---------- HELPERS ----------
static inline void oled_tty_clear_line(uint8_t page){
    if(!_oled_alive) return;
    oled_tty_cursor(0,page);

    uint8_t z[16]={0};
    for(int i=0;i<OLED_W;i+=16){
        if(_oled_i2c_write(0x40,z,16)!=ESP_OK) return;
    }
}

static inline void oled_tty_write_line(uint8_t page,const char* s){
    oled_tty_clear_line(page);
    oled_tty_cursor(0,page);
    oled_tty_print(s);
}