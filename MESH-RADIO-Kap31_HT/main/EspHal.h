#pragma once

#include <stdint.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"

#include <RadioLib.h>

#ifndef INPUT
  #define INPUT  0
#endif
#ifndef OUTPUT
  #define OUTPUT 1
#endif
#ifndef LOW
  #define LOW  0
#endif
#ifndef HIGH
  #define HIGH 1
#endif
#ifndef RISING
  #define RISING  2
#endif
#ifndef FALLING
  #define FALLING 3
#endif

class EspHal : public RadioLibHal {
public:
  EspHal() : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING) {}

  void setSpiPins(int sck, int miso, int mosi) { _sck = sck; _miso = miso; _mosi = mosi; }
  void setSpiHost(spi_host_device_t host) { _host = host; }
  void setSpiHz(int hz) { _hz = hz; }

  void init() override {}
  void term() override {}

  void pinMode(uint32_t pin, uint32_t mode) override {
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << pin;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.mode = (mode == OUTPUT) ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
    ESP_ERROR_CHECK(gpio_config(&io));
  }

  void digitalWrite(uint32_t pin, uint32_t value) override {
    gpio_set_level((gpio_num_t)pin, value ? 1 : 0);
  }

  uint32_t digitalRead(uint32_t pin) override {
    return (uint32_t)gpio_get_level((gpio_num_t)pin);
  }

  // --- Interrupts (für SX1262 TX/RX done wichtig) ---
  void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override {
    _irqPin = (int)interruptNum;
    _irqCb  = interruptCb;

    ESP_LOGI("EspHal", "attachInterrupt pin=%d mode=%lu", _irqPin, (unsigned long)mode);

    pinMode(_irqPin, INPUT);

    gpio_int_type_t it = GPIO_INTR_POSEDGE;
    if(mode == FALLING) it = GPIO_INTR_NEGEDGE;
    ESP_ERROR_CHECK(gpio_set_intr_type((gpio_num_t)_irqPin, it));

    // ISR-Service installieren: wenn schon da -> OK
    esp_err_t e = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if(e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
      ESP_ERROR_CHECK(e);
    }

    // alten Handler weg, neuen dran
    gpio_isr_handler_remove((gpio_num_t)_irqPin);
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)_irqPin, &EspHal::gpio_isr_trampoline, this));
    ESP_ERROR_CHECK(gpio_intr_enable((gpio_num_t)_irqPin));
  }

  void detachInterrupt(uint32_t interruptNum) override {
    (void)interruptNum;
    if(_irqPin >= 0) {
      gpio_isr_handler_remove((gpio_num_t)_irqPin);
      gpio_intr_disable((gpio_num_t)_irqPin);
    }
    _irqPin = -1;
    _irqCb = nullptr;
  }

  void delay(RadioLibTime_t ms) override {
    vTaskDelay(pdMS_TO_TICKS((uint32_t)ms));
  }

  void delayMicroseconds(RadioLibTime_t us) override {
    int64_t start = esp_timer_get_time();
    while((esp_timer_get_time() - start) < (int64_t)us) {}
  }

  RadioLibTime_t millis() override {
    return (RadioLibTime_t)(esp_timer_get_time() / 1000ULL);
  }

  RadioLibTime_t micros() override {
    return (RadioLibTime_t)(esp_timer_get_time());
  }

  long pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout) override {
    (void)pin; (void)state; (void)timeout;
    return 0;
  }

  // --- SPI ---
  void spiBegin() override {
    if(_spiInited) return;

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = _mosi;
    buscfg.miso_io_num = _miso;
    buscfg.sclk_io_num = _sck;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    ESP_ERROR_CHECK(spi_bus_initialize(_host, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = _hz;
    devcfg.mode = 0;
    devcfg.spics_io_num = -1;   // CS macht RadioLib per GPIO
    devcfg.queue_size = 1;

    ESP_ERROR_CHECK(spi_bus_add_device(_host, &devcfg, &_dev));
    _spiInited = true;
  }

  void spiBeginTransaction() override {}
  void spiEndTransaction() override {}
  void spiEnd() override {}

  void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override {
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = out;
    t.rx_buffer = in;
    ESP_ERROR_CHECK(spi_device_transmit(_dev, &t));
  }

private:
  static void IRAM_ATTR gpio_isr_trampoline(void* arg) {
    EspHal* self = static_cast<EspHal*>(arg);
    if(self && self->_irqCb) self->_irqCb();
  }

  int _irqPin = -1;
  void (*_irqCb)(void) = nullptr;

  bool _spiInited = false;
  spi_host_device_t _host = SPI2_HOST;
  spi_device_handle_t _dev = nullptr;

  int _sck  = -1;
  int _miso = -1;
  int _mosi = -1;
  int _hz   = 1 * 1000 * 1000; // <- bewusst erstmal 1 MHz (stabil), später hochdrehen
};