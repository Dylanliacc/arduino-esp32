// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "esp32-hal-gpio.h"
#include "esp32-hal-periman.h"
#include "hal/gpio_hal.h"
#include "soc/soc_caps.h"
#include "PCA95x5/PCA95x5.h"  //added for PCA9535
#include "Wire/src/Wire.h"

// redefine virtual PIN from num 100
#define PCA9535_BASE_PIN  100
#define PCA9535_PIN_COUNT 16

// init PCA9535
void setupPCA9535() {
  Wire.begin();
  pca9535.attach(Wire, 0x20);
}

// RGB_BUILTIN is defined in pins_arduino.h
// If RGB_BUILTIN is defined, it will be used as a pin number for the RGB LED
// If RGB_BUILTIN has a side effect that prevents using RMT Legacy driver in IDF 5.1
// Define ESP32_ARDUINO_NO_RGB_BUILTIN in build_opt.h or through CLI to disable RGB_BUILTIN
#ifdef ESP32_ARDUINO_NO_RGB_BUILTIN
#ifdef RGB_BUILTIN
#undef RGB_BUILTIN
#endif
#endif

// It fixes lack of pin definition for S3 and for any future SoC
// this function works for ESP32, ESP32-S2 and ESP32-S3 - including the C3, it will return -1 for any pin
#if SOC_TOUCH_SENSOR_NUM > 0
#include "soc/touch_sensor_periph.h"

int8_t digitalPinToTouchChannel(uint8_t pin) {
  int8_t ret = -1;
  if (pin < SOC_GPIO_PIN_COUNT) {
    for (uint8_t i = 0; i < SOC_TOUCH_SENSOR_NUM; i++) {
      if (touch_sensor_channel_io_map[i] == pin) {
        ret = i;
        break;
      }
    }
  }
  return ret;
}
#else
// No Touch Sensor available
int8_t digitalPinToTouchChannel(uint8_t pin) {
  return -1;
}
#endif

#ifdef SOC_ADC_SUPPORTED
#include "soc/adc_periph.h"

int8_t digitalPinToAnalogChannel(uint8_t pin) {
  uint8_t channel = 0;
  if (pin < SOC_GPIO_PIN_COUNT) {
    for (uint8_t i = 0; i < SOC_ADC_PERIPH_NUM; i++) {
      for (uint8_t j = 0; j < SOC_ADC_MAX_CHANNEL_NUM; j++) {
        if (adc_channel_io_map[i][j] == pin) {
          return channel;
        }
        channel++;
      }
    }
  }
  return -1;
}

int8_t analogChannelToDigitalPin(uint8_t channel) {
  if (channel >= (SOC_ADC_PERIPH_NUM * SOC_ADC_MAX_CHANNEL_NUM)) {
    return -1;
  }
  uint8_t adc_unit = (channel / SOC_ADC_MAX_CHANNEL_NUM);
  uint8_t adc_chan = (channel % SOC_ADC_MAX_CHANNEL_NUM);
  return adc_channel_io_map[adc_unit][adc_chan];
}
#else
// No Analog channels available
int8_t analogChannelToDigitalPin(uint8_t channel) {
  return -1;
}
#endif

typedef void (*voidFuncPtr)(void);
typedef void (*voidFuncPtrArg)(void *);
typedef struct {
  voidFuncPtr fn;
  void *arg;
  bool functional;
} InterruptHandle_t;
static InterruptHandle_t __pinInterruptHandlers[SOC_GPIO_PIN_COUNT] = {
  0,
};

#include "driver/rtc_io.h"

static bool gpioDetachBus(void *bus) {
  return true;
}

extern void ARDUINO_ISR_ATTR __pinMode(uint8_t pin, uint8_t mode) {  // fit TCA9535
  if (pin >= PCA9535_BASE_PIN && pin < PCA9535_BASE_PIN + PCA9535_PIN_COUNT) {
    uint8_t pcaPin = pin - PCA9535_BASE_PIN;
    if (mode == INPUT) {
      pca9535.direction(pcaPin, PCA95x5::Direction::IN);
    } else {
      pca9535.direction(pcaPin, PCA95x5::Direction::OUT);
    }
  } else {
    if (pin >= SOC_GPIO_PIN_COUNT) {
      log_e("Invalid IO %i selected", pin);
      return;
    }

    gpio_hal_context_t gpiohal;
    gpiohal.dev = GPIO_LL_GET_HW(GPIO_PORT_0);

    gpio_config_t conf = {
      .pin_bit_mask = (1ULL << pin),
      .mode = GPIO_MODE_DISABLE,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = gpiohal.dev->pin[pin].int_type
    };

    if (mode < 0x20) {
      conf.mode = mode & (INPUT | OUTPUT);
      if (mode & OPEN_DRAIN) {
        conf.mode |= GPIO_MODE_DEF_OD;
      }
      if (mode & PULLUP) {
        conf.pull_up_en = GPIO_PULLUP_ENABLE;
      }
      if (mode & PULLDOWN) {
        conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
      }
    }
    if (gpio_config(&conf) != ESP_OK) {
      log_e("IO %i config failed", pin);
      return;
    }
  }
}

#ifdef RGB_BUILTIN
uint8_t RGB_BUILTIN_storage = 0;
#endif

extern void ARDUINO_ISR_ATTR __digitalWrite(uint8_t pin, uint8_t val) {  // fit TCA9535
  if (pin >= PCA9535_BASE_PIN && pin < PCA9535_BASE_PIN + PCA9535_PIN_COUNT) {
    uint8_t pcaPin = pin - PCA9535_BASE_PIN;
    pca9535.write(pcaPin, val == HIGH ? PCA95x5::Level::H : PCA95x5::Level::L);
  } else {
    // use basic function
    if (perimanGetPinBus(pin, ESP32_BUS_TYPE_GPIO) != NULL) {
      gpio_set_level((gpio_num_t)pin, val);
    } else {
      log_e("IO %i is not set as GPIO.", pin);
    }
  }
}

extern int __digitalRead(uint8_t pin) {  // fit TCA9535
  if (pin >= PCA9535_BASE_PIN && pin < PCA9535_BASE_PIN + PCA9535_PIN_COUNT) {
    uint8_t pcaPin = pin - PCA9535_BASE_PIN;
    return pca9535.read(pcaPin) == PCA95x5::Level::H ? HIGH : LOW;
  } else {
    // 否则调用标准的 ESP32 GPIO 读取函数
    if (perimanGetPinBus(pin, ESP32_BUS_TYPE_GPIO) != NULL) {
      return gpio_get_level((gpio_num_t)pin);
    } else {
      log_e("IO %i is not set as GPIO.", pin);
      return 0;
    }
  }
}

static void ARDUINO_ISR_ATTR __onPinInterrupt(void *arg) {
  InterruptHandle_t *isr = (InterruptHandle_t *)arg;
  if (isr->fn) {
    if (isr->arg) {
      ((voidFuncPtrArg)isr->fn)(isr->arg);
    } else {
      isr->fn();
    }
  }
}

extern void cleanupFunctional(void *arg);

extern void __attachInterruptFunctionalArg(uint8_t pin, voidFuncPtrArg userFunc, void *arg, int intr_type, bool functional) {  // fit TCA9535
  if (pin >= PCA9535_BASE_PIN && pin < PCA9535_BASE_PIN + PCA9535_PIN_COUNT) {
    // PCA9535  do not support interrupt
    log_e("PCA9535 pin %i does not support interrupts", pin);
  } else {
    static bool interrupt_initialized = false;

    // makes sure that pin -1 (255) will never work -- this follows Arduino standard
    if (pin >= SOC_GPIO_PIN_COUNT) {
      return;
    }

    if (!interrupt_initialized) {
      esp_err_t err = gpio_install_isr_service((int)ARDUINO_ISR_FLAG);
      interrupt_initialized = (err == ESP_OK) || (err == ESP_ERR_INVALID_STATE);
    }
    if (!interrupt_initialized) {
      log_e("IO %i ISR Service Failed To Start", pin);
      return;
    }

    // if new attach without detach remove old info
    if (__pinInterruptHandlers[pin].functional && __pinInterruptHandlers[pin].arg) {
      cleanupFunctional(__pinInterruptHandlers[pin].arg);
    }
    __pinInterruptHandlers[pin].fn = (voidFuncPtr)userFunc;
    __pinInterruptHandlers[pin].arg = arg;
    __pinInterruptHandlers[pin].functional = functional;

    gpio_set_intr_type((gpio_num_t)pin, (gpio_int_type_t)(intr_type & 0x7));
    if (intr_type & 0x8) {
      gpio_wakeup_enable((gpio_num_t)pin, (gpio_int_type_t)(intr_type & 0x7));
    }
    gpio_isr_handler_add((gpio_num_t)pin, __onPinInterrupt, &__pinInterruptHandlers[pin]);

    //FIX interrupts on peripherals outputs (eg. LEDC,...)
    //Enable input in GPIO register
    gpio_hal_context_t gpiohal;
    gpiohal.dev = GPIO_LL_GET_HW(GPIO_PORT_0);
    gpio_hal_input_enable(&gpiohal, pin);
  }
}

extern void __attachInterrupt(uint8_t pin, voidFuncPtr userFunc, int intr_type) { // fit TCA9535 
  if (pin >= PCA9535_BASE_PIN && pin < PCA9535_BASE_PIN + PCA9535_PIN_COUNT) {
    // PCA9535 don't support
    log_e("PCA9535 pin %i does not support interrupts", pin);
  } else {
    // or we take basic function of it
    __attachInterruptFunctionalArg(pin, (voidFuncPtrArg)userFunc, NULL, intr_type, false);
  }
}

extern void __detachInterrupt(uint8_t pin) {  // fit TCA9535 
  if (pin >= PCA9535_BASE_PIN && pin < PCA9535_BASE_PIN + PCA9535_PIN_COUNT) {
    // PCA9535 does not support interrupts
    log_e("PCA9535 pin %i does not support interrupts", pin);
  } else {
    // if we use basic function  of ESP32 detachInterrupt 
    gpio_isr_handler_remove((gpio_num_t)pin);
    gpio_wakeup_disable((gpio_num_t)pin);

    if (__pinInterruptHandlers[pin].functional && __pinInterruptHandlers[pin].arg) {
      cleanupFunctional(__pinInterruptHandlers[pin].arg);
    }
    __pinInterruptHandlers[pin].fn = NULL;
    __pinInterruptHandlers[pin].arg = NULL;
    __pinInterruptHandlers[pin].functional = false;

    gpio_set_intr_type((gpio_num_t)pin, GPIO_INTR_DISABLE);
  }
}

extern void enableInterrupt(uint8_t pin) {
  gpio_intr_enable((gpio_num_t)pin);
}

extern void disableInterrupt(uint8_t pin) {
  gpio_intr_disable((gpio_num_t)pin);
}

extern void pinMode(uint8_t pin, uint8_t mode) __attribute__((weak, alias("__pinMode")));
extern void digitalWrite(uint8_t pin, uint8_t val) __attribute__((weak, alias("__digitalWrite")));
extern int digitalRead(uint8_t pin) __attribute__((weak, alias("__digitalRead")));
extern void attachInterrupt(uint8_t pin, voidFuncPtr handler, int mode) __attribute__((weak, alias("__attachInterrupt")));
extern void attachInterruptArg(uint8_t pin, voidFuncPtrArg handler, void *arg, int mode) __attribute__((weak, alias("__attachInterruptArg")));
extern void detachInterrupt(uint8_t pin) __attribute__((weak, alias("__detachInterrupt")));
