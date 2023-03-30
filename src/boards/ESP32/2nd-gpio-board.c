#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "gpio-board.h"

static Gpio_t *GpioIrq[GPIO_NUM_MAX];
static intr_handle_t gpio_intr_handle = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;

  if (GpioIrq[gpio_num] != NULL && GpioIrq[gpio_num]->IrqHandler != NULL) {
    GpioIrq[gpio_num]->IrqHandler(GpioIrq[gpio_num]->Context);
  }
}

void GpioMcuInit(Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config,
                 PinTypes type, uint32_t value) {
  gpio_config_t io_conf;

  obj->pin = pin;

  if (pin == NC) {
    return;
  }

  io_conf.pin_bit_mask = 1ULL << pin;
  io_conf.intr_type = GPIO_INTR_DISABLE;

  if (mode == PIN_INPUT) {
    io_conf.mode = GPIO_MODE_INPUT;
  } else {
    io_conf.mode = GPIO_MODE_OUTPUT;
  }

  if (type == PIN_PULL_UP) {
    io_conf.pull_up_en = 1;
  } else if (type == PIN_PULL_DOWN) {
    io_conf.pull_down_en = 1;
  } else {
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
  }

  gpio_config(&io_conf);

  if (mode == PIN_OUTPUT) {
    GpioMcuWrite(obj, value);
  }
}

void GpioMcuSetContext(Gpio_t *obj, void *context) { obj->Context = context; }

void GpioMcuSetInterrupt(Gpio_t *obj, IrqModes irqMode,
                         IrqPriorities irqPriority,
                         GpioIrqHandler *irqHandler) {
  gpio_config_t io_conf;
  gpio_int_type_t intr_type;

  if (irqHandler == NULL) {
    return;
  }

  obj->IrqHandler = irqHandler;

  if (irqMode == IRQ_RISING_EDGE) {
    intr_type = GPIO_INTR_POSEDGE;
  } else if (irqMode == IRQ_FALLING_EDGE) {
    intr_type = GPIO_INTR_NEGEDGE;
  } else {
    intr_type = GPIO_INTR_ANYEDGE;
  }

  io_conf.intr_type = intr_type;
  io_conf.pin_bit_mask = 1ULL << obj->pin;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = 0;
  io_conf.pull_down_en = 0;
  gpio_config(&io_conf);

  GpioIrq[obj->pin] = obj;

  if (gpio_intr_handle == NULL) {
    gpio_install_isr_service(0);
  }

  gpio_isr_handler_remove(obj->pin);
  gpio_isr_handler_add(obj->pin, gpio_isr_handler, (void *)obj->pin);
}

void GpioMcuRemoveInterrupt(Gpio_t *obj) {
  gpio_isr_handler_remove(obj->pin);
  GpioIrq[obj->pin] = NULL;
}

void GpioMcuWrite(Gpio_t *obj, uint32_t value) {
  if (obj == NULL) {
    while (1)
      ;
  }

  if (obj->pin == NC) {
    return;
  }

  gpio_set_level(obj->pin, value);
}

void GpioMcuToggle(Gpio_t *obj) {
  if (obj == NULL) {
    while (1)
      ;
  }
  if (obj->pin == NC) {
    return;
  }

  gpio_set_level(obj->pin, !gpio_get_level(obj->pin));
}

uint32_t GpioMcuRead(Gpio_t *obj) {
  if (obj == NULL) {
    while (1)
      ;
  }
  if (obj->pin == NC) {
    return 0;
  }

  return gpio_get_level(obj->pin);
  if (obj->pin == NC) {
    return 0;
  }

  return gpio_get_level(obj->pin);
}