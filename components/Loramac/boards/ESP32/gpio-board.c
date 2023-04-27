#include "gpio-board.h"
#include "driver/gpio.h"
#include <stdio.h>

void GpioMcuInit(Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config,
                 PinTypes type, uint32_t value) {
  obj->pin = pin;

  if (pin == NC) {
    return;
  }

  gpio_config_t io_conf;
  io_conf.pin_bit_mask = (1ULL << pin);
  io_conf.intr_type = GPIO_INTR_DISABLE;

  if (mode == PIN_INPUT) {
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en =
        (type == PIN_PULL_DOWN) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en =
        (type == PIN_PULL_UP) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  } else // mode output
  {
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  }

  gpio_config(&io_conf);

  // Sets initial output value
  if (mode == PIN_OUTPUT) {
    GpioMcuWrite(obj, value);
  }
}

void GpioMcuSetContext(Gpio_t *obj, void *context) { obj->Context = context; }

void gpio_isr_handler(void *arg) {
  Gpio_t *obj = (Gpio_t *)arg;
  if (obj->IrqHandler) {
    obj->IrqHandler(obj->Context);
  }
}

void GpioMcuSetInterrupt(Gpio_t *obj, IrqModes irqMode,
                         IrqPriorities irqPriority,
                         GpioIrqHandler *irqHandler) {
  if (irqMode == NO_IRQ) {
    gpio_set_intr_type(obj->pin, GPIO_INTR_DISABLE);
  } else {
    gpio_int_type_t intr_type;
    switch (irqMode) {
    case IRQ_RISING_EDGE:
      intr_type = GPIO_INTR_POSEDGE;
      break;
    case IRQ_FALLING_EDGE:
      intr_type = GPIO_INTR_NEGEDGE;
      break;
    case IRQ_RISING_FALLING_EDGE:
      intr_type = GPIO_INTR_ANYEDGE;
      break;
    default:
      intr_type = GPIO_INTR_DISABLE;
    }
    gpio_set_intr_type(obj->pin, intr_type);
  }

  obj->IrqHandler = irqHandler;

  gpio_install_isr_service(0);
  gpio_isr_handler_add(obj->pin, gpio_isr_handler, (void *)obj);
}

void GpioMcuRemoveInterrupt(Gpio_t *obj) { gpio_isr_handler_remove(obj->pin); }

void GpioMcuWrite(Gpio_t *obj, uint32_t value) {
  if (obj == NULL) {
    // assert_param( LMN_STATUS_ERROR );
    while (1)
      printf("Error on GpioMcuWrite!\n");
    ;
  }
  // Check if pin is not connected
  if (obj->pin == NC) {
    return;
  }
  gpio_set_level(obj->pin, value);
}

void GpioMcuToggle(Gpio_t *obj) {
  if (obj == NULL) {
    // assert_param( LMN_STATUS_ERROR );
    while (1)
      ;
  }

  // Check if pin is not connected
  if (obj->pin == NC) {
    return;
  }
  gpio_set_level(obj->pin, !gpio_get_level(obj->pin));
}

uint32_t GpioMcuRead(Gpio_t *obj) {
  if (obj == NULL) {
    // assert_param( LMN_STATUS_ERROR );
    while (1)
      ;
  }
  // Check if pin is not connected
  if (obj->pin == NC) {
    return 0;
  }
  return (uint32_t)gpio_get_level(obj->pin);
}
