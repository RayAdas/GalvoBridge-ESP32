#include "LaserCtrl.h"

#include <driver/gpio.h>

static constexpr gpio_num_t LightPin = GPIO_NUM_3;

void laserCtrl_init()
{
    gpio_set_level(LightPin, 0);
    gpio_set_direction(LightPin, GPIO_MODE_OUTPUT);
}

void laserCtrl_setOutput(bool enabled)
{
    gpio_set_level(LightPin, enabled);
}
