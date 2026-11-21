#pragma once

#include <Arduino.h>

class DAC8562;

void galvoParser_init();
uint32_t galvoParser_scan(DAC8562 &dac, bool timerFault, uint32_t coalescedTicks);
void galvoParser_setAppliedPeriod(uint32_t period);
