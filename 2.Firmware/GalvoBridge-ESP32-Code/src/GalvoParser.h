#pragma once

#include <Arduino.h>

namespace GalvoPlayback
{
constexpr uint32_t BaseRate = 24000;
constexpr uint32_t TimerHz = 10000000;
constexpr uint32_t InitialPeriod = (TimerHz + BaseRate / 2) / BaseRate;
}

class DAC8562;

void galvoParser_init();
uint32_t galvoParser_scan(DAC8562 &dac, bool timerFault, uint32_t coalescedTicks);
void galvoParser_setAppliedPeriod(uint32_t period);
