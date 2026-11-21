#include <Arduino.h>
#include <esp_rom_sys.h>
#include "DAC8562.h"

DAC8562::DAC8562(uint32_t frequency)
    : _settings(frequency, MSBFIRST, SPI_MODE1) {} // DAC8562需Mode 1

void DAC8562::begin()
{
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);

    // 初始化设备
    write(DAC_CMD_RESET, 0x0001);
    write(DAC_CMD_REF_GAIN_CONFIG, 0x0001);
}

void DAC8562::_transfer(uint8_t cmd, uint16_t data)
{
    // Arduino SPI reads the backing buffer a full 32-bit word at a time.
    alignas(uint32_t) const uint8_t word[4] = {cmd, uint8_t(data >> 8), uint8_t(data), 0};
    digitalWrite(CS_PIN, LOW);
    SPI.writeBytes(word, 3);
    digitalWrite(CS_PIN, HIGH);
}

void DAC8562::write(uint8_t cmd_byte, uint16_t data_val)
{
    SPI.beginTransaction(_settings);
    _transfer(cmd_byte, data_val);
    SPI.endTransaction();
}

void DAC8562::sync_outputs(uint16_t ch_a_val, uint16_t ch_b_val)
{
    // 按照原逻辑，先写A但不更新，后写B并同时更新A和B
    SPI.beginTransaction(_settings);
    _transfer(DAC_CMD_WRITE_CH_A, ch_a_val);
    // Each command requires a separate SYNC pulse and at least 80 ns high.
    esp_rom_delay_us(1);
    _transfer(DAC_CMD_WRITE_CH_B_AND_UPDATE_ALL, ch_b_val);
    SPI.endTransaction();
}
