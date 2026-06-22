#include "As5600Sensor.h"

bool As5600Sensor::begin(TwoWire& wire, uint8_t sda_pin, uint8_t scl_pin,
                         uint8_t address) {
  wire_ = &wire;
  address_ = address;

  wire_->begin(sda_pin, scl_pin);
  wire_->setClock(400000); // Fast-mode: margem segura para o controle ADRC a 500 Hz.

  wire_->beginTransmission(address_);
  detected_ = (wire_->endTransmission() == 0);
  return detected_;
}

bool As5600Sensor::readRawAngle(uint16_t* raw_angle) {
  if (!wire_ || !raw_angle || !detected_) return false;

  wire_->beginTransmission(address_);
  wire_->write(REG_RAW_ANGLE_H);
  if (wire_->endTransmission(false) != 0) return false;

  const int bytes = wire_->requestFrom((int)address_, 2);
  if (bytes != 2) return false;

  const uint8_t high_byte = (uint8_t)wire_->read();
  const uint8_t low_byte = (uint8_t)wire_->read();

  *raw_angle = (uint16_t)(((uint16_t)high_byte << 8) | low_byte) & 0x0FFF;
  return true;
}

bool As5600Sensor::readAngleDeg(float* angle_deg) {
  if (!angle_deg) return false;

  uint16_t raw = 0;
  if (!readRawAngle(&raw)) return false;

  *angle_deg = ((float)raw * 360.0f) / 4096.0f;
  return true;
}
