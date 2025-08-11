#include <ros/ros.h>
#include <serial/serial.h>

#include <bitset>
#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "disturbance_estimation/motor_state.h"

class Motor {
 public:
  Motor() {}

  void parseData(const std::vector<uint8_t>& data);
  void printData() const;

  uint16_t get_packetNumber() { return this->packetNumber_; };
  float get_rcThrottle() { return this->rcThrottle_; };
  float get_actualThrottle() { return this->actualThrottle_; };
  uint16_t get_electricalSpeed() { return this->electricalSpeed_; };
  float get_busVoltage() { return this->busVoltage_; };
  float get_busCurrent() { return this->busCurrent_; };
  float get_phaseCurrent() { return this->phaseCurrent_; };
  uint8_t get_mosTemperature() { return this->mosTemperature_; };
  uint8_t get_capacitorTemperature() { return this->capacitorTemperature_; };
  uint16_t get_statusCode() { return this->statusCode_; };

 private:
  uint8_t channel;
  uint16_t packetNumber_;
  float rcThrottle_;
  float actualThrottle_;
  uint16_t electricalSpeed_;
  float busVoltage_;
  float busCurrent_;
  float phaseCurrent_;
  uint8_t mosTemperature_;
  uint8_t capacitorTemperature_;
  uint16_t statusCode_;
};
