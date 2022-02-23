#ifndef ARDUINO_SENSORS_HPP
#define ARDUINO_SENSORS_HPP

#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"

#define ARDUINO_ENCODER_COUNT 2
#define ARDUINO_ENCODER_PPR 100.0
#define ARDUINO_PORT 20
#define ARDUINO_BAUD 115200

void arduino_sensors_setup();

class ArduinoEncoder : public okapi::ContinuousRotarySensor {
private:
  std::atomic<int32_t> *currentRawVal;
  int32_t tareOffset;

public:
  ArduinoEncoder(std::atomic<int32_t> *encoder);

  /**
   * Get the current sensor value, in degrees.
   *
   * @return the current sensor value, or `PROS_ERR` on a failure.
   */
  virtual double get() const override;

  /**
   * Reset the sensor to zero.
   *
   * @return `1` on success, `PROS_ERR` on fail
   */
  virtual std::int32_t reset() override;

  /**
   * Get the sensor value for use in a control loop. This method might be automatically called in
   * another thread by the controller.
   *
   * @return the current sensor value, or `PROS_ERR` on a failure.
   */
  virtual double controllerGet() override;
};

ArduinoEncoder arduino_encoder_create(int index);

#endif /* ARDUINO_SENSORS_HPP */