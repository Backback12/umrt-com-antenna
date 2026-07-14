#pragma once

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>

namespace umrt_com_antenna
{

/**
 * @brief Interfaces a QMC5883P magnetometer over I2C1 (SDA=GPIO2, SCL=GPIO3).
 *
 * NOTE: boards silkscreened "HMC5883L"/"GY-271" now commonly ship this chip
 * instead (QST QMC5883P, ID 0x80, fixed I2C address 0x2C). It has a different
 * register map, byte order, and scaling than the HMC5883L/QMC5883L, per the
 * QST QMC5883P datasheet (Rev. C).
 *
 * Assumes the sensor is mounted flat, so heading is derived from X/Y only
 * (no tilt compensation).
 *
 * Publishes:
 *  - mag/data_raw     (sensor_msgs/MagneticField)  raw X/Y/Z field, in Tesla
 *  - imu/heading_deg  (std_msgs/Float64)           heading, 0-360 deg
 */
class Qmc5883pNode : public rclcpp::Node
{
public:
  Qmc5883pNode();
  ~Qmc5883pNode() override;

private:
  void openI2C();
  void configureSensor();
  void timerCallback();

  bool readMagRaw(int16_t out[3]);  // [x, y, z]

  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);
  void readBlock(uint8_t reg, uint8_t * buf, size_t len);

  std::string i2c_bus_ = "/dev/i2c-1";  // SDA=GPIO2 / SCL=GPIO3 on the Pi header
  int i2c_fd_ = -1;

  static constexpr uint8_t MAG_ADDR = 0x2C;  // fixed address per QMC5883P datasheet

  double declination_deg_ = 0.0;  // local magnetic declination correction, set via param

  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace umrt_com_antenna