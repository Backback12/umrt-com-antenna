#pragma once

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>

namespace umrt_com_antenna
{

/**
 * @brief Interfaces an MPU9265/MPU9250 (accel + gyro + AK8963 magnetometer) over I2C1
 *        (SDA=GPIO2, SCL=GPIO3), publishes raw sensor readings, and fuses them into
 *        a single heading (yaw) estimate.
 *
 * Publishes:
 *  - imu/data_raw   (sensor_msgs/Imu)            raw accel + gyro, no orientation
 *  - imu/mag        (sensor_msgs/MagneticField)  raw magnetometer, in Tesla
 *  - imu/heading_deg (std_msgs/Float64)          fused yaw heading, 0-360 deg
 */
class Mpu9265Node : public rclcpp::Node
{
public:
  Mpu9265Node();
  ~Mpu9265Node() override;

private:
  void openI2C();
  void configureImu();
  void configureMagnetometer();
  void timerCallback();

  void readImuRaw(int16_t out[7]);   // [accelX,Y,Z, temp, gyroX,Y,Z]
  bool readMagRaw(int16_t out[3]);   // [magX,Y,Z], false if data-not-ready

  uint8_t readRegister(uint8_t addr, uint8_t reg);
  void writeRegister(uint8_t addr, uint8_t reg, uint8_t value);
  void readBlock(uint8_t addr, uint8_t reg, uint8_t * buf, size_t len);

  std::string i2c_bus_ = "/dev/i2c-1";  // SDA=GPIO2 / SCL=GPIO3 on the Pi header
  int i2c_fd_ = -1;

  static constexpr uint8_t MPU_ADDR = 0x68;   // AD0 tied low; use 0x69 if tied high
  static constexpr uint8_t MAG_ADDR = 0x0C;   // AK8963, reached via I2C bypass

  double yaw_deg_ = 0.0;
  rclcpp::Time last_time_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace umrt_com_antenna