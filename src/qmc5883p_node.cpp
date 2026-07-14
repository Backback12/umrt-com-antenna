#include "umrt_com_antenna/qmc5883p_node.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace umrt_com_antenna
{

Qmc5883pNode::Qmc5883pNode()
: Node("qmc5883p_node")
{
  i2c_bus_ = this->declare_parameter<std::string>("i2c_bus", i2c_bus_);
  declination_deg_ = this->declare_parameter<double>("declination_deg", declination_deg_);

  openI2C();
  configureSensor();

  mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag/data_raw", 10);
  heading_pub_ = this->create_publisher<std_msgs::msg::Float64>("imu/heading_deg", 10);

  // Sensor is configured for 200Hz ODR below, so 20ms (50Hz) polling is plenty
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&Qmc5883pNode::timerCallback, this));
}

Qmc5883pNode::~Qmc5883pNode()
{
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
  }
}

void Qmc5883pNode::openI2C()
{
  i2c_fd_ = open(i2c_bus_.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    RCLCPP_FATAL(this->get_logger(), "Failed to open I2C bus %s: %s",
      i2c_bus_.c_str(), std::strerror(errno));
    throw std::runtime_error("Could not open I2C bus");
  }
}

void Qmc5883pNode::readBlock(uint8_t reg, uint8_t * buf, size_t len)
{
  // Combined write(reg)+read(len) transaction with a proper repeated START
  i2c_msg msgs[2];
  msgs[0] = {MAG_ADDR, 0, 1, &reg};
  msgs[1] = {MAG_ADDR, I2C_M_RD, static_cast<uint16_t>(len), buf};

  i2c_rdwr_ioctl_data ioctl_data{msgs, 2};
  if (ioctl(i2c_fd_, I2C_RDWR, &ioctl_data) < 0) {
    RCLCPP_ERROR(this->get_logger(), "I2C read failed (reg 0x%02X): %s",
      reg, std::strerror(errno));
  }
}

uint8_t Qmc5883pNode::readRegister(uint8_t reg)
{
  uint8_t val = 0;
  readBlock(reg, &val, 1);
  return val;
}

void Qmc5883pNode::writeRegister(uint8_t reg, uint8_t value)
{
  uint8_t buf[2] = {reg, value};
  i2c_msg msg = {MAG_ADDR, 0, 2, buf};
  i2c_rdwr_ioctl_data ioctl_data{&msg, 1};

  if (ioctl(i2c_fd_, I2C_RDWR, &ioctl_data) < 0) {
    RCLCPP_ERROR(this->get_logger(), "I2C write failed (reg 0x%02X): %s",
      reg, std::strerror(errno));
  }
}

void Qmc5883pNode::configureSensor()
{
  uint8_t chip_id = readRegister(0x00);  // CHIPID, should read 0x80
  RCLCPP_INFO(this->get_logger(), "QMC5883P CHIPID = 0x%02X (expect 0x80)", chip_id);

  writeRegister(0x0B, 0x80);  // Control Reg 2: soft reset, restore defaults
  rclcpp::sleep_for(std::chrono::milliseconds(10));

  // Per QST datasheet section 7.1/7.2 application examples: axis sign register,
  // not listed in the main register map table but required by the vendor's
  // own setup sequence.
  writeRegister(0x29, 0x06);

  // Control Reg 2 (0x0B): RNG<1:0>=11 -> +-2 Gauss range (best sensitivity,
  // Earth's field is well within it), SET/RESET MODE<1:0>=00 -> set+reset on.
  writeRegister(0x0B, 0x0C);

  // Control Reg 1 (0x0A): OSR2=00(1x), OSR1=00(8x oversample, lowest noise),
  // ODR=11(200Hz), MODE=11(continuous mode).
  writeRegister(0x0A, 0x0F);

  // NOTE: production code should still do a hard/soft-iron calibration pass
  // (rotate the sensor through 360 deg, fit an ellipse, store offsets/scale)
  // even though this chip has built-in offset cancellation.
}

bool Qmc5883pNode::readMagRaw(int16_t out[3])
{
  uint8_t status = readRegister(0x09);  // Status register
  if (!(status & 0x01)) {
    return false;  // DRDY not set, data not ready
  }

  uint8_t raw[6];
  readBlock(0x01, raw, 6);  // XOUT_LSB, XOUT_MSB, YOUT_LSB, YOUT_MSB, ZOUT_LSB, ZOUT_MSB
  out[0] = static_cast<int16_t>((raw[1] << 8) | raw[0]);  // X
  out[1] = static_cast<int16_t>((raw[3] << 8) | raw[2]);  // Y
  out[2] = static_cast<int16_t>((raw[5] << 8) | raw[4]);  // Z
  return true;
}

void Qmc5883pNode::timerCallback()
{
  int16_t mag_raw[3];
  if (!readMagRaw(mag_raw)) {
    return;  // sample not ready yet, try again next tick
  }

  // +-2 Gauss range -> 15000 LSB/Gauss (QST datasheet Table 2); 1 Gauss = 1e-4 Tesla
  constexpr double MAG_SCALE = (1.0 / 15000.0) * 1e-4;  // Tesla per LSB

  double mx = mag_raw[0] * MAG_SCALE;
  double my = mag_raw[1] * MAG_SCALE;
  double mz = mag_raw[2] * MAG_SCALE;

  rclcpp::Time now = this->now();

  sensor_msgs::msg::MagneticField mag_msg;
  mag_msg.header.stamp = now;
  mag_msg.header.frame_id = "mag_link";
  mag_msg.magnetic_field.x = mx;
  mag_msg.magnetic_field.y = my;
  mag_msg.magnetic_field.z = mz;
  mag_pub_->publish(mag_msg);

  // Sensor is mounted flat, so heading comes straight from X/Y (no tilt compensation)
  double heading_rad = std::atan2(my, mx);
  double heading_deg = heading_rad * 180.0 / M_PI + declination_deg_;
  if (heading_deg < 0.0) heading_deg += 360.0;
  if (heading_deg >= 360.0) heading_deg -= 360.0;

  std_msgs::msg::Float64 heading_msg;
  heading_msg.data = heading_deg;
  heading_pub_->publish(heading_msg);
}

}  // namespace umrt_com_antenna