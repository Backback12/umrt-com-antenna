#include "umrt_com_antenna/mpu9265_node.hpp"

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

Mpu9265Node::Mpu9265Node()
: Node("mpu9265_node")
{
  i2c_bus_ = this->declare_parameter<std::string>("i2c_bus", i2c_bus_);

  openI2C();
  configureImu();
  configureMagnetometer();

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
  mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
  heading_pub_ = this->create_publisher<std_msgs::msg::Float64>("imu/heading_deg", 10);

  last_time_ = this->now();

  // 50 Hz publish/fusion rate
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&Mpu9265Node::timerCallback, this));
}

Mpu9265Node::~Mpu9265Node()
{
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
  }
}

void Mpu9265Node::openI2C()
{
  i2c_fd_ = open(i2c_bus_.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    RCLCPP_FATAL(this->get_logger(), "Failed to open I2C bus %s: %s",
      i2c_bus_.c_str(), std::strerror(errno));
    throw std::runtime_error("Could not open I2C bus");
  }
}

void Mpu9265Node::readBlock(uint8_t addr, uint8_t reg, uint8_t * buf, size_t len)
{
  // Combined write(reg)+read(len) transaction with a proper repeated START,
  // required because the MPU9265/AK8963 need the register pointer set first.
  i2c_msg msgs[2];
  msgs[0] = {addr, 0, 1, &reg};
  msgs[1] = {addr, I2C_M_RD, static_cast<uint16_t>(len), buf};

  i2c_rdwr_ioctl_data ioctl_data{msgs, 2};
  if (ioctl(i2c_fd_, I2C_RDWR, &ioctl_data) < 0) {
    RCLCPP_ERROR(this->get_logger(), "I2C read failed (addr 0x%02X reg 0x%02X): %s",
      addr, reg, std::strerror(errno));
  }
}

uint8_t Mpu9265Node::readRegister(uint8_t addr, uint8_t reg)
{
  uint8_t val = 0;
  readBlock(addr, reg, &val, 1);
  return val;
}

void Mpu9265Node::writeRegister(uint8_t addr, uint8_t reg, uint8_t value)
{
  uint8_t buf[2] = {reg, value};
  i2c_msg msg = {addr, 0, 2, buf};
  i2c_rdwr_ioctl_data ioctl_data{&msg, 1};

  if (ioctl(i2c_fd_, I2C_RDWR, &ioctl_data) < 0) {
    RCLCPP_ERROR(this->get_logger(), "I2C write failed (addr 0x%02X reg 0x%02X): %s",
      addr, reg, std::strerror(errno));
  }
}

void Mpu9265Node::configureImu()
{
  uint8_t who = readRegister(MPU_ADDR, 0x75);  // WHO_AM_I
  RCLCPP_INFO(this->get_logger(), "MPU WHO_AM_I = 0x%02X", who);

  writeRegister(MPU_ADDR, 0x6B, 0x00);  // PWR_MGMT_1: wake, internal clock
  rclcpp::sleep_for(std::chrono::milliseconds(50));
  writeRegister(MPU_ADDR, 0x19, 0x04);  // SMPLRT_DIV
  writeRegister(MPU_ADDR, 0x1A, 0x03);  // CONFIG: DLPF ~44Hz
  writeRegister(MPU_ADDR, 0x1B, 0x00);  // GYRO_CONFIG: +-250 dps
  writeRegister(MPU_ADDR, 0x1C, 0x00);  // ACCEL_CONFIG: +-2g
  writeRegister(MPU_ADDR, 0x37, 0x02);  // INT_PIN_CFG: BYPASS_EN, exposes AK8963 at 0x0C
}

void Mpu9265Node::configureMagnetometer()
{
  writeRegister(MAG_ADDR, 0x0A, 0x00);  // CNTL1: power down first
  rclcpp::sleep_for(std::chrono::milliseconds(10));
  writeRegister(MAG_ADDR, 0x0A, 0x16);  // CNTL1: continuous mode 2 (100Hz), 16-bit output
  // NOTE: production code should also read the ASAX/ASAY/ASAZ sensitivity
  // adjustment values from fuse ROM (CNTL1=0x0F) and apply per-axis
  // hard/soft-iron calibration for an accurate heading.
}

void Mpu9265Node::readImuRaw(int16_t out[7])
{
  uint8_t raw[14];
  readBlock(MPU_ADDR, 0x3B, raw, 14);  // ACCEL_XOUT_H .. GYRO_ZOUT_L
  for (int i = 0; i < 7; ++i) {
    out[i] = static_cast<int16_t>((raw[2 * i] << 8) | raw[2 * i + 1]);
  }
}

bool Mpu9265Node::readMagRaw(int16_t out[3])
{
  uint8_t st1 = readRegister(MAG_ADDR, 0x02);  // ST1
  if (!(st1 & 0x01)) {
    return false;  // data not ready yet
  }

  uint8_t raw[7];
  readBlock(MAG_ADDR, 0x03, raw, 7);  // HXL..HZH + ST2 (little-endian, ST2 latches the sample)
  out[0] = static_cast<int16_t>((raw[1] << 8) | raw[0]);
  out[1] = static_cast<int16_t>((raw[3] << 8) | raw[2]);
  out[2] = static_cast<int16_t>((raw[5] << 8) | raw[4]);
  return true;
}

void Mpu9265Node::timerCallback()
{
  int16_t imu_raw[7];
  int16_t mag_raw[3] = {0, 0, 0};

  readImuRaw(imu_raw);
  bool mag_ok = readMagRaw(mag_raw);

  constexpr double ACCEL_SCALE = 9.80665 / 16384.0;        // m/s^2 per LSB, +-2g
  constexpr double GYRO_SCALE = (M_PI / 180.0) / 131.0;    // rad/s per LSB, +-250dps
  constexpr double MAG_SCALE = 0.15e-6;                    // Tesla per LSB, 16-bit mode

  double ax = imu_raw[0] * ACCEL_SCALE;
  double ay = imu_raw[1] * ACCEL_SCALE;
  double az = imu_raw[2] * ACCEL_SCALE;
  double gx = imu_raw[4] * GYRO_SCALE;
  double gy = imu_raw[5] * GYRO_SCALE;
  double gz = imu_raw[6] * GYRO_SCALE;

  rclcpp::Time now = this->now();
  double dt = (last_time_.nanoseconds() > 0) ? (now - last_time_).seconds() : 0.02;
  last_time_ = now;

  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header.stamp = now;
  imu_msg.header.frame_id = "imu_link";
  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;
  imu_msg.angular_velocity.x = gx;
  imu_msg.angular_velocity.y = gy;
  imu_msg.angular_velocity.z = gz;
  imu_msg.orientation_covariance[0] = -1;  // orientation not filled by this message
  imu_pub_->publish(imu_msg);

  double mx = 0.0, my = 0.0, mz = 0.0;
  if (mag_ok) {
    mx = mag_raw[0] * MAG_SCALE;
    my = mag_raw[1] * MAG_SCALE;
    mz = mag_raw[2] * MAG_SCALE;

    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.header.stamp = now;
    mag_msg.header.frame_id = "imu_link";
    mag_msg.magnetic_field.x = mx;
    mag_msg.magnetic_field.y = my;
    mag_msg.magnetic_field.z = mz;
    mag_pub_->publish(mag_msg);
  }

  // --- Tilt-compensated compass heading, fused with gyro-integrated yaw ---
  double roll = std::atan2(ay, az);
  double pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));

  double mag_heading_deg = yaw_deg_;  // hold last estimate if a mag sample wasn't ready
  if (mag_ok) {
    double mx_comp = mx * std::cos(pitch) + mz * std::sin(pitch);
    double my_comp = mx * std::sin(roll) * std::sin(pitch) + my * std::cos(roll) -
      mz * std::sin(roll) * std::cos(pitch);
    double heading_rad = std::atan2(-my_comp, mx_comp);
    mag_heading_deg = heading_rad * 180.0 / M_PI;
    if (mag_heading_deg < 0) mag_heading_deg += 360.0;
  }

  double gyro_yaw_deg = yaw_deg_ + gz * 180.0 / M_PI * dt;

  // Resolve wraparound (e.g. 359 vs 1) before blending the two estimates
  double diff = gyro_yaw_deg - mag_heading_deg;
  if (diff > 180.0) mag_heading_deg += 360.0;
  else if (diff < -180.0) mag_heading_deg -= 360.0;

  constexpr double ALPHA = 0.98;  // trust gyro short-term, mag corrects long-term drift
  yaw_deg_ = ALPHA * gyro_yaw_deg + (1.0 - ALPHA) * mag_heading_deg;
  if (yaw_deg_ >= 360.0) yaw_deg_ -= 360.0;
  if (yaw_deg_ < 0.0) yaw_deg_ += 360.0;

  std_msgs::msg::Float64 heading_msg;
  heading_msg.data = yaw_deg_;
  heading_pub_->publish(heading_msg);
}

}  // namespace umrt_com_antenna