#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace
{
constexpr size_t PKT_LEN = 16;
constexpr double ACCEL_SCALE = 9.80665 / 8192.0;
constexpr double GYRO_SCALE = M_PI / (180.0 * 65.5);
}

class ImuBridgeNode : public rclcpp::Node
{
public:
  ImuBridgeNode() : Node("imu_bridge")
  {
    declare_parameter("port", "/dev/ttyACM0");
    declare_parameter("frame_id", "imu_link");
    declare_parameter("topic", "/imu/data_raw");
    declare_parameter("linear_acceleration_stddev", 0.2);
    declare_parameter("angular_velocity_stddev", 0.02);

    port_ = get_parameter("port").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    const auto topic = get_parameter("topic").as_string();
    const auto linear_accel_stddev = get_parameter("linear_acceleration_stddev").as_double();
    const auto angular_velocity_stddev = get_parameter("angular_velocity_stddev").as_double();

    const auto linear_variance = linear_accel_stddev * linear_accel_stddev;
    const auto angular_variance = angular_velocity_stddev * angular_velocity_stddev;
    linear_accel_covariance_ = {
      linear_variance, 0.0, 0.0,
      0.0, linear_variance, 0.0,
      0.0, 0.0, linear_variance
    };
    angular_velocity_covariance_ = {
      angular_variance, 0.0, 0.0,
      0.0, angular_variance, 0.0,
      0.0, 0.0, angular_variance
    };

    pub_ = create_publisher<sensor_msgs::msg::Imu>(topic, 10);

    fd_ = open_serial(port_);
    if (fd_ < 0) {
      throw std::runtime_error("Cannot open serial port " + port_);
    }

    RCLCPP_INFO(get_logger(), "Opened %s and publishing raw IMU on %s", port_.c_str(), topic.c_str());

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&ImuBridgeNode::read_loop, this));
  }

  ~ImuBridgeNode() override
  {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  int open_serial(const std::string & path)
  {
    int fd = open(path.c_str(), O_RDONLY | O_NOCTTY);
    if (fd < 0) {
      return fd;
    }

    struct termios tty {};
    tcgetattr(fd, &tty);

    cfmakeraw(&tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIFLUSH);
    return fd;
  }

  void read_loop()
  {
    uint8_t tmp[256];
    const ssize_t n = read(fd_, tmp, sizeof(tmp));
    if (n <= 0) {
      return;
    }

    for (ssize_t i = 0; i < n; ++i) {
      buf_[buf_len_++] = tmp[i];
      if (buf_len_ >= sizeof(buf_)) {
        buf_len_ = 0;
      }
    }

    while (buf_len_ >= PKT_LEN) {
      if (buf_[0] != 0xAA || buf_[1] != 0x55) {
        memmove(buf_, buf_ + 1, --buf_len_);
        continue;
      }

      uint16_t checksum = 0;
      for (size_t j = 2; j < 14; ++j) {
        checksum += buf_[j];
      }

      const uint16_t packet_checksum = (buf_[14] << 8) | buf_[15];
      if (checksum != packet_checksum) {
        memmove(buf_, buf_ + 1, --buf_len_);
        continue;
      }

      const int16_t ax = static_cast<int16_t>((buf_[2] << 8) | buf_[3]);
      const int16_t ay = static_cast<int16_t>((buf_[4] << 8) | buf_[5]);
      const int16_t az = static_cast<int16_t>((buf_[6] << 8) | buf_[7]);
      const int16_t gx = static_cast<int16_t>((buf_[8] << 8) | buf_[9]);
      const int16_t gy = static_cast<int16_t>((buf_[10] << 8) | buf_[11]);
      const int16_t gz = static_cast<int16_t>((buf_[12] << 8) | buf_[13]);

      sensor_msgs::msg::Imu msg;
      msg.header.stamp = now();
      msg.header.frame_id = frame_id_;

      msg.orientation_covariance[0] = -1.0;
      msg.angular_velocity_covariance = angular_velocity_covariance_;
      msg.linear_acceleration_covariance = linear_accel_covariance_;

      msg.linear_acceleration.x = ax * ACCEL_SCALE;
      msg.linear_acceleration.y = ay * ACCEL_SCALE;
      msg.linear_acceleration.z = az * ACCEL_SCALE;

      msg.angular_velocity.x = gx * GYRO_SCALE;
      msg.angular_velocity.y = gy * GYRO_SCALE;
      msg.angular_velocity.z = gz * GYRO_SCALE;

      pub_->publish(msg);

      buf_len_ -= PKT_LEN;
      memmove(buf_, buf_ + PKT_LEN, buf_len_);
    }
  }

  std::string port_;
  std::string frame_id_;
  int fd_ = -1;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::array<double, 9> linear_accel_covariance_ {};
  std::array<double, 9> angular_velocity_covariance_ {};
  uint8_t buf_[512] = {};
  size_t buf_len_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ImuBridgeNode>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("imu_bridge"), "%s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
