#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "arm_serial_control/srv/send_hex.hpp"

namespace
{

std::optional<speed_t> baud_to_termios(const int baud_rate)
{
  switch (baud_rate) {
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: return std::nullopt;
  }
}

std::vector<uint8_t> parse_hex(const std::string & input)
{
  std::string compact;
  for (const char character : input) {
    if (!std::isspace(static_cast<unsigned char>(character))) {
      compact += character;
    }
  }

  if (compact.empty() || compact.size() % 2 != 0) {
    throw std::invalid_argument("hex data must contain complete byte pairs");
  }

  std::vector<uint8_t> bytes;
  bytes.reserve(compact.size() / 2);
  for (std::size_t index = 0; index < compact.size(); index += 2) {
    const std::string byte_text = compact.substr(index, 2);
    std::size_t parsed = 0;
    const auto value = std::stoul(byte_text, &parsed, 16);
    if (parsed != byte_text.size() || value > 0xff) {
      throw std::invalid_argument("invalid hex byte: " + byte_text);
    }
    bytes.push_back(static_cast<uint8_t>(value));
  }
  return bytes;
}

std::string bytes_to_hex(const uint8_t * bytes, const std::size_t count)
{
  std::ostringstream stream;
  stream << std::uppercase << std::hex << std::setfill('0');
  for (std::size_t index = 0; index < count; ++index) {
    if (index != 0) {
      stream << ' ';
    }
    stream << std::setw(2) << static_cast<unsigned int>(bytes[index]);
  }
  return stream.str();
}

}  // namespace

class SerialController : public rclcpp::Node
{
public:
  SerialController()
  : Node("serial_controller"), serial_fd_(-1)
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    const auto auto_connect = declare_parameter<bool>("auto_connect", false);
    const auto read_period_ms = declare_parameter<int>("read_period_ms", 20);
    log_received_text_ = declare_parameter<bool>("log_received_text", true);

    received_publisher_ = create_publisher<std_msgs::msg::String>("~/received_hex", 10);
    received_text_publisher_ = create_publisher<std_msgs::msg::String>("~/received_text", 10);
    connect_service_ = create_service<std_srvs::srv::Trigger>(
      "~/connect",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = connect();
        response->message = response->success ? "serial port connected" : last_error_;
      });
    disconnect_service_ = create_service<std_srvs::srv::Trigger>(
      "~/disconnect",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        disconnect();
        response->success = true;
        response->message = "serial port disconnected";
      });
    send_service_ = create_service<arm_serial_control::srv::SendHex>(
      "~/send_hex",
      [this](const std::shared_ptr<arm_serial_control::srv::SendHex::Request> request,
        std::shared_ptr<arm_serial_control::srv::SendHex::Response> response) {
        send_hex(request->data, *response);
      });

    read_timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(read_period_ms, 1L)),
      std::bind(&SerialController::read_available, this));

    if (auto_connect) {
      connect();
    }
  }

  ~SerialController() override
  {
    disconnect();
  }

private:
  bool set_normal_run_modem_lines()
  {
    int modem_bits = 0;
    if (ioctl(serial_fd_, TIOCMGET, &modem_bits) != 0) {
      last_error_ = "cannot read modem control lines: " + std::string(std::strerror(errno));
      return false;
    }

    // The Elite V2 CH340 auto-download circuit enters the STM32 bootloader
    // when DTR is low and RTS is high. Keep its normal-run state explicitly.
    modem_bits |= TIOCM_DTR;
    modem_bits &= ~TIOCM_RTS;
    if (ioctl(serial_fd_, TIOCMSET, &modem_bits) != 0) {
      last_error_ = "cannot set modem control lines: " + std::string(std::strerror(errno));
      return false;
    }
    return true;
  }

  bool connect()
  {
    if (serial_fd_ >= 0) {
      return true;
    }

    const auto speed = baud_to_termios(baud_rate_);
    if (!speed) {
      last_error_ = "unsupported baud_rate: " + std::to_string(baud_rate_);
      RCLCPP_ERROR(get_logger(), "%s", last_error_.c_str());
      return false;
    }

    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd_ < 0) {
      last_error_ = "cannot open " + port_ + ": " + std::strerror(errno);
      RCLCPP_ERROR(get_logger(), "%s", last_error_.c_str());
      return false;
    }

    if (!set_normal_run_modem_lines()) {
      disconnect();
      return false;
    }

    termios options{};
    if (tcgetattr(serial_fd_, &options) != 0) {
      last_error_ = "cannot read serial settings: " + std::string(std::strerror(errno));
      disconnect();
      return false;
    }
    cfmakeraw(&options);
    cfsetispeed(&options, *speed);
    cfsetospeed(&options, *speed);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~HUPCL;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
      last_error_ = "cannot apply serial settings: " + std::string(std::strerror(errno));
      disconnect();
      return false;
    }

    if (!set_normal_run_modem_lines()) {
      disconnect();
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "Connected to %s at %d baud (DTR=high, RTS=low)",
      port_.c_str(), baud_rate_);
    return true;
  }

  void disconnect()
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  void send_hex(
    const std::string & hex_data,
    arm_serial_control::srv::SendHex::Response & response)
  {
    try {
      const auto bytes = parse_hex(hex_data);
      if (!connect()) {
        response.success = false;
        response.message = last_error_;
        return;
      }
      std::size_t total_written = 0;
      while (total_written < bytes.size()) {
        const auto written = write(
          serial_fd_, bytes.data() + total_written, bytes.size() - total_written);
        if (written < 0) {
          if (errno == EINTR) {
            continue;
          }
          response.success = false;
          response.message = "serial write failed: " + std::string(std::strerror(errno));
          last_error_ = response.message;
          disconnect();
          return;
        }
        if (written == 0) {
          response.success = false;
          response.message = "serial write made no progress";
          return;
        }
        total_written += static_cast<std::size_t>(written);
      }
      tcdrain(serial_fd_);
      response.success = true;
      response.message = "wrote " + std::to_string(total_written) + " byte(s)";
    } catch (const std::exception & error) {
      response.success = false;
      response.message = error.what();
    }
  }

  void read_available()
  {
    if (serial_fd_ < 0) {
      return;
    }
    uint8_t buffer[256];
    const auto count = read(serial_fd_, buffer, sizeof(buffer));
    if (count > 0) {
      std_msgs::msg::String message;
      message.data = bytes_to_hex(buffer, static_cast<std::size_t>(count));
      received_publisher_->publish(message);
      publish_received_text(buffer, static_cast<std::size_t>(count));
    } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      last_error_ = "serial read failed: " + std::string(std::strerror(errno));
      RCLCPP_WARN(get_logger(), "%s", last_error_.c_str());
      disconnect();
    }
  }

  void publish_received_text(const uint8_t * bytes, const std::size_t count)
  {
    for (std::size_t index = 0; index < count; ++index) {
      const auto character = static_cast<char>(bytes[index]);
      const bool line_break = character == '\n';
      const bool printable = std::isprint(static_cast<unsigned char>(character)) ||
        character == '\r' || character == '\t' || line_break;

      if (!printable) {
        received_text_buffer_.clear();
        continue;
      }

      if (character != '\r') {
        received_text_buffer_ += character;
      }

      if (received_text_buffer_.size() > 256) {
        received_text_buffer_.clear();
        continue;
      }

      if (line_break) {
        if (!received_text_buffer_.empty() && received_text_buffer_.back() == '\n') {
          received_text_buffer_.pop_back();
        }
        if (!received_text_buffer_.empty()) {
          std_msgs::msg::String message;
          message.data = received_text_buffer_;
          received_text_publisher_->publish(message);
          if (log_received_text_) {
            RCLCPP_DEBUG(get_logger(), "Serial text: %s", received_text_buffer_.c_str());
          }
        }
        received_text_buffer_.clear();
      }
    }
  }

  std::string port_;
  int baud_rate_;
  int serial_fd_;
  std::string last_error_;
  bool log_received_text_{};
  std::string received_text_buffer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr received_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr received_text_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disconnect_service_;
  rclcpp::Service<arm_serial_control::srv::SendHex>::SharedPtr send_service_;
  rclcpp::TimerBase::SharedPtr read_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialController>());
  rclcpp::shutdown();
  return 0;
}
