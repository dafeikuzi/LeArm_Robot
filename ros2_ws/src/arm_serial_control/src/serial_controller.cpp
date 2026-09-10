#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <memory>
#include <netdb.h>
#include <netinet/tcp.h>
#include <optional>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
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
  : Node("serial_controller")
  {
    transport_ = declare_parameter<std::string>("transport", "tcp");
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    tcp_host_ = declare_parameter<std::string>("tcp_host", "127.0.0.1");
    tcp_port_ = declare_parameter<int>("tcp_port", 8766);
    connect_timeout_ms_ = declare_parameter<int>("connect_timeout_ms", 1000);
    reconnect_interval_ms_ = declare_parameter<int>("reconnect_interval_ms", 1000);
    auto_connect_ = declare_parameter<bool>("auto_connect", true);
    const auto read_period_ms = declare_parameter<int>("read_period_ms", 20);
    log_received_text_ = declare_parameter<bool>("log_received_text", true);

    if (transport_ != "tcp" && transport_ != "serial") {
      throw std::invalid_argument("transport must be 'tcp' or 'serial'");
    }
    if (tcp_port_ < 1 || tcp_port_ > 65535 || connect_timeout_ms_ < 1 || reconnect_interval_ms_ < 1) {
      throw std::invalid_argument("TCP port and timeout parameters are invalid");
    }

    received_publisher_ = create_publisher<std_msgs::msg::String>("~/received_hex", 10);
    received_text_publisher_ = create_publisher<std_msgs::msg::String>("~/received_text", 10);
    connect_service_ = create_service<std_srvs::srv::Trigger>(
      "~/connect",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = connect_transport(true);
        response->message = response->success ? transport_ + " transport connected" : last_error_;
      });
    disconnect_service_ = create_service<std_srvs::srv::Trigger>(
      "~/disconnect",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        disconnect_transport();
        response->success = true;
        response->message = transport_ + " transport disconnected";
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
    if (auto_connect_) {
      connect_transport(true);
    }
  }

  ~SerialController() override
  {
    disconnect_transport();
  }

private:
  bool connect_transport(const bool force = false)
  {
    if (transport_fd_ >= 0) {
      return true;
    }
    const auto now = std::chrono::steady_clock::now();
    if (!force && last_connect_attempt_.time_since_epoch().count() != 0 &&
      now - last_connect_attempt_ < std::chrono::milliseconds(reconnect_interval_ms_))
    {
      return false;
    }
    last_connect_attempt_ = now;
    return transport_ == "tcp" ? connect_tcp() : connect_serial();
  }

  bool connect_tcp()
  {
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo * addresses = nullptr;
    const auto service = std::to_string(tcp_port_);
    const int lookup = getaddrinfo(tcp_host_.c_str(), service.c_str(), &hints, &addresses);
    if (lookup != 0) {
      last_error_ = "cannot resolve TCP host " + tcp_host_ + ": " + gai_strerror(lookup);
      return false;
    }

    int final_error = ECONNREFUSED;
    for (auto * address = addresses; address != nullptr; address = address->ai_next) {
      const int candidate = socket(address->ai_family, address->ai_socktype, address->ai_protocol);
      if (candidate < 0) {
        final_error = errno;
        continue;
      }
      const int flags = fcntl(candidate, F_GETFL, 0);
      if (flags < 0 || fcntl(candidate, F_SETFL, flags | O_NONBLOCK) != 0) {
        final_error = errno;
        close(candidate);
        continue;
      }
      int result = ::connect(candidate, address->ai_addr, address->ai_addrlen);
      if (result != 0 && errno == EINPROGRESS) {
        pollfd descriptor{candidate, POLLOUT, 0};
        const int poll_result = poll(&descriptor, 1, connect_timeout_ms_);
        if (poll_result > 0) {
          int socket_error = 0;
          socklen_t length = sizeof(socket_error);
          if (getsockopt(candidate, SOL_SOCKET, SO_ERROR, &socket_error, &length) == 0 &&
            socket_error == 0)
          {
            result = 0;
          } else {
            result = -1;
            final_error = socket_error == 0 ? errno : socket_error;
          }
        } else {
          result = -1;
          final_error = poll_result == 0 ? ETIMEDOUT : errno;
        }
      } else if (result != 0) {
        final_error = errno;
      }
      if (result == 0) {
        int enabled = 1;
        setsockopt(candidate, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
        setsockopt(candidate, SOL_SOCKET, SO_KEEPALIVE, &enabled, sizeof(enabled));
        transport_fd_ = candidate;
        freeaddrinfo(addresses);
        RCLCPP_INFO(get_logger(), "Connected to TCP serial bridge at %s:%d", tcp_host_.c_str(), tcp_port_);
        return true;
      }
      close(candidate);
    }
    freeaddrinfo(addresses);
    last_error_ = "cannot connect to TCP serial bridge " + tcp_host_ + ":" +
      std::to_string(tcp_port_) + ": " + std::strerror(final_error);
    RCLCPP_WARN(get_logger(), "%s", last_error_.c_str());
    return false;
  }

  bool set_normal_run_modem_lines()
  {
    int modem_bits = 0;
    if (ioctl(transport_fd_, TIOCMGET, &modem_bits) != 0) {
      last_error_ = "cannot read modem control lines: " + std::string(std::strerror(errno));
      return false;
    }
    modem_bits |= TIOCM_DTR;
    modem_bits &= ~TIOCM_RTS;
    if (ioctl(transport_fd_, TIOCMSET, &modem_bits) != 0) {
      last_error_ = "cannot set modem control lines: " + std::string(std::strerror(errno));
      return false;
    }
    return true;
  }

  bool connect_serial()
  {
    const auto speed = baud_to_termios(baud_rate_);
    if (!speed) {
      last_error_ = "unsupported baud_rate: " + std::to_string(baud_rate_);
      return false;
    }
    transport_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (transport_fd_ < 0) {
      last_error_ = "cannot open " + port_ + ": " + std::strerror(errno);
      return false;
    }
    termios options{};
    if (!set_normal_run_modem_lines() || tcgetattr(transport_fd_, &options) != 0) {
      if (last_error_.empty()) {
        last_error_ = "cannot read serial settings: " + std::string(std::strerror(errno));
      }
      disconnect_transport();
      return false;
    }
    cfmakeraw(&options);
    cfsetispeed(&options, *speed);
    cfsetospeed(&options, *speed);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~(HUPCL | CSTOPB | CRTSCTS | CSIZE);
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    if (tcsetattr(transport_fd_, TCSANOW, &options) != 0 || !set_normal_run_modem_lines()) {
      last_error_ = "cannot configure serial port: " + std::string(std::strerror(errno));
      disconnect_transport();
      return false;
    }
    RCLCPP_INFO(get_logger(), "Connected to %s at %d baud", port_.c_str(), baud_rate_);
    return true;
  }

  void disconnect_transport()
  {
    if (transport_fd_ >= 0) {
      close(transport_fd_);
      transport_fd_ = -1;
    }
  }

  bool write_all(const uint8_t * data, const std::size_t size)
  {
    std::size_t total = 0;
    while (total < size) {
      const auto written = transport_ == "tcp" ?
        send(transport_fd_, data + total, size - total, MSG_NOSIGNAL) :
        write(transport_fd_, data + total, size - total);
      if (written > 0) {
        total += static_cast<std::size_t>(written);
        continue;
      }
      if (written < 0 && errno == EINTR) {
        continue;
      }
      if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        pollfd descriptor{transport_fd_, POLLOUT, 0};
        if (poll(&descriptor, 1, connect_timeout_ms_) > 0) {
          continue;
        }
        errno = ETIMEDOUT;
      }
      last_error_ = transport_ + " write failed: " + std::strerror(errno);
      disconnect_transport();
      return false;
    }
    if (transport_ == "serial") {
      tcdrain(transport_fd_);
    }
    return true;
  }

  void send_hex(const std::string & hex_data, arm_serial_control::srv::SendHex::Response & response)
  {
    try {
      const auto bytes = parse_hex(hex_data);
      if (!connect_transport() || !write_all(bytes.data(), bytes.size())) {
        response.success = false;
        response.message = last_error_;
        return;
      }
      response.success = true;
      response.message = "wrote " + std::to_string(bytes.size()) + " byte(s) over " + transport_;
    } catch (const std::exception & error) {
      response.success = false;
      response.message = error.what();
    }
  }

  void read_available()
  {
    if (transport_fd_ < 0) {
      if (auto_connect_) {
        connect_transport();
      }
      return;
    }
    uint8_t buffer[256];
    const auto count = transport_ == "tcp" ?
      recv(transport_fd_, buffer, sizeof(buffer), MSG_DONTWAIT) :
      read(transport_fd_, buffer, sizeof(buffer));
    if (count > 0) {
      std_msgs::msg::String message;
      message.data = bytes_to_hex(buffer, static_cast<std::size_t>(count));
      received_publisher_->publish(message);
      publish_received_text(buffer, static_cast<std::size_t>(count));
    } else if (count == 0 && transport_ == "tcp") {
      last_error_ = "TCP serial bridge closed the connection";
      RCLCPP_WARN(get_logger(), "%s", last_error_.c_str());
      disconnect_transport();
    } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      last_error_ = transport_ + " read failed: " + std::strerror(errno);
      RCLCPP_WARN(get_logger(), "%s", last_error_.c_str());
      disconnect_transport();
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

  std::string transport_;
  std::string port_;
  int baud_rate_{};
  std::string tcp_host_;
  int tcp_port_{};
  int connect_timeout_ms_{};
  int reconnect_interval_ms_{};
  bool auto_connect_{};
  int transport_fd_{-1};
  std::chrono::steady_clock::time_point last_connect_attempt_{};
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
