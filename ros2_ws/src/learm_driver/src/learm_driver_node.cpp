#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "arm_serial_control/srv/send_hex.hpp"
#include "learm_driver/protocol.hpp"
#include "learm_driver/srv/get_arm_status.hpp"
#include "learm_driver/srv/move_joints.hpp"

namespace
{

constexpr uint32_t kMinimumMoveTimeMs = 20;
constexpr uint32_t kMaximumMoveTimeMs = 30000;
constexpr std::size_t kJointCount = 6;
constexpr std::size_t kStatusPayloadSize = 26;

struct JointCalibration
{
  uint8_t id{};
  double min_position_rad{};
  double max_position_rad{};
  int min_position_pulse_us{};
  int max_position_pulse_us{};
};

struct PendingResponse
{
  bool active{false};
  bool ready{false};
  uint8_t sequence{};
  uint8_t request_command{};
  uint8_t expected_response{};
  learm_driver::Frame frame;
};

void append_u16_le(std::vector<uint8_t> & output, const uint16_t value)
{
  output.push_back(static_cast<uint8_t>(value & 0xFFU));
  output.push_back(static_cast<uint8_t>((value >> 8) & 0xFFU));
}

uint16_t read_u16_le(const std::vector<uint8_t> & input, const std::size_t offset)
{
  return static_cast<uint16_t>(input[offset]) |
    (static_cast<uint16_t>(input[offset + 1]) << 8);
}

}  // namespace

class LeArmDriver : public rclcpp::Node
{
public:
  LeArmDriver()
  : Node("learm_driver")
  {
    calibration_enabled_ = declare_parameter<bool>("calibration_enabled", false);
    ack_timeout_ms_ = declare_parameter<int>("ack_timeout_ms", 200);
    const auto serial_service = declare_parameter<std::string>(
      "serial_send_service", "/serial_controller/send_hex");
    const auto received_topic = declare_parameter<std::string>(
      "serial_received_topic", "/serial_controller/received_hex");
    load_calibration();

    serial_client_ = create_client<arm_serial_control::srv::SendHex>(serial_service);
    received_subscription_ = create_subscription<std_msgs::msg::String>(
      received_topic, 20,
      std::bind(&LeArmDriver::received_hex_callback, this, std::placeholders::_1));

    move_service_ = create_service<learm_driver::srv::MoveJoints>(
      "~/move_joints",
      std::bind(&LeArmDriver::move_joints_callback, this, std::placeholders::_1, std::placeholders::_2));
    status_service_ = create_service<learm_driver::srv::GetArmStatus>(
      "~/get_status",
      std::bind(&LeArmDriver::get_status_callback, this, std::placeholders::_1, std::placeholders::_2));
    estop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/emergency_stop",
      std::bind(&LeArmDriver::emergency_stop_callback, this, std::placeholders::_1, std::placeholders::_2));
    clear_estop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/clear_estop",
      std::bind(&LeArmDriver::clear_estop_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(), "LeArm driver ready; calibration is %s",
      calibration_enabled_ ? "enabled" : "disabled");
  }

private:
  void load_calibration()
  {
    for (std::size_t index = 1; index <= kJointCount; ++index) {
      const auto name = "joint_" + std::to_string(index);
      JointCalibration calibration;
      calibration.id = static_cast<uint8_t>(declare_parameter<int>(name + ".id", index));
      calibration.min_position_rad = declare_parameter<double>(name + ".min_position_rad", 0.0);
      calibration.max_position_rad = declare_parameter<double>(name + ".max_position_rad", 0.0);
      calibration.min_position_pulse_us = declare_parameter<int>(
        name + ".pulse_at_min_position_us", 0);
      calibration.max_position_pulse_us = declare_parameter<int>(
        name + ".pulse_at_max_position_us", 0);
      calibration_.emplace(name, calibration);
    }
  }

  bool validate_calibration(std::string & error) const
  {
    std::set<uint8_t> ids;
    for (const auto & [name, calibration] : calibration_) {
      if (calibration.id < 1 || calibration.id > kJointCount || !ids.insert(calibration.id).second) {
        error = "calibration contains an invalid or duplicate servo ID";
        return false;
      }
      if (!std::isfinite(calibration.min_position_rad) ||
        !std::isfinite(calibration.max_position_rad) ||
        calibration.max_position_rad <= calibration.min_position_rad)
      {
        error = "calibration must define an increasing angle range for " + name;
        return false;
      }
      if (calibration.min_position_pulse_us < 500 || calibration.min_position_pulse_us > 2500 ||
        calibration.max_position_pulse_us < 500 || calibration.max_position_pulse_us > 2500)
      {
        error = "calibration pulse range must stay within 500 to 2500 us";
        return false;
      }
      if (calibration.id == 1 &&
        (calibration.min_position_pulse_us > 1500 || calibration.max_position_pulse_us > 1500))
      {
        error = "servo ID 1 must not exceed 1500 us";
        return false;
      }
    }
    return true;
  }

  bool position_to_pulse(const JointCalibration & calibration, const double position_rad,
    uint16_t & pulse, std::string & error) const
  {
    if (!std::isfinite(position_rad) || position_rad < calibration.min_position_rad ||
      position_rad > calibration.max_position_rad)
    {
      error = "target position is outside the calibrated joint range";
      return false;
    }

    const double fraction = (position_rad - calibration.min_position_rad) /
      (calibration.max_position_rad - calibration.min_position_rad);
    const double mapped = static_cast<double>(calibration.min_position_pulse_us) + fraction *
      static_cast<double>(calibration.max_position_pulse_us - calibration.min_position_pulse_us);
    const auto rounded = static_cast<int>(std::lround(mapped));
    if (rounded < 500 || rounded > 2500 || (calibration.id == 1 && rounded > 1500)) {
      error = "calibration maps the target to an unsafe PWM pulse";
      return false;
    }
    pulse = static_cast<uint16_t>(rounded);
    return true;
  }

  void move_joints_callback(
    const std::shared_ptr<learm_driver::srv::MoveJoints::Request> request,
    std::shared_ptr<learm_driver::srv::MoveJoints::Response> response)
  {
    if (!calibration_enabled_) {
      response->success = false;
      response->error_code = learm_driver::kStatusCalibrationDisabled;
      response->message = "motion is disabled until calibration_enabled is true";
      return;
    }
    std::string error;
    if (!validate_calibration(error)) {
      response->success = false;
      response->error_code = learm_driver::kStatusCalibrationInvalid;
      response->message = error;
      return;
    }
    if (request->joint_names.empty() || request->joint_names.size() > kJointCount ||
      request->joint_names.size() != request->positions_rad.size())
    {
      response->success = false;
      response->error_code = learm_driver::kStatusInvalidPayload;
      response->message = "joint names and positions must have the same length from 1 to 6";
      return;
    }
    if (request->duration_ms < kMinimumMoveTimeMs || request->duration_ms > kMaximumMoveTimeMs) {
      response->success = false;
      response->error_code = learm_driver::kStatusInvalidPayload;
      response->message = "duration_ms must be between 20 and 30000";
      return;
    }

    std::set<std::string> names;
    std::vector<std::pair<uint8_t, uint16_t>> targets;
    for (std::size_t index = 0; index < request->joint_names.size(); ++index) {
      const auto & name = request->joint_names[index];
      const auto calibration = calibration_.find(name);
      if (calibration == calibration_.end() || !names.insert(name).second) {
        response->success = false;
        response->error_code = learm_driver::kStatusInvalidPayload;
        response->message = "joint names must be unique values from joint_1 to joint_6";
        return;
      }
      uint16_t pulse = 0;
      if (!position_to_pulse(calibration->second, request->positions_rad[index], pulse, error)) {
        response->success = false;
        response->error_code = learm_driver::kStatusInvalidPayload;
        response->message = name + ": " + error;
        return;
      }
      targets.emplace_back(calibration->second.id, pulse);
    }

    std::sort(targets.begin(), targets.end());
    std::vector<uint8_t> payload;
    append_u16_le(payload, static_cast<uint16_t>(request->duration_ms));
    payload.push_back(static_cast<uint8_t>(targets.size()));
    for (const auto & [id, pulse] : targets) {
      payload.push_back(id);
      append_u16_le(payload, pulse);
    }

    learm_driver::Frame frame;
    uint8_t error_code = learm_driver::kStatusOk;
    if (!send_transaction(learm_driver::kCommandMovePulses, payload, learm_driver::kCommandAck,
        frame, error_code, error))
    {
      response->success = false;
      response->error_code = error_code;
      response->message = error;
      return;
    }
    if (!decode_ack(frame, learm_driver::kCommandMovePulses, error_code, error)) {
      response->success = false;
      response->error_code = error_code;
      response->message = error;
      return;
    }
    response->success = true;
    response->error_code = learm_driver::kStatusOk;
    response->message = "motion accepted by STM32";
  }

  void get_status_callback(
    const std::shared_ptr<learm_driver::srv::GetArmStatus::Request>,
    std::shared_ptr<learm_driver::srv::GetArmStatus::Response> response)
  {
    std::string error;
    uint8_t error_code = learm_driver::kStatusOk;
    learm_driver::Frame frame;
    if (!send_transaction(learm_driver::kCommandGetStatus, {}, learm_driver::kCommandStatus,
        frame, error_code, error))
    {
      response->success = false;
      response->error_code = error_code;
      response->message = error;
      return;
    }
    if (frame.payload.size() != kStatusPayloadSize) {
      response->success = false;
      response->error_code = learm_driver::kStatusInvalidPayload;
      response->message = "STM32 returned an invalid status payload";
      return;
    }
    response->moving = (frame.payload[0] & 0x01U) != 0;
    response->estop_active = (frame.payload[0] & 0x02U) != 0;
    response->moving_mask = frame.payload[1];
    for (std::size_t index = 0; index < kJointCount; ++index) {
      response->current_pulse_us[index] = read_u16_le(frame.payload, 2 + (index * 2));
      response->target_pulse_us[index] = read_u16_le(frame.payload, 14 + (index * 2));
    }
    response->success = true;
    response->error_code = learm_driver::kStatusOk;
    response->message = "status received from STM32";
  }

  void emergency_stop_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    send_control_command(learm_driver::kCommandEmergencyStop, "emergency stop", *response);
  }

  void clear_estop_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    send_control_command(learm_driver::kCommandClearEstop, "clear estop", *response);
  }

  void send_control_command(const uint8_t command, const std::string & label,
    std_srvs::srv::Trigger::Response & response)
  {
    std::string error;
    uint8_t error_code = learm_driver::kStatusOk;
    learm_driver::Frame frame;
    if (!send_transaction(command, {}, learm_driver::kCommandAck, frame, error_code, error) ||
      !decode_ack(frame, command, error_code, error))
    {
      response.success = false;
      response.message = label + " failed (" + std::to_string(error_code) + "): " + error;
      return;
    }
    response.success = true;
    response.message = label + " accepted by STM32";
  }

  bool send_transaction(const uint8_t command, const std::vector<uint8_t> & payload,
    const uint8_t expected_response, learm_driver::Frame & output, uint8_t & error_code,
    std::string & error)
  {
    std::unique_lock<std::mutex> transaction_lock(transaction_mutex_);
    const uint8_t sequence = next_sequence();
    {
      std::lock_guard<std::mutex> pending_lock(pending_mutex_);
      pending_ = PendingResponse{true, false, sequence, command, expected_response, {}};
    }

    if (!serial_client_->wait_for_service(std::chrono::milliseconds(ack_timeout_ms_))) {
      clear_pending(sequence);
      error_code = learm_driver::kStatusTransportError;
      error = "serial send service is unavailable";
      return false;
    }

    auto request = std::make_shared<arm_serial_control::srv::SendHex::Request>();
    request->data = learm_driver::bytes_to_hex(learm_driver::encode_frame(sequence, command, payload));
    auto send_future = serial_client_->async_send_request(request);
    if (send_future.wait_for(std::chrono::milliseconds(ack_timeout_ms_)) != std::future_status::ready) {
      clear_pending(sequence);
      error_code = learm_driver::kStatusTransportError;
      error = "serial send service timed out";
      return false;
    }
    const auto send_response = send_future.get();
    if (!send_response->success) {
      clear_pending(sequence);
      error_code = learm_driver::kStatusTransportError;
      error = send_response->message;
      return false;
    }

    std::unique_lock<std::mutex> pending_lock(pending_mutex_);
    const bool received = pending_cv_.wait_for(
      pending_lock, std::chrono::milliseconds(ack_timeout_ms_),
      [this, sequence]() {return pending_.active && pending_.sequence == sequence && pending_.ready;});
    if (!received) {
      pending_ = PendingResponse{};
      error_code = learm_driver::kStatusTimeout;
      error = "STM32 response timed out; movement command was not retried";
      return false;
    }
    output = pending_.frame;
    pending_ = PendingResponse{};
    return true;
  }

  bool decode_ack(const learm_driver::Frame & frame, const uint8_t request_command,
    uint8_t & error_code, std::string & error) const
  {
    if (frame.command != learm_driver::kCommandAck || frame.payload.size() != 2 ||
      frame.payload[0] != request_command)
    {
      error_code = learm_driver::kStatusInvalidPayload;
      error = "STM32 returned an invalid ACK";
      return false;
    }
    error_code = frame.payload[1];
    if (error_code != learm_driver::kStatusOk) {
      error = "STM32 rejected the command";
      return false;
    }
    return true;
  }

  void received_hex_callback(const std_msgs::msg::String::SharedPtr message)
  {
    std::vector<uint8_t> bytes;
    std::string error;
    if (!learm_driver::hex_to_bytes(message->data, bytes, error)) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid serial input: %s", error.c_str());
      return;
    }

    std::lock_guard<std::mutex> lock(receive_mutex_);
    receive_buffer_.insert(receive_buffer_.end(), bytes.begin(), bytes.end());
    while (true) {
      while (receive_buffer_.size() >= 2 &&
        (receive_buffer_[0] != learm_driver::kFrameHeader1 ||
        receive_buffer_[1] != learm_driver::kFrameHeader2))
      {
        receive_buffer_.erase(receive_buffer_.begin());
      }
      if (receive_buffer_.size() < 6) {
        return;
      }
      const std::size_t frame_size = learm_driver::kFrameMinLength + receive_buffer_[5];
      if (frame_size > learm_driver::kFrameMaxLength) {
        receive_buffer_.erase(receive_buffer_.begin());
        continue;
      }
      if (receive_buffer_.size() < frame_size) {
        return;
      }
      std::vector<uint8_t> frame_bytes(
        receive_buffer_.begin(), receive_buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
      receive_buffer_.erase(
        receive_buffer_.begin(), receive_buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
      learm_driver::Frame frame;
      if (!learm_driver::decode_frame(frame_bytes, frame, error)) {
        RCLCPP_WARN(get_logger(), "Ignoring invalid STM32 frame: %s", error.c_str());
        continue;
      }
      handle_frame(frame);
    }
  }

  void handle_frame(const learm_driver::Frame & frame)
  {
    if (frame.version != learm_driver::kProtocolVersion) {
      return;
    }
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (!pending_.active || pending_.sequence != frame.sequence ||
      pending_.expected_response != frame.command)
    {
      return;
    }
    pending_.frame = frame;
    pending_.ready = true;
    pending_cv_.notify_all();
  }

  void clear_pending(const uint8_t sequence)
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (pending_.active && pending_.sequence == sequence) {
      pending_ = PendingResponse{};
    }
  }

  uint8_t next_sequence()
  {
    ++sequence_;
    if (sequence_ == 0) {
      ++sequence_;
    }
    return sequence_;
  }

  bool calibration_enabled_{};
  int ack_timeout_ms_{};
  uint8_t sequence_{};
  std::map<std::string, JointCalibration> calibration_;
  std::vector<uint8_t> receive_buffer_;
  PendingResponse pending_;
  std::mutex transaction_mutex_;
  std::mutex pending_mutex_;
  std::mutex receive_mutex_;
  std::condition_variable pending_cv_;

  rclcpp::Client<arm_serial_control::srv::SendHex>::SharedPtr serial_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr received_subscription_;
  rclcpp::Service<learm_driver::srv::MoveJoints>::SharedPtr move_service_;
  rclcpp::Service<learm_driver::srv::GetArmStatus>::SharedPtr status_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_estop_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(std::make_shared<LeArmDriver>());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
