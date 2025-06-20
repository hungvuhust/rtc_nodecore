#pragma once

#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace rtcrobot_core {

/**
 * @brief Signal Handler cho graceful shutdown ROS2 executor
 *
 * Class này quản lý signal handling (SIGINT, SIGTERM) để shutdown
 * ROS2 executor một cách graceful khi nhận signal
 */
class SignalHandler {
public:
  /**
   * @brief Constructor
   */
  SignalHandler();

  /**
   * @brief Destructor
   */
  ~SignalHandler();

  /**
   * @brief Setup signal handling
   * @return true nếu setup thành công
   */
  bool setup();

  /**
   * @brief Kiểm tra có signal shutdown không
   * @return true nếu có signal shutdown
   */
  bool isShutdownRequested() const;

  /**
   * @brief Reset shutdown flag
   */
  void reset();

  /**
   * @brief Spin executor với signal handling
   * @param node ROS2 node để spin
   * @return exit code
   */
  int spinWithSignalHandling(std::shared_ptr<rclcpp::Node> node);

  /**
   * @brief Spin executor với timeout và signal handling
   * @param executor ROS2 executor
   * @param timeout_ms Timeout in milliseconds (0 = no timeout)
   * @return exit code
   */
  int spinWithSignalHandling(
      rclcpp::executors::SingleThreadedExecutor &executor, int timeout_ms = 0);

  /**
   * @brief Get singleton instance
   * @return reference to singleton instance
   */
  static SignalHandler &getInstance();

private:
  static std::atomic<bool> shutdown_requested_;
  static SignalHandler    *instance_;

  bool signal_setup_done_;

  /**
   * @brief Signal handler function
   * @param signal Signal number
   */
  static void signalHandlerFunction(int signal);

  // Prevent copy
  SignalHandler(const SignalHandler &)            = delete;
  SignalHandler &operator=(const SignalHandler &) = delete;
};

} // namespace rtcrobot_core