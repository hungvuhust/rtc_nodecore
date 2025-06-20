#include "rtc_nodecore/signal_handler.hpp"
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace rtcrobot_core {

// Static member definitions
std::atomic<bool> SignalHandler::shutdown_requested_{false};
SignalHandler    *SignalHandler::instance_ = nullptr;

SignalHandler::SignalHandler() : signal_setup_done_(false) { instance_ = this; }

SignalHandler::~SignalHandler() { instance_ = nullptr; }

bool SignalHandler::setup() {
  if (signal_setup_done_) {
    return true;
  }

  // Setup SIGINT handler (Ctrl+C)
  if (std::signal(SIGINT, signalHandlerFunction) == SIG_ERR) {
    std::cerr << "❌ Error setting up SIGINT handler" << std::endl;
    return false;
  }

  // Setup SIGTERM handler (kill command)
  if (std::signal(SIGTERM, signalHandlerFunction) == SIG_ERR) {
    std::cerr << "❌ Error setting up SIGTERM handler" << std::endl;
    return false;
  }

  signal_setup_done_ = true;
  std::cout << "✅ Signal handlers setup successfully (SIGINT, SIGTERM)"
            << std::endl;
  return true;
}

bool SignalHandler::isShutdownRequested() const {
  return shutdown_requested_.load();
}

void SignalHandler::reset() { shutdown_requested_.store(false); }

int SignalHandler::spinWithSignalHandling(std::shared_ptr<rclcpp::Node> node) {
  if (!setup()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to setup signal handling");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),
              "Node spinning... (Ctrl+C để thoát gracefully)");

  try {
    // Spin until shutdown requested
    while (rclcpp::ok() && !isShutdownRequested()) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (isShutdownRequested()) {
      RCLCPP_INFO(node->get_logger(),
                  "Shutdown signal received, stopping gracefully...");
    }

  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Exception during spin: %s", e.what());
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Node shutdown completed");
  return 0;
}

int SignalHandler::spinWithSignalHandling(
    rclcpp::executors::SingleThreadedExecutor &executor, int timeout_ms) {
  if (!setup()) {
    std::cerr << "❌ Failed to setup signal handling" << std::endl;
    return 1;
  }

  std::cout << "🔄 Executor spinning... (Ctrl+C để thoát gracefully)"
            << std::endl;

  try {
    if (timeout_ms > 0) {
      // Spin with timeout
      auto timeout = std::chrono::milliseconds(timeout_ms);
      while (rclcpp::ok() && !isShutdownRequested()) {
        executor.spin_once(timeout);
      }
    } else {
      // Spin indefinitely until shutdown
      while (rclcpp::ok() && !isShutdownRequested()) {
        executor.spin_once(std::chrono::milliseconds(100));
      }
    }

    if (isShutdownRequested()) {
      std::cout
          << "🛑 Shutdown signal received, stopping executor gracefully..."
          << std::endl;
    }

  } catch (const std::exception &e) {
    std::cerr << "❌ Exception during executor spin: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "✅ Executor shutdown completed" << std::endl;
  return 0;
}

SignalHandler &SignalHandler::getInstance() {
  static SignalHandler instance;
  return instance;
}

void SignalHandler::signalHandlerFunction(int signal) {
  // Set shutdown flag
  shutdown_requested_.store(true);

  // Print signal info
  const char *signal_name = "UNKNOWN";
  switch (signal) {
  case SIGINT: signal_name = "SIGINT (Ctrl+C)"; break;
  case SIGTERM: signal_name = "SIGTERM"; break;
  default: break;
  }

  std::cout << std::endl
            << "🔴 Signal " << signal_name << " received!" << std::endl
            << "🔄 Initiating graceful shutdown..." << std::endl;

  // Don't call rclcpp::shutdown() here as it might cause deadlock
  // Let the main loop handle it
}

} // namespace rtcrobot_core