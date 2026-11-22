// Copyright (C) 2023 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <csignal>
#include <exception>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "chatbot_ros/chat_bot.hpp"
#include "yasmin/logs.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_ros/yasmin_node.hpp"

namespace {

/// File-static pointer used by the SIGTERM handler. Set once before the state
/// machine runs and cleared before shutdown.
chatbot_ros::ChatBot *g_chatbot = nullptr;

/// SIGTERM (docker stop, `timeout`, `kill`) must cancel the state machine like
/// SIGINT does; otherwise rclcpp's SIGTERM handler shuts the context down while
/// a state is still waiting and the wait loops spin forever.
void handle_sigterm(int) {
  if (g_chatbot != nullptr) {
    g_chatbot->get_state_machine()->cancel_state_machine();
  }
}

} // namespace

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  struct sigaction previous_sigterm_action {};
  bool sigterm_installed = false;
  // Declared outside the try block so it is destroyed only after the SIGTERM
  // handler pointer has been cleared (no dangling pointer in the handler).
  std::unique_ptr<chatbot_ros::ChatBot> chatbot;

  try {
    yasmin_ros::set_ros_loggers();

    chatbot = std::make_unique<chatbot_ros::ChatBot>();
    g_chatbot = chatbot.get();

    struct sigaction sigterm_action {};
    sigterm_action.sa_handler = handle_sigterm;
    sigemptyset(&sigterm_action.sa_mask);
    sigterm_action.sa_flags = 0;
    if (sigaction(SIGTERM, &sigterm_action, &previous_sigterm_action) == 0) {
      sigterm_installed = true;
    } else {
      YASMIN_LOG_WARN("Could not install SIGTERM handler");
    }

    const std::string outcome = chatbot->execute();
    YASMIN_LOG_INFO("ChatBot finished with outcome: %s", outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN("%s", e.what());
  }

  g_chatbot = nullptr;
  if (sigterm_installed) {
    sigaction(SIGTERM, &previous_sigterm_action, nullptr);
  }
  chatbot.reset();

  yasmin_ros::YasminNode::destroy_instance();
  rclcpp::shutdown();
  return 0;
}
