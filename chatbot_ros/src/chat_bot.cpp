// Copyright (C) 2023 Miguel Ángel González Santamarta
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "chatbot_ros/chat_bot.hpp"

#include <initializer_list>
#include <memory>
#include <string>

#include "chatbot_ros/states/generate_chat_completions_state.hpp"
#include "chatbot_ros/states/listening_fsm.hpp"
#include "chatbot_ros/states/speak_response_state.hpp"
#include "chatbot_ros/states/speak_state.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/yasmin_node.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

namespace chatbot_ros {

namespace {

/// Concurrence that resets the shared pipeline before each responding cycle
/// so the same state objects can be reused across conversation turns
/// (`SpeechPipeline::close` is otherwise permanent).
class RespondingConcurrence : public yasmin::Concurrence {
public:
  RespondingConcurrence(const SpeechPipeline::SharedPtr &pipeline,
                        const yasmin::StateMap &states,
                        const std::string &default_outcome,
                        const yasmin::OutcomeMap &outcome_map)
      : yasmin::Concurrence(states, default_outcome, outcome_map),
        pipeline_(pipeline) {}

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
    this->pipeline_->reset();
    return yasmin::Concurrence::execute(blackboard);
  }

private:
  SpeechPipeline::SharedPtr pipeline_;
};

} // namespace

yasmin::Concurrence::SharedPtr
create_responding_concurrence(const SpeechPipeline::SharedPtr &pipeline,
                              const ChatbotConfig &config,
                              const ConversationHistory::SharedPtr &history,
                              const EmbeddingMemory::SharedPtr &memory) {
  auto generate_state = std::make_shared<GenerateChatCompletionsState>(
      pipeline, config, history, memory);
  auto speak_state = std::make_shared<SpeakResponseState>(pipeline);

  yasmin::StateMap states{{"GENERATE", generate_state}, {"SPEAK", speak_state}};
  yasmin::OutcomeMap outcomes{
      {yasmin_ros::basic_outcomes::SUCCEED,
       {{"GENERATE", yasmin_ros::basic_outcomes::SUCCEED},
        {"SPEAK", yasmin_ros::basic_outcomes::SUCCEED}}},
      {yasmin_ros::basic_outcomes::ABORT,
       {{"GENERATE", yasmin_ros::basic_outcomes::ABORT}}},
      {yasmin_ros::basic_outcomes::CANCEL,
       {{"GENERATE", yasmin_ros::basic_outcomes::CANCEL},
        {"SPEAK", yasmin_ros::basic_outcomes::CANCEL}}},
  };

  return std::make_shared<RespondingConcurrence>(
      pipeline, states, yasmin_ros::basic_outcomes::ABORT, outcomes);
}

ChatBot::ChatBot()
    : ChatBot(
          ChatbotConfig::from_node(yasmin_ros::YasminNode::get_instance())) {}

ChatBot::ChatBot(const ChatbotConfig &config)
    : pipeline_(std::make_shared<SpeechPipeline>()), config_(config),
      history_(std::make_shared<ConversationHistory>()),
      memory_(std::make_shared<EmbeddingMemory>(
          this->config_, yasmin_ros::YasminNode::get_instance())) {
  if (!this->config_.greeting.empty()) {
    this->history_->add_assistant_message(this->config_.greeting);
  }
  this->build_state_machine();
}

void ChatBot::build_state_machine() {
  this->state_machine_ = yasmin::StateMachine::make_shared(
      std::initializer_list<std::string>{yasmin_ros::basic_outcomes::CANCEL,
                                         yasmin_ros::basic_outcomes::ABORT},
      true);

  this->state_machine_->add_state(
      "GREETING", std::make_shared<SpeakState>(),
      {{yasmin_ros::basic_outcomes::SUCCEED, "LISTENING"},
       {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT},
       {yasmin_ros::basic_outcomes::CANCEL,
        yasmin_ros::basic_outcomes::CANCEL}});

  this->state_machine_->add_state(
      "LISTENING", std::make_shared<ListenFSM>(this->config_),
      {{yasmin_ros::basic_outcomes::SUCCEED, "RESPONDING"},
       {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT},
       {yasmin_ros::basic_outcomes::CANCEL,
        yasmin_ros::basic_outcomes::CANCEL}});

  this->state_machine_->add_state(
      "RESPONDING",
      create_responding_concurrence(this->pipeline_, this->config_,
                                    this->history_, this->memory_),
      {{yasmin_ros::basic_outcomes::SUCCEED, "LISTENING"},
       {yasmin_ros::basic_outcomes::ABORT, yasmin_ros::basic_outcomes::ABORT},
       {yasmin_ros::basic_outcomes::CANCEL,
        yasmin_ros::basic_outcomes::CANCEL}});

  if (this->config_.greeting.empty()) {
    this->state_machine_->set_start_state("LISTENING");
  }
}

std::string ChatBot::execute() {
  auto blackboard = yasmin::Blackboard::make_shared();
  if (!this->config_.greeting.empty()) {
    blackboard->set<std::string>("tts", this->config_.greeting);
  }
  yasmin_viewer::YasminViewerPub yasmin_pub(this->state_machine_, "CHAT_BOT");
  return (*this->state_machine_)(blackboard);
}

yasmin::StateMachine::SharedPtr ChatBot::get_state_machine() const {
  return this->state_machine_;
}

const ChatbotConfig &ChatBot::get_config() const { return this->config_; }

} // namespace chatbot_ros
