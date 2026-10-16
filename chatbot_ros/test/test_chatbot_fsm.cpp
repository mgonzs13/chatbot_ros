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

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "audio_common_msgs/action/tts.hpp"
#include "llama_msgs/action/generate_chat_completions.hpp"
#include "whisper_msgs/action/stt.hpp"

#include "chatbot_ros/chat_bot.hpp"
#include "chatbot_ros/states/listening_fsm.hpp"
#include "chatbot_ros/states/speech_pipeline.hpp"
#include "yasmin/blackboard.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/ros_clients_cache.hpp"
#include "yasmin_ros/yasmin_node.hpp"

using namespace std::chrono_literals;
using TTS = audio_common_msgs::action::TTS;
using STT = whisper_msgs::action::STT;
using GenerateChatCompletions = llama_msgs::action::GenerateChatCompletions;

class StubTtsServer {
public:
  explicit StubTtsServer(const rclcpp::Node::SharedPtr &node) {
    this->server_ = rclcpp_action::create_server<TTS>(
        node, "/say",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const TTS::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<TTS>>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](
            std::shared_ptr<rclcpp_action::ServerGoalHandle<TTS>> goal_handle) {
          std::lock_guard<std::mutex> lock(this->mutex_);
          this->spoken_texts_.push_back(goal_handle->get_goal()->text);
          auto result = std::make_shared<TTS::Result>();
          result->text = goal_handle->get_goal()->text;
          goal_handle->succeed(result);
        });
  }

  std::vector<std::string> get_spoken_texts() {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->spoken_texts_;
  }

private:
  rclcpp_action::Server<TTS>::SharedPtr server_;
  std::mutex mutex_;
  std::vector<std::string> spoken_texts_;
};

class StubSttServer {
public:
  StubSttServer(const rclcpp::Node::SharedPtr &node,
                std::vector<std::string> transcriptions)
      : transcriptions_(transcriptions.begin(), transcriptions.end()) {
    this->server_ = rclcpp_action::create_server<STT>(
        node, "/whisper/listen",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const STT::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<STT>>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](
            std::shared_ptr<rclcpp_action::ServerGoalHandle<STT>> goal_handle) {
          std::string transcription = "";
          {
            std::lock_guard<std::mutex> lock(this->mutex_);
            if (!this->transcriptions_.empty()) {
              transcription = this->transcriptions_.front();
              this->transcriptions_.pop_front();
            }
          }
          auto result = std::make_shared<STT::Result>();
          result->transcription.text = transcription;
          goal_handle->succeed(result);
        });
  }

private:
  rclcpp_action::Server<STT>::SharedPtr server_;
  std::mutex mutex_;
  std::deque<std::string> transcriptions_;
};

class StubChatCompletionsServer {
public:
  StubChatCompletionsServer(const rclcpp::Node::SharedPtr &node,
                            std::vector<std::string> feedback_chunks,
                            std::string result_text, bool abort)
      : feedback_chunks_(std::move(feedback_chunks)),
        result_text_(std::move(result_text)), abort_(abort) {
    this->server_ = rclcpp_action::create_server<GenerateChatCompletions>(
        node, "/llama/generate_chat_completions",
        [](const rclcpp_action::GoalUUID &,
           std::shared_ptr<const GenerateChatCompletions::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<GenerateChatCompletions>>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<
               rclcpp_action::ServerGoalHandle<GenerateChatCompletions>>
                   goal_handle) {
          {
            std::lock_guard<std::mutex> lock(this->mutex_);
            this->last_goal_ = *goal_handle->get_goal();
          }
          // Let the client process the goal acceptance before publishing the
          // first feedback: the rclcpp_action client silently drops feedback
          // for goals whose handle it has not registered yet, which makes
          // immediate feedback delivery flaky.
          std::this_thread::sleep_for(200ms);
          for (const std::string &chunk : this->feedback_chunks_) {
            auto feedback =
                std::make_shared<GenerateChatCompletions::Feedback>();
            llama_msgs::msg::ChatChoiceChunk choice;
            choice.index = 0;
            choice.delta.role = "assistant";
            choice.delta.content = chunk;
            feedback->choices.push_back(choice);
            goal_handle->publish_feedback(feedback);
            std::this_thread::sleep_for(50ms);
          }
          std::this_thread::sleep_for(50ms);
          auto result = std::make_shared<GenerateChatCompletions::Result>();
          llama_msgs::msg::ChatChoice choice;
          choice.message.role = "assistant";
          choice.message.content = this->result_text_;
          result->choices.push_back(choice);
          result->usage.total_tokens = 3;
          if (this->abort_) {
            goal_handle->abort(result);
          } else {
            goal_handle->succeed(result);
          }
        });
  }

  GenerateChatCompletions::Goal get_last_goal() {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->last_goal_;
  }

private:
  rclcpp_action::Server<GenerateChatCompletions>::SharedPtr server_;
  std::vector<std::string> feedback_chunks_;
  std::string result_text_;
  bool abort_;
  std::mutex mutex_;
  GenerateChatCompletions::Goal last_goal_;
};

class ChatbotFsmTest : public ::testing::Test {
protected:
  void SetUp() override {
    this->server_node_ =
        std::make_shared<rclcpp::Node>("chatbot_test_stub_servers");
    this->executor_ =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    this->executor_->add_node(this->server_node_);
    this->spin_thread_ = std::thread([this]() { this->executor_->spin(); });
  }

  void TearDown() override {
    this->executor_->cancel();
    if (this->spin_thread_.joinable()) {
      this->spin_thread_.join();
    }
    // Drop the cached action clients and the yasmin client node so the next
    // test starts from a fresh discovery state. Otherwise the cached client
    // can still list the previous test's destroyed stub servers as
    // available; a goal sent in that window is written to a dead endpoint
    // and lost, leaving the state machine waiting forever for a result.
    yasmin_ros::ROSClientsCache::clear_all();
    yasmin_ros::YasminNode::destroy_instance();
  }

  rclcpp::Node::SharedPtr server_node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TEST_F(ChatbotFsmTest, ListenFsmHappyPath) {
  auto tts_server = std::make_shared<StubTtsServer>(this->server_node_);
  auto stt_server = std::make_shared<StubSttServer>(
      this->server_node_, std::vector<std::string>{"hello"});

  chatbot_ros::ListenFSM fsm;
  auto blackboard = yasmin::Blackboard::make_shared();
  const std::string outcome = fsm(blackboard);

  EXPECT_EQ(outcome, yasmin_ros::basic_outcomes::SUCCEED);
  ASSERT_TRUE(blackboard->contains("stt"));
  EXPECT_EQ(blackboard->get<std::string>("stt"), "hello");
  EXPECT_TRUE(tts_server->get_spoken_texts().empty());
}

TEST_F(ChatbotFsmTest, ListenFsmComplainsAndRetries) {
  auto tts_server = std::make_shared<StubTtsServer>(this->server_node_);
  auto stt_server = std::make_shared<StubSttServer>(
      this->server_node_, std::vector<std::string>{"", "hello"});

  chatbot_ros::ListenFSM fsm;
  auto blackboard = yasmin::Blackboard::make_shared();
  const std::string outcome = fsm(blackboard);

  EXPECT_EQ(outcome, yasmin_ros::basic_outcomes::SUCCEED);
  EXPECT_EQ(blackboard->get<std::string>("stt"), "hello");
  const auto spoken = tts_server->get_spoken_texts();
  ASSERT_EQ(spoken.size(), 1u);
  EXPECT_EQ(spoken[0], "I didn't understand youd. Can you repeat?");
}

TEST_F(ChatbotFsmTest, RespondingConcurrenceSpeaksStreamedSentences) {
  auto tts_server = std::make_shared<StubTtsServer>(this->server_node_);
  auto generate_server = std::make_shared<StubChatCompletionsServer>(
      this->server_node_, std::vector<std::string>{"", "Hi.", " Bye!"},
      "Hi. Bye!", false);

  auto pipeline = std::make_shared<chatbot_ros::SpeechPipeline>();
  auto concurrence = chatbot_ros::create_responding_concurrence(pipeline);

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<std::string>("stt", "What's up?");
  const std::string outcome = (*concurrence)(blackboard);

  EXPECT_EQ(outcome, yasmin_ros::basic_outcomes::SUCCEED);
  const auto spoken = tts_server->get_spoken_texts();
  ASSERT_EQ(spoken.size(), 2u);
  EXPECT_EQ(spoken[0], "Hi.");
  EXPECT_EQ(spoken[1], " Bye!");

  const GenerateChatCompletions::Goal goal = generate_server->get_last_goal();
  ASSERT_EQ(goal.messages.size(), 1u);
  EXPECT_EQ(goal.messages[0].role, "user");
  EXPECT_EQ(goal.messages[0].content, "What's up?");
  EXPECT_TRUE(goal.add_generation_prompt);
  EXPECT_TRUE(goal.use_jinja);
  EXPECT_TRUE(goal.stream);
  EXPECT_FLOAT_EQ(goal.sampling_config.temp, 0.0F);
}

TEST_F(ChatbotFsmTest, RespondingConcurrenceCanRunMultipleCycles) {
  auto tts_server = std::make_shared<StubTtsServer>(this->server_node_);
  auto generate_server = std::make_shared<StubChatCompletionsServer>(
      this->server_node_, std::vector<std::string>{"Hi."}, "Hi.", false);

  auto pipeline = std::make_shared<chatbot_ros::SpeechPipeline>();
  auto concurrence = chatbot_ros::create_responding_concurrence(pipeline);

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<std::string>("stt", "What's up?");

  EXPECT_EQ((*concurrence)(blackboard), yasmin_ros::basic_outcomes::SUCCEED);
  EXPECT_EQ((*concurrence)(blackboard), yasmin_ros::basic_outcomes::SUCCEED);

  const auto spoken = tts_server->get_spoken_texts();
  ASSERT_EQ(spoken.size(), 2u);
  EXPECT_EQ(spoken[0], "Hi.");
  EXPECT_EQ(spoken[1], "Hi.");
}

TEST_F(ChatbotFsmTest, RespondingConcurrenceAbortsWhenGenerationAborts) {
  auto tts_server = std::make_shared<StubTtsServer>(this->server_node_);
  auto generate_server = std::make_shared<StubChatCompletionsServer>(
      this->server_node_, std::vector<std::string>{"Hi."}, "Hi.", true);

  auto pipeline = std::make_shared<chatbot_ros::SpeechPipeline>();
  auto concurrence = chatbot_ros::create_responding_concurrence(pipeline);

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<std::string>("stt", "What's up?");
  const std::string outcome = (*concurrence)(blackboard);

  EXPECT_EQ(outcome, yasmin_ros::basic_outcomes::ABORT);
  EXPECT_EQ(generate_server->get_last_goal().messages.size(), 1u);
}

int main(int argc, char **argv) {
  // Run each test process on its own ROS domain (unless the caller picked
  // one). A fixed domain is not enough: two test processes running
  // concurrently on the same domain share action names, so a goal can be
  // served by the other process's stub server and feedback/results for the
  // same goal UUID reach both clients - producing spurious extra or missing
  // spoken sentences and occasional hangs. Deriving the domain from the PID
  // isolates concurrent runs. Must happen before rclcpp::init() so the RMW
  // picks the value up.
  if (getenv("ROS_DOMAIN_ID") == nullptr) {
    const int domain_id = 1 + (static_cast<int>(getpid()) % 229);
    setenv("ROS_DOMAIN_ID", std::to_string(domain_id).c_str(), 1);
  }
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  yasmin_ros::YasminNode::destroy_instance();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
