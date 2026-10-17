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

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <functional>
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
#include "chatbot_ros/chatbot_config.hpp"
#include "chatbot_ros/conversation_history.hpp"
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
  StubTtsServer(const rclcpp::Node::SharedPtr &node,
                std::chrono::milliseconds delay = 0ms)
      : delay_(delay) {
    // Reentrant groups so blocking stub executions (speaking, listening)
    // do not serialize with each other.
    this->group_ =
        node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
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
          {
            std::lock_guard<std::mutex> lock(this->mutex_);
            this->spoken_texts_.push_back(goal_handle->get_goal()->text);
          }
          const auto deadline = std::chrono::steady_clock::now() + this->delay_;
          while (std::chrono::steady_clock::now() < deadline) {
            if (goal_handle->is_canceling()) {
              std::lock_guard<std::mutex> lock(this->mutex_);
              this->canceled_count_++;
              auto result = std::make_shared<TTS::Result>();
              result->text = goal_handle->get_goal()->text;
              goal_handle->canceled(result);
              return;
            }
            std::this_thread::sleep_for(20ms);
          }
          auto result = std::make_shared<TTS::Result>();
          result->text = goal_handle->get_goal()->text;
          goal_handle->succeed(result);
        },
        rcl_action_server_get_default_options(), this->group_);
  }

  std::vector<std::string> get_spoken_texts() {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->spoken_texts_;
  }

  int get_canceled_count() {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->canceled_count_;
  }

private:
  rclcpp_action::Server<TTS>::SharedPtr server_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::chrono::milliseconds delay_;
  std::mutex mutex_;
  std::vector<std::string> spoken_texts_;
  int canceled_count_{0};
};

class StubSttServer {
public:
  StubSttServer(const rclcpp::Node::SharedPtr &node,
                std::vector<std::string> transcriptions,
                std::chrono::milliseconds delay = 0ms,
                std::function<bool()> gate = nullptr, int reject_goals = 0)
      : delay_(delay), gate_(std::move(gate)), rejections_left_(reject_goals),
        transcriptions_(transcriptions.begin(), transcriptions.end()) {
    this->group_ =
        node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    this->server_ = rclcpp_action::create_server<STT>(
        node, "/whisper/listen",
        [this](const rclcpp_action::GoalUUID &,
               std::shared_ptr<const STT::Goal>) {
          if (this->rejections_left_ > 0) {
            this->rejections_left_--;
            return rclcpp_action::GoalResponse::REJECT;
          }
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<STT>>) {
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](
            std::shared_ptr<rclcpp_action::ServerGoalHandle<STT>> goal_handle) {
          // Optional gate: hold the transcription until another stub is
          // ready (e.g. the robot started speaking).
          const auto gate_deadline = std::chrono::steady_clock::now() + 30s;
          while (this->gate_ && !this->gate_()) {
            if (goal_handle->is_canceling()) {
              this->cancel_goal(goal_handle);
              return;
            }
            if (std::chrono::steady_clock::now() > gate_deadline) {
              auto result = std::make_shared<STT::Result>();
              goal_handle->abort(result);
              return;
            }
            std::this_thread::sleep_for(20ms);
          }

          std::string transcription = "";
          {
            std::lock_guard<std::mutex> lock(this->mutex_);
            if (!this->transcriptions_.empty()) {
              transcription = this->transcriptions_.front();
              this->transcriptions_.pop_front();
            }
          }

          const auto deadline = std::chrono::steady_clock::now() + this->delay_;
          while (std::chrono::steady_clock::now() < deadline) {
            if (goal_handle->is_canceling()) {
              this->cancel_goal(goal_handle);
              return;
            }
            std::this_thread::sleep_for(20ms);
          }

          auto result = std::make_shared<STT::Result>();
          result->transcription.text = transcription;
          goal_handle->succeed(result);
        },
        rcl_action_server_get_default_options(), this->group_);
  }

  int get_canceled_count() {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->canceled_count_;
  }

private:
  void cancel_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<STT>>
                       &goal_handle) {
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->canceled_count_++;
    }
    auto result = std::make_shared<STT::Result>();
    goal_handle->canceled(result);
  }

  rclcpp_action::Server<STT>::SharedPtr server_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::chrono::milliseconds delay_;
  std::function<bool()> gate_;
  int rejections_left_{0};
  std::mutex mutex_;
  std::deque<std::string> transcriptions_;
  int canceled_count_{0};
};

class StubChatCompletionsServer {
public:
  StubChatCompletionsServer(const rclcpp::Node::SharedPtr &node,
                            std::vector<std::string> feedback_chunks,
                            std::string result_text, bool abort)
      : feedback_chunks_(std::move(feedback_chunks)),
        result_text_(std::move(result_text)), abort_(abort) {
    this->group_ =
        node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
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
            this->goals_.push_back(*goal_handle->get_goal());
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
        },
        rcl_action_server_get_default_options(), this->group_);
  }

  GenerateChatCompletions::Goal get_last_goal() {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->goals_.empty() ? GenerateChatCompletions::Goal()
                                : this->goals_.back();
  }

  std::size_t get_goal_count() {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->goals_.size();
  }

private:
  rclcpp_action::Server<GenerateChatCompletions>::SharedPtr server_;
  rclcpp::CallbackGroup::SharedPtr group_;
  std::vector<std::string> feedback_chunks_;
  std::string result_text_;
  bool abort_;
  std::mutex mutex_;
  std::vector<GenerateChatCompletions::Goal> goals_;
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
    // Stop the executor before destroying the stub servers: destroying a
    // waitable while the executor is waiting on it corrupts the wait set.
    this->executor_->cancel();
    if (this->spin_thread_.joinable()) {
      this->spin_thread_.join();
    }
    this->tts_server_.reset();
    this->stt_server_.reset();
    this->generate_server_.reset();
    // Drop the cached action clients and the yasmin client node so the next
    // test starts from a fresh discovery state. Otherwise the cached client
    // can still list the previous test's destroyed stub servers as
    // available; a goal sent in that window is written to a dead endpoint
    // and lost, leaving the state machine waiting forever for a result.
    yasmin_ros::ROSClientsCache::clear_all();
    yasmin_ros::YasminNode::destroy_instance();
    this->executor_.reset();
    this->server_node_.reset();
  }

  std::shared_ptr<StubTtsServer>
  make_tts_server(std::chrono::milliseconds delay = 0ms) {
    this->tts_server_ =
        std::make_shared<StubTtsServer>(this->server_node_, delay);
    return this->tts_server_;
  }

  std::shared_ptr<StubSttServer>
  make_stt_server(std::vector<std::string> transcriptions,
                  std::chrono::milliseconds delay = 0ms,
                  std::function<bool()> gate = nullptr, int reject_goals = 0) {
    this->stt_server_ = std::make_shared<StubSttServer>(
        this->server_node_, std::move(transcriptions), delay, std::move(gate),
        reject_goals);
    return this->stt_server_;
  }

  std::shared_ptr<StubChatCompletionsServer>
  make_generate_server(std::vector<std::string> feedback_chunks,
                       std::string result_text, bool abort) {
    this->generate_server_ = std::make_shared<StubChatCompletionsServer>(
        this->server_node_, std::move(feedback_chunks), std::move(result_text),
        abort);
    return this->generate_server_;
  }

  rclcpp::Node::SharedPtr server_node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::shared_ptr<StubTtsServer> tts_server_;
  std::shared_ptr<StubSttServer> stt_server_;
  std::shared_ptr<StubChatCompletionsServer> generate_server_;
};

TEST_F(ChatbotFsmTest, ListenFsmHappyPath) {
  auto tts_server = this->make_tts_server();
  auto stt_server = this->make_stt_server(std::vector<std::string>{"hello"});

  chatbot_ros::ListenFSM fsm;
  auto blackboard = yasmin::Blackboard::make_shared();
  const std::string outcome = fsm(blackboard);

  EXPECT_EQ(outcome, yasmin_ros::basic_outcomes::SUCCEED);
  ASSERT_TRUE(blackboard->contains("stt"));
  EXPECT_EQ(blackboard->get<std::string>("stt"), "hello");
  EXPECT_TRUE(tts_server->get_spoken_texts().empty());
}

TEST_F(ChatbotFsmTest, ListenFsmFiltersNoiseTranscriptions) {
  auto tts_server = this->make_tts_server();
  // "[BLANK_AUDIO]" is removed by the structural filter and the hallucinated
  // phrase by the ignored phrases list; both must end up as "no speech".
  auto stt_server = this->make_stt_server(std::vector<std::string>{
      "[BLANK_AUDIO]", "Thank you for watching!", "hello robot"});

  chatbot_ros::ListenFSM fsm;
  auto blackboard = yasmin::Blackboard::make_shared();
  const std::string outcome = fsm(blackboard);

  EXPECT_EQ(outcome, yasmin_ros::basic_outcomes::SUCCEED);
  EXPECT_EQ(blackboard->get<std::string>("stt"), "hello robot");
  const auto spoken = tts_server->get_spoken_texts();
  ASSERT_EQ(spoken.size(), 2u);
}

TEST_F(ChatbotFsmTest, ListenFsmRecoversFromRejectedGoals) {
  // The whisper server rejects a goal while it finishes canceling the
  // previous one (which happens right after an interruption).
  auto tts_server = this->make_tts_server();
  auto stt_server =
      this->make_stt_server(std::vector<std::string>{"hello"}, 0ms, nullptr, 2);

  chatbot_ros::ListenFSM fsm;
  auto blackboard = yasmin::Blackboard::make_shared();
  const std::string outcome = fsm(blackboard);

  EXPECT_EQ(outcome, yasmin_ros::basic_outcomes::SUCCEED);
  EXPECT_EQ(blackboard->get<std::string>("stt"), "hello");
  EXPECT_TRUE(tts_server->get_spoken_texts().empty());
}

TEST_F(ChatbotFsmTest, ListenFsmComplainsAndRetries) {
  auto tts_server = this->make_tts_server();
  auto stt_server =
      this->make_stt_server(std::vector<std::string>{"", "hello"});

  chatbot_ros::ListenFSM fsm;
  auto blackboard = yasmin::Blackboard::make_shared();
  const std::string outcome = fsm(blackboard);

  EXPECT_EQ(outcome, yasmin_ros::basic_outcomes::SUCCEED);
  EXPECT_EQ(blackboard->get<std::string>("stt"), "hello");
  const auto spoken = tts_server->get_spoken_texts();
  ASSERT_EQ(spoken.size(), 1u);
  EXPECT_EQ(spoken[0], "Sorry, I didn't hear anything. Could you repeat that?");
}

TEST_F(ChatbotFsmTest, RespondingConcurrenceSpeaksStreamedSentences) {
  auto tts_server = this->make_tts_server();
  auto generate_server = this->make_generate_server(
      std::vector<std::string>{"", "Hi.", " Bye!"}, "Hi. Bye!", false);

  auto pipeline = std::make_shared<chatbot_ros::SpeechPipeline>();
  auto concurrence = chatbot_ros::create_responding_concurrence(pipeline);

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<std::string>("stt", "What's up?");
  const std::string outcome = (*concurrence)(blackboard);

  EXPECT_EQ(outcome, yasmin_ros::basic_outcomes::SUCCEED);
  const auto spoken = tts_server->get_spoken_texts();
  ASSERT_EQ(spoken.size(), 2u);
  EXPECT_EQ(spoken[0], "Hi.");
  EXPECT_EQ(spoken[1], "Bye!");

  const GenerateChatCompletions::Goal goal = generate_server->get_last_goal();
  ASSERT_EQ(goal.messages.size(), 2u);
  EXPECT_EQ(goal.messages[0].role, "system");
  EXPECT_FALSE(goal.messages[0].content.empty());
  EXPECT_EQ(goal.messages[1].role, "user");
  EXPECT_EQ(goal.messages[1].content, "What's up?");
  EXPECT_TRUE(goal.add_generation_prompt);
  EXPECT_TRUE(goal.use_jinja);
  EXPECT_TRUE(goal.stream);
  EXPECT_FLOAT_EQ(goal.sampling_config.temp, 0.4F);
}

TEST_F(ChatbotFsmTest, RespondingConcurrenceSanitizesMarkdownSpeech) {
  auto tts_server = this->make_tts_server();
  auto generate_server = this->make_generate_server(
      std::vector<std::string>{"Sure, use **bold** ", "and `code` here."},
      "Sure, use **bold** and `code` here.", false);

  auto pipeline = std::make_shared<chatbot_ros::SpeechPipeline>();
  auto concurrence = chatbot_ros::create_responding_concurrence(pipeline);

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<std::string>("stt", "How do I format code?");
  EXPECT_EQ((*concurrence)(blackboard), yasmin_ros::basic_outcomes::SUCCEED);

  const auto spoken = tts_server->get_spoken_texts();
  ASSERT_EQ(spoken.size(), 1u);
  EXPECT_EQ(spoken[0], "Sure, use bold and code here.");
}

TEST_F(ChatbotFsmTest, RespondingConcurrenceKeepsConversationHistory) {
  auto tts_server = this->make_tts_server();
  auto generate_server =
      this->make_generate_server(std::vector<std::string>{"Hi."}, "Hi.", false);

  auto history = std::make_shared<chatbot_ros::ConversationHistory>();
  auto pipeline = std::make_shared<chatbot_ros::SpeechPipeline>();
  auto concurrence = chatbot_ros::create_responding_concurrence(
      pipeline, chatbot_ros::ChatbotConfig(), history);

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<std::string>("stt", "What's up?");
  EXPECT_EQ((*concurrence)(blackboard), yasmin_ros::basic_outcomes::SUCCEED);
  EXPECT_EQ((*concurrence)(blackboard), yasmin_ros::basic_outcomes::SUCCEED);

  const auto spoken = tts_server->get_spoken_texts();
  ASSERT_EQ(spoken.size(), 2u);
  EXPECT_EQ(spoken[0], "Hi.");
  EXPECT_EQ(spoken[1], "Hi.");

  const GenerateChatCompletions::Goal goal = generate_server->get_last_goal();
  ASSERT_EQ(goal.messages.size(), 4u);
  EXPECT_EQ(goal.messages[0].role, "system");
  EXPECT_EQ(goal.messages[1].role, "user");
  EXPECT_EQ(goal.messages[1].content, "What's up?");
  EXPECT_EQ(goal.messages[2].role, "assistant");
  EXPECT_EQ(goal.messages[2].content, "Hi.");
  EXPECT_EQ(goal.messages[3].role, "user");
}

TEST_F(ChatbotFsmTest, RespondingConcurrenceAbortsWhenGenerationAborts) {
  auto tts_server = this->make_tts_server();
  auto generate_server =
      this->make_generate_server(std::vector<std::string>{"Hi."}, "Hi.", true);

  auto pipeline = std::make_shared<chatbot_ros::SpeechPipeline>();
  auto concurrence = chatbot_ros::create_responding_concurrence(pipeline);

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<std::string>("stt", "What's up?");
  const std::string outcome = (*concurrence)(blackboard);

  EXPECT_EQ(outcome, yasmin_ros::basic_outcomes::ABORT);
  EXPECT_EQ(generate_server->get_last_goal().messages.size(), 2u);
}

TEST_F(ChatbotFsmTest, RespondingConcurrenceReportsInterruption) {
  // The TTS server keeps "speaking" until the interaction is canceled.
  auto tts_server = this->make_tts_server(10s);
  // The transcription is only released once the robot started speaking, so
  // the test does not depend on action server discovery timing.
  auto stt_server = this->make_stt_server(
      std::vector<std::string>{"actually stop"}, 0ms,
      [tts_server]() { return !tts_server->get_spoken_texts().empty(); });
  auto generate_server = this->make_generate_server(
      std::vector<std::string>{
          "Once upon a time there was a little robot in a factory."},
      "Once upon a time there was a little robot in a factory.", false);

  auto pipeline = std::make_shared<chatbot_ros::SpeechPipeline>();
  auto concurrence = chatbot_ros::create_responding_concurrence(pipeline);

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<std::string>("stt", "tell me a long story");
  const std::string outcome = (*concurrence)(blackboard);

  EXPECT_EQ(outcome, chatbot_ros::INTERRUPTED_OUTCOME);
  ASSERT_TRUE(blackboard->contains("stt"));
  EXPECT_EQ(blackboard->get<std::string>("stt"), "actually stop");
  EXPECT_GE(tts_server->get_canceled_count(), 1);
}

TEST_F(ChatbotFsmTest, RespondingConcurrenceIgnoresOwnVoiceEcho) {
  // The robot speaks and the microphone picks it up: Whisper transcribes
  // the robot's own sentence. That must not interrupt the response.
  const std::string sentence =
      "Once upon a time there was a little robot in a factory.";
  auto tts_server = this->make_tts_server(1500ms);
  auto stt_server = this->make_stt_server(
      std::vector<std::string>{sentence}, 0ms,
      [tts_server]() { return !tts_server->get_spoken_texts().empty(); });
  auto generate_server = this->make_generate_server(
      std::vector<std::string>{sentence}, sentence, false);

  auto pipeline = std::make_shared<chatbot_ros::SpeechPipeline>();
  auto concurrence = chatbot_ros::create_responding_concurrence(pipeline);

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<std::string>("stt", "tell me a story");
  const std::string outcome = (*concurrence)(blackboard);

  EXPECT_EQ(outcome, yasmin_ros::basic_outcomes::SUCCEED);
  EXPECT_EQ(blackboard->get<std::string>("stt"), "tell me a story");
  EXPECT_EQ(tts_server->get_canceled_count(), 0);
}

TEST_F(ChatbotFsmTest, RespondingConcurrenceStopsListenerWhenSpeakingEnds) {
  auto tts_server = this->make_tts_server(700ms);
  auto stt_server =
      this->make_stt_server(std::vector<std::string>{"hello there"}, 10s);
  auto generate_server =
      this->make_generate_server(std::vector<std::string>{"Hi."}, "Hi.", false);

  auto pipeline = std::make_shared<chatbot_ros::SpeechPipeline>();
  auto concurrence = chatbot_ros::create_responding_concurrence(pipeline);

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<std::string>("stt", "What's up?");
  const std::string outcome = (*concurrence)(blackboard);

  EXPECT_EQ(outcome, yasmin_ros::basic_outcomes::SUCCEED);
  // The listener never interrupted, so the original transcript is kept.
  EXPECT_EQ(blackboard->get<std::string>("stt"), "What's up?");
  const auto spoken = tts_server->get_spoken_texts();
  ASSERT_EQ(spoken.size(), 1u);
  EXPECT_EQ(spoken[0], "Hi.");
}

TEST_F(ChatbotFsmTest, RespondingConcurrenceInterruptDisabled) {
  auto tts_server = this->make_tts_server();
  auto generate_server =
      this->make_generate_server(std::vector<std::string>{"Hi."}, "Hi.", false);

  chatbot_ros::ChatbotConfig config;
  config.interrupt_enabled = false;
  config.system_prompt = "";
  config.temperature = 0.0;

  auto pipeline = std::make_shared<chatbot_ros::SpeechPipeline>();
  auto concurrence =
      chatbot_ros::create_responding_concurrence(pipeline, config);

  auto blackboard = yasmin::Blackboard::make_shared();
  blackboard->set<std::string>("stt", "hello");
  const std::string outcome = (*concurrence)(blackboard);

  EXPECT_EQ(outcome, yasmin_ros::basic_outcomes::SUCCEED);
  const GenerateChatCompletions::Goal goal = generate_server->get_last_goal();
  ASSERT_EQ(goal.messages.size(), 1u);
  EXPECT_EQ(goal.messages[0].role, "user");
  EXPECT_FLOAT_EQ(goal.sampling_config.temp, 0.0F);
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
