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

#ifndef CHATBOT_ROS__STATES__SPEECH_PIPELINE_HPP_
#define CHATBOT_ROS__STATES__SPEECH_PIPELINE_HPP_

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

namespace chatbot_ros {

class SpeechPipeline {
public:
  using SharedPtr = std::shared_ptr<SpeechPipeline>;

  void push(const std::string &sentence);
  void close(const std::string &generation_outcome);
  void cancel();

  /// Clears queue/flags/outcome so the pipeline can be reused for a new
  /// generation cycle. Must only be called when no other thread is using it.
  void reset();

  bool pop(std::string &sentence);

  bool is_closed() const;
  bool is_canceled() const;
  std::string get_generation_outcome() const;

private:
  mutable std::mutex mutex_;
  std::condition_variable cond_;
  std::deque<std::string> queue_;
  bool closed_{false};
  bool canceled_{false};
  std::string generation_outcome_;
};

} // namespace chatbot_ros

#endif // CHATBOT_ROS__STATES__SPEECH_PIPELINE_HPP_
