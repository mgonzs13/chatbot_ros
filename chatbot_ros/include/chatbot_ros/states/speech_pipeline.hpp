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
