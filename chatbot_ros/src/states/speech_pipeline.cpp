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

#include "chatbot_ros/states/speech_pipeline.hpp"

namespace chatbot_ros {

void SpeechPipeline::push(const std::string &sentence) {
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->closed_ || this->canceled_) {
      return;
    }
    this->queue_.push_back(sentence);
  }
  this->cond_.notify_all();
}

void SpeechPipeline::close(const std::string &generation_outcome) {
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->closed_) {
      return;
    }
    this->closed_ = true;
    this->generation_outcome_ = generation_outcome;
  }
  this->cond_.notify_all();
}

void SpeechPipeline::cancel() {
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->canceled_ = true;
  }
  this->cond_.notify_all();
}

void SpeechPipeline::reset() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->queue_.clear();
  this->closed_ = false;
  this->canceled_ = false;
  this->generation_outcome_.clear();
}

bool SpeechPipeline::pop(std::string &sentence) {
  std::unique_lock<std::mutex> lock(this->mutex_);
  this->cond_.wait(lock, [this]() {
    return this->canceled_ || !this->queue_.empty() || this->closed_;
  });
  if (this->canceled_) {
    return false;
  }
  if (!this->queue_.empty()) {
    sentence = this->queue_.front();
    this->queue_.pop_front();
    return true;
  }
  return false;
}

bool SpeechPipeline::is_closed() const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->closed_;
}

bool SpeechPipeline::is_canceled() const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->canceled_;
}

std::string SpeechPipeline::get_generation_outcome() const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->generation_outcome_;
}

} // namespace chatbot_ros
