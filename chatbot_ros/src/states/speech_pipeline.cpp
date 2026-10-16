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
