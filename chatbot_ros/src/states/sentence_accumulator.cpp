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

#include "chatbot_ros/states/sentence_accumulator.hpp"

namespace chatbot_ros {

namespace {
const std::string WHITESPACE = " \t\n\r\f\v";
} // namespace

bool SentenceAccumulator::ends_with_punctuation(const std::string &text) {
  const std::size_t end = text.find_last_not_of(WHITESPACE);
  if (end == std::string::npos) {
    return false;
  }
  const char last = text[end];
  return last == '.' || last == '!' || last == '?' || last == ':';
}

std::vector<std::string> SentenceAccumulator::add(const std::string &text) {
  this->buffer_ += text;
  std::vector<std::string> sentences;
  if (ends_with_punctuation(this->buffer_)) {
    sentences.push_back(this->buffer_);
    this->buffer_.clear();
  }
  return sentences;
}

std::optional<std::string> SentenceAccumulator::flush() {
  if (this->buffer_.find_first_not_of(WHITESPACE) == std::string::npos) {
    this->buffer_.clear();
    return std::nullopt;
  }
  std::string leftover = this->buffer_;
  this->buffer_.clear();
  return leftover;
}

void SentenceAccumulator::clear() { this->buffer_.clear(); }

} // namespace chatbot_ros
