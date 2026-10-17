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
