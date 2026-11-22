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

#ifndef CHATBOT_ROS__TEXT__SPEECH_SANITIZER_HPP_
#define CHATBOT_ROS__TEXT__SPEECH_SANITIZER_HPP_

#include <string>

namespace chatbot_ros {
namespace text {

/// Turns an LLM response into plain, natural text suitable for TTS.
///
/// The sanitizer removes markdown structure (headings, lists, emphasis,
/// code fences, links), emoji and other symbols and collapses the result
/// into a single whitespace-normalized line.
class SpeechSanitizer {
public:
  /// Returns the sanitized text, or an empty string when nothing
  /// speakable is left.
  static std::string sanitize(const std::string &text);
};

} // namespace text
} // namespace chatbot_ros

#endif // CHATBOT_ROS__TEXT__SPEECH_SANITIZER_HPP_
