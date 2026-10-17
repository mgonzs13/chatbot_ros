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
