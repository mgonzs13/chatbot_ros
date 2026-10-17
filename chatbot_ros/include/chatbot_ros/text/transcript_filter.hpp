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

#ifndef CHATBOT_ROS__TEXT__TRANSCRIPT_FILTER_HPP_
#define CHATBOT_ROS__TEXT__TRANSCRIPT_FILTER_HPP_

#include <string>
#include <vector>

namespace chatbot_ros {
namespace text {

/// Cleans up Whisper transcriptions that do not contain real speech.
///
/// Whisper emits annotations like `[BLANK_AUDIO]`, `(music)`, `♪ ... ♪` or
/// hallucinated phrases (`Thank you for watching!`) when it processes
/// silence or noise. This filter removes those artifacts and reports
/// whether anything speakable is left.
class TranscriptFilter {
public:
  explicit TranscriptFilter(
      std::vector<std::string> ignored_phrases = std::vector<std::string>());

  /// Returns the cleaned transcription, or an empty string when the raw
  /// transcription is noise or a hallucination. This is what callers
  /// should use.
  std::string clean_or_empty(const std::string &raw) const;

  /// Removes non-speech annotations and normalizes whitespace, without
  /// applying the ignored phrases. Mostly useful for debugging and tests.
  std::string clean(const std::string &raw) const;

private:
  std::vector<std::string> ignored_phrases_;
};

} // namespace text
} // namespace chatbot_ros

#endif // CHATBOT_ROS__TEXT__TRANSCRIPT_FILTER_HPP_
