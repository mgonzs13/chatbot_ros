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
