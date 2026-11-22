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

#include "chatbot_ros/text/transcript_filter.hpp"

#include <cctype>
#include <utility>

namespace chatbot_ros {
namespace text {

namespace {

const std::string WHITESPACE = " \t\n\r\f\v";

/// Removes every annotation delimited by `open`/`close` (e.g. `[music]`).
std::string remove_between(const std::string &text, char open, char close) {
  std::string result;
  bool inside = false;
  for (char c : text) {
    if (c == open) {
      inside = true;
      continue;
    }
    if (c == close) {
      inside = false;
      continue;
    }
    if (!inside) {
      result += c;
    }
  }
  return result;
}

std::string remove_between(const std::string &text, const std::string &open,
                           const std::string &close) {
  std::string result;
  bool inside = false;
  std::size_t pos = 0;
  while (pos < text.size()) {
    if (!inside && text.compare(pos, open.size(), open) == 0) {
      inside = true;
      pos += open.size();
      continue;
    }
    if (inside && text.compare(pos, close.size(), close) == 0) {
      inside = false;
      pos += close.size();
      continue;
    }
    if (!inside) {
      result += text[pos];
    }
    pos++;
  }
  return result;
}

std::string collapse(const std::string &text) {
  std::string result;
  bool last_space = true;
  for (char c : text) {
    if (std::isspace(static_cast<unsigned char>(c))) {
      if (!last_space) {
        result += ' ';
      }
      last_space = true;
    } else {
      result += c;
      last_space = false;
    }
  }
  const std::size_t begin = result.find_first_not_of(WHITESPACE);
  if (begin == std::string::npos) {
    return "";
  }
  const std::size_t end = result.find_last_not_of(WHITESPACE);
  return result.substr(begin, end - begin + 1);
}

/// Lower case, alphanumeric words separated by single spaces.
std::string normalize(const std::string &text) {
  std::string result;
  bool last_space = true;
  for (unsigned char c : text) {
    if (std::isalnum(c)) {
      result += static_cast<char>(std::tolower(c));
      last_space = false;
    } else if (!last_space) {
      result += ' ';
      last_space = true;
    }
  }
  const std::size_t end = result.find_last_not_of(' ');
  return (end == std::string::npos) ? "" : result.substr(0, end + 1);
}

std::size_t count_speakable(const std::string &text) {
  std::size_t count = 0;
  for (unsigned char c : text) {
    if (std::isalnum(c) || c >= 0x80) {
      count++;
    }
  }
  return count;
}

} // namespace

TranscriptFilter::TranscriptFilter(std::vector<std::string> ignored_phrases)
    : ignored_phrases_(std::move(ignored_phrases)) {
  for (std::string &phrase : this->ignored_phrases_) {
    phrase = normalize(phrase);
  }
}

std::string TranscriptFilter::clean(const std::string &raw) const {
  std::string result = raw;
  result = remove_between(result, '[', ']');
  result = remove_between(result, '(', ')');
  result = remove_between(result, '{', '}');
  result = remove_between(result, "*", "*");
  result = remove_between(result, "\u266a", "\u266a"); // ♪
  result = remove_between(result, "\u266b", "\u266b"); // ♫
  result = collapse(result);
  if (count_speakable(result) < 2) {
    return "";
  }
  return result;
}

std::string TranscriptFilter::clean_or_empty(const std::string &raw) const {
  const std::string cleaned = this->clean(raw);
  if (cleaned.empty()) {
    return "";
  }

  const std::string normalized = normalize(cleaned);
  if (normalized.size() < 2) {
    return "";
  }

  for (const std::string &phrase : this->ignored_phrases_) {
    if (phrase.empty()) {
      continue;
    }
    if (normalized == phrase) {
      return "";
    }
    // The utterance contains a known hallucination.
    if (normalized.find(phrase) != std::string::npos) {
      return "";
    }
    // "thank you" inside "thank you for watching" is still a hallucination.
    if (normalized.size() >= 8 &&
        phrase.find(normalized) != std::string::npos) {
      return "";
    }
  }
  return cleaned;
}

} // namespace text
} // namespace chatbot_ros
