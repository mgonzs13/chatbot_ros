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

#include "chatbot_ros/text/speech_sanitizer.hpp"

#include <cctype>
#include <cstdint>
#include <string>
#include <vector>

namespace chatbot_ros {
namespace text {

namespace {

const std::string WHITESPACE = " \t\n\r\f\v";

bool is_ascii_space(char c) {
  return std::isspace(static_cast<unsigned char>(c));
}

std::string replace_all(const std::string &text, const std::string &needle,
                        const std::string &replacement) {
  std::string result;
  std::size_t pos = 0;
  while (pos < text.size()) {
    const std::size_t found = text.find(needle, pos);
    if (found == std::string::npos) {
      result += text.substr(pos);
      break;
    }
    result += text.substr(pos, found - pos);
    result += replacement;
    pos = found + needle.size();
  }
  return result;
}

std::string trim(const std::string &text) {
  const std::size_t begin = text.find_first_not_of(WHITESPACE);
  if (begin == std::string::npos) {
    return "";
  }
  const std::size_t end = text.find_last_not_of(WHITESPACE);
  return text.substr(begin, end - begin + 1);
}

/// Removes markdown code fences (```...```) keeping no code content, since
/// code should never be read out loud.
std::string remove_code_fences(const std::string &text) {
  std::string result;
  std::size_t pos = 0;
  while (pos < text.size()) {
    const std::size_t fence = text.find("```", pos);
    if (fence == std::string::npos) {
      result += text.substr(pos);
      break;
    }
    result += text.substr(pos, fence - pos);
    // Skip the fence and the optional language tag on the same line.
    std::size_t line_end = text.find('\n', fence);
    if (line_end == std::string::npos) {
      break;
    }
    const std::size_t closing = text.find("```", line_end);
    if (closing == std::string::npos) {
      pos = line_end;
      continue;
    }
    pos = closing + 3;
  }
  return replace_all(result, "```", " ");
}

/// Replaces markdown links/images and bare URLs. Links keep their label,
/// images are dropped completely.
std::string remove_links(const std::string &text) {
  std::string result;
  std::size_t pos = 0;
  while (pos < text.size()) {
    const std::size_t bracket = text.find('[', pos);
    if (bracket == std::string::npos) {
      result += text.substr(pos);
      break;
    }

    const bool is_image = bracket > 0 && text[bracket - 1] == '!';
    const std::size_t open = is_image ? bracket - 1 : bracket;
    const std::size_t close = text.find(']', bracket);
    std::size_t paren_end = std::string::npos;
    if (close != std::string::npos && close + 1 < text.size() &&
        text[close + 1] == '(') {
      paren_end = text.find(')', close + 2);
    }

    if (paren_end == std::string::npos) {
      // Not a markdown link; keep the characters and continue.
      result += text.substr(pos, bracket - pos);
      if (!is_image) {
        result += '[';
      }
      pos = bracket + 1;
      continue;
    }

    if (open >= pos) {
      result += text.substr(pos, open - pos);
    } else if (!result.empty() && result.back() == '!') {
      // The image marker was already emitted in a previous iteration.
      result.pop_back();
    }
    if (!is_image) {
      result += " " + text.substr(bracket + 1, close - bracket - 1) + " ";
    }
    pos = paren_end + 1;
  }

  // Bare URLs.
  for (const std::string &prefix :
       {std::string("https://"), std::string("http://"), std::string("www.")}) {
    std::size_t start = result.find(prefix);
    while (start != std::string::npos) {
      std::size_t end = start;
      while (end < result.size() && !is_ascii_space(result[end])) {
        end++;
      }
      result.erase(start, end - start);
      start = result.find(prefix, start);
    }
  }
  return result;
}

/// True for code points that should never be read out loud: emoji,
/// dingbats, arrows, geometric shapes, variation selectors, etc.
bool is_symbol_code_point(uint32_t code_point) {
  return (code_point >= 0x200D && code_point <= 0x200D) ||   // ZWJ
         (code_point >= 0x20E3 && code_point <= 0x20E3) ||   // combining keycap
         (code_point >= 0x2190 && code_point <= 0x21FF) ||   // arrows
         (code_point >= 0x2300 && code_point <= 0x23FF) ||   // technical
         (code_point >= 0x25A0 && code_point <= 0x25FF) ||   // geometric shapes
         (code_point >= 0x2600 && code_point <= 0x27BF) ||   // misc symbols
         (code_point >= 0x2900 && code_point <= 0x297F) ||   // arrows
         (code_point >= 0x2B00 && code_point <= 0x2BFF) ||   // misc arrows
         (code_point >= 0x1F000 && code_point <= 0x1FAFF) || // emoji
         (code_point >= 0xFE00 && code_point <= 0xFE0F);     // variation sel.
}

std::string strip_symbols(const std::string &text) {
  std::string result;
  std::size_t i = 0;
  while (i < text.size()) {
    const unsigned char byte = static_cast<unsigned char>(text[i]);
    if (byte < 0x80) {
      result += static_cast<char>(byte);
      i++;
      continue;
    }

    // Decode a UTF-8 sequence.
    std::size_t length = 1;
    uint32_t code_point = 0;
    if ((byte & 0xE0) == 0xC0) {
      length = 2;
      code_point = byte & 0x1F;
    } else if ((byte & 0xF0) == 0xE0) {
      length = 3;
      code_point = byte & 0x0F;
    } else if ((byte & 0xF8) == 0xF0) {
      length = 4;
      code_point = byte & 0x07;
    } else {
      i++;
      continue; // Invalid lead byte, drop it.
    }

    if (i + length > text.size()) {
      break;
    }
    bool valid = true;
    for (std::size_t k = 1; k < length; k++) {
      const unsigned char continuation =
          static_cast<unsigned char>(text[i + k]);
      if ((continuation & 0xC0) != 0x80) {
        valid = false;
        break;
      }
      code_point = (code_point << 6) | (continuation & 0x3F);
    }
    if (!valid) {
      i++;
      continue;
    }

    if (!is_symbol_code_point(code_point)) {
      result += text.substr(i, length);
    }
    i += length;
  }
  return result;
}

/// Removes markdown emphasis and inline-code characters and rewrites
/// punctuation that reads badly aloud.
std::string strip_markup(const std::string &text) {
  std::string result;
  result.reserve(text.size());
  for (char c : text) {
    switch (c) {
    case '*':
    case '`':
    case '~':
      break;
    case '_':
      result += ' ';
      break;
    default:
      result += c;
      break;
    }
  }
  result = replace_all(result, "—", ", ");
  result = replace_all(result, "–", ", ");
  result = replace_all(result, "…", "...");
  return result;
}

/// Removes line prefixes such as headings, quotes and list markers and
/// joins all lines into a single one.
std::string flatten_lines(const std::string &text) {
  std::string result;
  std::size_t pos = 0;
  while (pos <= text.size()) {
    std::size_t newline = text.find('\n', pos);
    if (newline == std::string::npos) {
      newline = text.size();
    }
    std::string line = text.substr(pos, newline - pos);
    std::size_t begin = line.find_first_not_of(WHITESPACE);
    if (begin == std::string::npos) {
      pos = newline + 1;
      continue;
    }
    line = line.substr(begin);

    bool stripped = true;
    while (stripped && !line.empty()) {
      stripped = false;
      if (line[0] == '#') {
        line.erase(0, 1);
        stripped = true;
      } else if (line[0] == '>') {
        line.erase(0, 1);
        stripped = true;
      } else if (line.size() >= 2 &&
                 (line[0] == '-' || line[0] == '+' || line[0] == '•') &&
                 is_ascii_space(line[1])) {
        line.erase(0, 2);
        stripped = true;
      } else {
        // Numbered list marker: "1. ", "2) ".
        std::size_t digits = 0;
        while (digits < line.size() &&
               std::isdigit(static_cast<unsigned char>(line[digits]))) {
          digits++;
        }
        if (digits > 0 && digits + 1 < line.size() &&
            (line[digits] == '.' || line[digits] == ')') &&
            is_ascii_space(line[digits + 1])) {
          line.erase(0, digits + 2);
          stripped = true;
        }
      }
    }

    line = trim(line);
    if (!line.empty()) {
      if (!result.empty()) {
        result += ' ';
      }
      result += line;
    }
    pos = newline + 1;
  }
  return result;
}

std::string collapse_whitespace(const std::string &text) {
  std::string result;
  result.reserve(text.size());
  bool last_space = true;
  for (char c : text) {
    if (is_ascii_space(c)) {
      if (!last_space) {
        result += ' ';
      }
      last_space = true;
    } else {
      // Never keep a space in front of punctuation.
      if (last_space && !result.empty() &&
          (c == '.' || c == ',' || c == '!' || c == '?' || c == ';' ||
           c == ':')) {
        result.pop_back();
      }
      result += c;
      last_space = false;
    }
  }
  return trim(result);
}

bool has_speakable_content(const std::string &text) {
  for (unsigned char c : text) {
    if (std::isalnum(c) || c >= 0x80) {
      return true;
    }
  }
  return false;
}

} // namespace

std::string SpeechSanitizer::sanitize(const std::string &text) {
  std::string result = remove_code_fences(text);
  result = remove_links(result);
  result = strip_symbols(result);
  result = strip_markup(result);
  result = flatten_lines(result);
  result = collapse_whitespace(result);
  if (!has_speakable_content(result)) {
    return "";
  }
  return result;
}

} // namespace text
} // namespace chatbot_ros
