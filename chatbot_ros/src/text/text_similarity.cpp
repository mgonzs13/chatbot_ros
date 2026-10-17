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

#include "chatbot_ros/text/text_similarity.hpp"

#include <algorithm>
#include <cctype>
#include <unordered_map>

namespace chatbot_ros {
namespace text {

std::vector<std::string> tokenize(const std::string &text) {
  std::vector<std::string> tokens;
  std::string current;
  for (unsigned char c : text) {
    if (std::isalnum(c) || c >= 0x80) {
      current += static_cast<char>(std::tolower(c));
    } else if (!current.empty()) {
      tokens.push_back(current);
      current.clear();
    }
  }
  if (!current.empty()) {
    tokens.push_back(current);
  }
  return tokens;
}

double similarity(const std::string &candidate, const std::string &reference) {
  const std::vector<std::string> candidate_tokens = tokenize(candidate);
  const std::vector<std::string> reference_tokens = tokenize(reference);
  if (candidate_tokens.empty() || reference_tokens.empty()) {
    return 0.0;
  }

  std::unordered_map<std::string, std::size_t> reference_counts;
  for (const std::string &token : reference_tokens) {
    reference_counts[token]++;
  }

  std::size_t intersection = 0;
  std::unordered_map<std::string, std::size_t> remaining = reference_counts;
  for (const std::string &token : candidate_tokens) {
    auto it = remaining.find(token);
    if (it != remaining.end() && it->second > 0) {
      it->second--;
      intersection++;
    }
  }

  const double containment = static_cast<double>(intersection) /
                             static_cast<double>(candidate_tokens.size());
  const double jaccard =
      static_cast<double>(intersection) /
      static_cast<double>(candidate_tokens.size() + reference_tokens.size() -
                          intersection);
  return std::max(containment, jaccard);
}

} // namespace text
} // namespace chatbot_ros
