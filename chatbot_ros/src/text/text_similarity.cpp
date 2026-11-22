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

#include "chatbot_ros/text/text_similarity.hpp"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

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

double subsequence_similarity(const std::string &candidate,
                              const std::string &reference) {
  const std::vector<std::string> candidate_tokens = tokenize(candidate);
  const std::vector<std::string> reference_tokens = tokenize(reference);
  if (candidate_tokens.empty() || reference_tokens.empty()) {
    return 0.0;
  }

  // Longest common subsequence over tokens.
  std::vector<std::size_t> previous(reference_tokens.size() + 1, 0);
  std::vector<std::size_t> current(reference_tokens.size() + 1, 0);
  for (const std::string &candidate_token : candidate_tokens) {
    for (std::size_t j = 1; j <= reference_tokens.size(); j++) {
      if (candidate_token == reference_tokens[j - 1]) {
        current[j] = previous[j - 1] + 1;
      } else {
        current[j] = std::max(previous[j], current[j - 1]);
      }
    }
    std::swap(previous, current);
    std::fill(current.begin(), current.end(), 0);
  }

  return static_cast<double>(previous[reference_tokens.size()]) /
         static_cast<double>(candidate_tokens.size());
}

} // namespace text
} // namespace chatbot_ros
