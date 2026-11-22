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

#ifndef CHATBOT_ROS__TEXT__TEXT_SIMILARITY_HPP_
#define CHATBOT_ROS__TEXT__TEXT_SIMILARITY_HPP_

#include <string>
#include <vector>

namespace chatbot_ros {
namespace text {

/// Splits text into lower case alphanumeric words.
std::vector<std::string> tokenize(const std::string &text);

/// Similarity in [0, 1] between two texts, based on word overlap.
///
/// Takes the maximum of the Jaccard index and the fraction of `candidate`
/// words that also appear in `reference`, so a short candidate contained in
/// a longer reference scores high.
double similarity(const std::string &candidate, const std::string &reference);

/// Longest common subsequence ratio in [0, 1]: the fraction of
/// `candidate` words that also appear in `reference` in the same order.
///
/// Unlike `similarity`, word order matters and extra inserted or missing
/// words are tolerated, which makes it well suited to compare a Whisper
/// transcription with the sentence it was transcribed from.
double subsequence_similarity(const std::string &candidate,
                              const std::string &reference);

} // namespace text
} // namespace chatbot_ros

#endif // CHATBOT_ROS__TEXT__TEXT_SIMILARITY_HPP_
