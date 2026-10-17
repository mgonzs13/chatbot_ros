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
