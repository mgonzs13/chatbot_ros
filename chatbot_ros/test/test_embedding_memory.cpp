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

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <unistd.h>
#include <vector>

#include <nlohmann/json.hpp>

#include "chatbot_ros/chatbot_config.hpp"
#include "chatbot_ros/embedding_memory.hpp"
#include "chatbot_ros/text/text_similarity.hpp"

using chatbot_ros::ChatbotConfig;
using chatbot_ros::EmbeddingMemory;

namespace {

/// Tiny bag-of-words embedder so the tests do not need a model server.
std::optional<std::vector<float>> fake_embedder(const std::string &text) {
  const std::vector<std::string> vocabulary = {"hello", "robot", "weather",
                                               "music", "football"};
  std::vector<float> embedding(vocabulary.size(), 0.0F);
  for (const std::string &word : chatbot_ros::text::tokenize(text)) {
    for (std::size_t i = 0; i < vocabulary.size(); i++) {
      if (word == vocabulary[i]) {
        embedding[i] += 1.0F;
      }
    }
  }
  return embedding;
}

} // namespace

class EmbeddingMemoryTest : public ::testing::Test {
protected:
  void SetUp() override {
    this->storage_file_ =
        std::filesystem::temp_directory_path() /
        ("chatbot_memory_test_" + std::to_string(getpid()) + "_" +
         ::testing::UnitTest::GetInstance()->current_test_info()->name() +
         ".json");
    std::filesystem::remove(this->storage_file_);
  }

  void TearDown() override { std::filesystem::remove(this->storage_file_); }

  ChatbotConfig make_config() {
    ChatbotConfig config;
    config.memory_enabled = true;
    config.memory_storage_file = this->storage_file_.string();
    config.memory_retrieve_k = 2;
    config.memory_min_similarity = 0.5;
    config.memory_max_entries = 3;
    return config;
  }

  std::filesystem::path storage_file_;
};

TEST_F(EmbeddingMemoryTest, RetrievesSimilarTurns) {
  EmbeddingMemory memory(this->make_config(), nullptr);
  memory.set_embedder(fake_embedder);

  memory.remember_turn("hello robot", "hello human");
  memory.remember_turn("what is the weather", "it is sunny");

  const std::vector<std::string> memories = memory.retrieve("robot hello");
  ASSERT_FALSE(memories.empty());
  EXPECT_NE(memories[0].find("hello robot"), std::string::npos);
  EXPECT_EQ(memories[0].find("weather"), std::string::npos);
}

TEST_F(EmbeddingMemoryTest, SkipsUnrelatedTurns) {
  EmbeddingMemory memory(this->make_config(), nullptr);
  memory.set_embedder(fake_embedder);

  memory.remember_turn("what is the weather", "it is sunny");
  EXPECT_TRUE(memory.retrieve("hello robot").empty());
}

TEST_F(EmbeddingMemoryTest, PersistsAndReloadsMemories) {
  {
    EmbeddingMemory memory(this->make_config(), nullptr);
    memory.set_embedder(fake_embedder);
    memory.remember_turn("hello robot", "hello human");
  }

  EmbeddingMemory reloaded(this->make_config(), nullptr);
  reloaded.set_embedder(fake_embedder);
  const std::vector<std::string> memories = reloaded.retrieve("hello robot");
  ASSERT_EQ(memories.size(), 1u);
  EXPECT_NE(memories[0].find("hello human"), std::string::npos);
}

TEST_F(EmbeddingMemoryTest, PrunesOldestEntries) {
  EmbeddingMemory memory(this->make_config(), nullptr);
  memory.set_embedder(fake_embedder);

  memory.remember_turn("hello robot", "one");
  memory.remember_turn("hello weather", "two");
  memory.remember_turn("hello music", "three");
  memory.remember_turn("hello football", "four");

  std::ifstream file(this->storage_file_);
  ASSERT_TRUE(file.good());
  nlohmann::json data;
  file >> data;
  EXPECT_EQ(data["entries"].size(), 3u);
  // The oldest turn ("hello robot") must have been dropped.
  EXPECT_EQ(data.dump().find("one"), std::string::npos);
}

TEST_F(EmbeddingMemoryTest, DisabledMemoryDoesNothing) {
  ChatbotConfig config = this->make_config();
  config.memory_enabled = false;

  EmbeddingMemory memory(config, nullptr);
  memory.set_embedder(fake_embedder);
  memory.remember_turn("hello robot", "hello human");

  EXPECT_TRUE(memory.retrieve("hello robot").empty());
  EXPECT_FALSE(std::filesystem::exists(this->storage_file_));
}

TEST(EmbeddingMemoryCosine, HandlesDegenerateInputs) {
  EXPECT_DOUBLE_EQ(EmbeddingMemory::cosine_similarity({}, {}), 0.0);
  EXPECT_DOUBLE_EQ(EmbeddingMemory::cosine_similarity({1.0F}, {1.0F, 2.0F}),
                   0.0);
  EXPECT_DOUBLE_EQ(EmbeddingMemory::cosine_similarity({0.0F}, {0.0F}), 0.0);
  EXPECT_NEAR(EmbeddingMemory::cosine_similarity({1.0F, 0.0F}, {1.0F, 0.0F}),
              1.0, 1e-6);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
