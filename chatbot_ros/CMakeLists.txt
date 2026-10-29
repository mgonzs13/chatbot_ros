cmake_minimum_required(VERSION 3.10)
project(chatbot_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_viewer REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(llama_msgs REQUIRED)
find_package(whisper_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(chatbot_ros_lib
  src/chat_bot.cpp
  src/chatbot_config.cpp
  src/conversation_history.cpp
  src/echo_guard.cpp
  src/embedding_memory.cpp
  src/text/speech_sanitizer.cpp
  src/text/text_similarity.cpp
  src/text/transcript_filter.cpp
  src/states/check_stt_state.cpp
  src/states/generate_chat_completions_state.cpp
  src/states/interrupt_listening_state.cpp
  src/states/listen_state.cpp
  src/states/listening_fsm.cpp
  src/states/sentence_accumulator.cpp
  src/states/speak_response_state.cpp
  src/states/speak_state.cpp
  src/states/speech_pipeline.cpp)
target_include_directories(chatbot_ros_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(chatbot_ros_lib PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  yasmin::yasmin
  yasmin_ros::yasmin_ros
  yasmin_viewer::yasmin_viewer
  nlohmann_json::nlohmann_json
  ${audio_common_msgs_TARGETS}
  ${llama_msgs_TARGETS}
  ${whisper_msgs_TARGETS})

add_executable(chat_bot_node src/chat_bot_node.cpp)
target_link_libraries(chat_bot_node chatbot_ros_lib)

install(TARGETS chatbot_ros_lib
  EXPORT export_chatbot_ros_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS chat_bot_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_speech_pipeline test/test_speech_pipeline.cpp)
  target_link_libraries(test_speech_pipeline chatbot_ros_lib)

  ament_add_gtest(test_sentence_accumulator test/test_sentence_accumulator.cpp)
  target_link_libraries(test_sentence_accumulator chatbot_ros_lib)

  ament_add_gtest(test_check_stt_state test/test_check_stt_state.cpp)
  target_link_libraries(test_check_stt_state chatbot_ros_lib)

  ament_add_gtest(test_text_utils test/test_text_utils.cpp)
  target_link_libraries(test_text_utils chatbot_ros_lib)

  ament_add_gtest(test_chatbot_config test/test_chatbot_config.cpp)
  target_link_libraries(test_chatbot_config chatbot_ros_lib)

  ament_add_gtest(test_embedding_memory test/test_embedding_memory.cpp)
  target_link_libraries(test_embedding_memory chatbot_ros_lib)

  ament_add_gtest(test_chatbot_fsm test/test_chatbot_fsm.cpp
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_chatbot_fsm chatbot_ros_lib)
  set_tests_properties(test_chatbot_fsm PROPERTIES TIMEOUT 120)
endif()

ament_export_targets(export_chatbot_ros_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action yasmin yasmin_ros yasmin_viewer
  audio_common_msgs llama_msgs whisper_msgs nlohmann_json)

ament_package()
