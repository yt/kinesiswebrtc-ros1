cmake_minimum_required(VERSION 2.8.3)
project(h264_encoder_core)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)

# find_package(FFmpeg REQUIRED)
find_package(aws_common REQUIRED)


#############
## Compile ##
#############

# add the publisher example
add_library(${PROJECT_NAME} STATIC
  src/h264_encoder.cpp
  src/h264_encoder_node_config.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE include
  PRIVATE ${aws_common_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  avcodec
  avutil
  swscale
  dl
)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
EXPORT "${PROJECT_NAME}-targets"
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

## Mark cpp header files for installation
install(
DIRECTORY include/
DESTINATION include
)

export(TARGETS ${PROJECT_NAME}
FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake")
export(PACKAGE ${PROJECT_NAME})

CONFIGURE_FILE(commonConfig.cmake.in
${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake @ONLY)
CONFIGURE_FILE(commonConfigVersion.cmake.in
${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake @ONLY)

INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(EXPORT ${PROJECT_NAME}-targets DESTINATION share/${PROJECT_NAME}/cmake)


#############
## Tests ##
#############

enable_testing()
find_package(GTest QUIET)
if(NOT GTEST_FOUND)
message(WARNING "Could not find GTest. Not building unit tests.")
else()
add_executable(test_h264_encoder_core test/h264_encoder_test.cpp)
target_include_directories(test_h264_encoder_core
  PRIVATE include
  PRIVATE ${aws_common_INCLUDE_DIRS})
target_link_libraries(test_h264_encoder_core
  ${GTEST_LIBRARIES}
  pthread
  ${aws_common_LIBRARIES}
  ${PROJECT_NAME}
)
add_test(NAME test_h264_encoder_core COMMAND test_h264_encoder_core --gtest_output=xml:test_results/)
endif()