# Set the minimum required CMake version
cmake_minimum_required(VERSION 3.5)

# Set the project name
set(PROJECT_NAME as2_platform_multirotor_simulator)
project(${PROJECT_NAME})

# Default to C++17 if not set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set Release as default build type if not set
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rosidl_default_generators
  rclcpp
  as2_core
  as2_msgs
  std_srvs
  std_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ResetSimulatorState.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Add project submodules
if(${multirotor_simulator_FOUND})
  message(STATUS "multirotor_simulator found")
else()
  message(STATUS "multirotor_simulator not found")
  include(FetchContent)
  fetchcontent_declare(
    multirotor_simulator
    GIT_REPOSITORY https://github.com/RPS98/multirotor_simulator.git
    GIT_TAG v2.0
  )
  fetchcontent_makeavailable(multirotor_simulator)
endif()

# Include necessary directories
include_directories(
  include
  include/${PROJECT_NAME}

  {EIGEN3_INCLUDE_DIRS}

  # Incluye multirotor_simulator
  ${CMAKE_BINARY_DIR}/_deps/multirotor_simulator-src/include
)

# Set source files
set(SOURCE_CPP_FILES
  src/${PROJECT_NAME}.cpp
  src/as2_interface.cpp
)
set(PROJECT_LIBRARY_TARGET ${PROJECT_NAME}_lib)

# Create the node executable
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp ${SOURCE_CPP_FILES})
target_include_directories(${PROJECT_NAME}_node PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}__cpp)
target_link_libraries(${PROJECT_NAME}_node multirotor_simulator ${cpp_typesupport_target})

# Create the library
add_library(${PROJECT_LIBRARY_TARGET} STATIC ${SOURCE_CPP_FILES})
set_target_properties(${PROJECT_LIBRARY_TARGET} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_include_directories(${PROJECT_LIBRARY_TARGET} PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
ament_target_dependencies(${PROJECT_LIBRARY_TARGET} ${PROJECT_DEPENDENCIES})
add_dependencies(${PROJECT_LIBRARY_TARGET} ${PROJECT_NAME}__cpp)
target_link_libraries(${PROJECT_LIBRARY_TARGET} multirotor_simulator ${cpp_typesupport_target})

# Install the headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the node executable
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the library
install(
  TARGETS ${PROJECT_LIBRARY_TARGET}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Build tests if testing is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(tests)
endif()

# Create the ament package
ament_export_dependencies(rosidl_default_runtime)
ament_package()
