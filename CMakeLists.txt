cmake_minimum_required(VERSION 3.8)
project(abv_comms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(abv_idl REQUIRED)

set(SOURCES
  src/RosTopicManager.cpp
  src/ABV_Comms.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    abv_idl
)

# Include directories for the library
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Export the shared library as a package target
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp abv_idl)

#### Build the examples #### 
## Thruster Control example
add_executable(thruster_control_ex ${CMAKE_CURRENT_LIST_DIR}/examples/thruster_control/main.cpp)
target_link_libraries(thruster_control_ex abv_comms)
ament_target_dependencies(thruster_control_ex rclcpp abv_idl)

# Install the executables
install(TARGETS
  thruster_control_ex
  DESTINATION lib/${PROJECT_NAME}
)

# TODO:
## Pose Control example ###

# Link against the abv_idl interface
#target_link_libraries(${PROJECT_NAME} PUBLIC rclcpp abv_idl)

install(DIRECTORY include/
    DESTINATION include/
)

# Install the shared library and headers
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)


ament_package()
