cmake_minimum_required(VERSION 3.5)
project(dex_controller)

# Apppend CMake module path for custom modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

# Make conan.cmake commands available
if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
  message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
  file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/0.18.1/conan.cmake"
                "${CMAKE_BINARY_DIR}/conan.cmake"
                TLS_VERIFY ON)
endif()

include(${CMAKE_BINARY_DIR}/conan.cmake)

conan_cmake_configure(REQUIRES eigen/3.4.0
                      GENERATORS CMakeToolchain CMakeDeps)

conan_cmake_autodetect(settings)

conan_cmake_install(PATH_OR_REFERENCE .
                    BUILD missing
                    REMOTE conancenter
                    SETTINGS ${settings})


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_critics REQUIRED)
find_package(dwb_plugins REQUIRED)

find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 17)

# Temporary disable unused-parameter error/warning for the dummy controller.
# Please remove this line when done developing =)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-parameter -Wno-unused-parameter -Werror=maybe-uninitialized -Wno-maybe-uninitialized")



include_directories(
  include
)
set(library_name dex_controller)
set(dependencies
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    pluginlib
    nav_msgs
    nav2_core
    nav2_util
    nav2_regulated_pure_pursuit_controller
    tf2_eigen
)

add_library(${library_name} SHARED
    src/dex_controller.cpp
    )

ament_target_dependencies(${library_name}
    rclcpp
    geometry_msgs
    pluginlib
    nav_msgs
    nav2_core
    nav2_util
    nav2_regulated_pure_pursuit_controller
    tf2_eigen
)

target_link_libraries(${library_name}
  Eigen3::Eigen
)

install(TARGETS dex_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core dex_controller.xml)

ament_package()
