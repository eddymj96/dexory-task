ament_add_gtest(simple_pid_tests simple_pid_tests.cpp)
ament_target_dependencies(simple_pid_tests ${dependencies})
target_link_libraries(simple_pid_tests ${library_name})

ament_add_gtest(dex_controller_tests dex_controller_tests.cpp)
ament_target_dependencies(dex_controller_tests ${dependencies})
target_link_libraries(dex_controller_tests ${library_name})
