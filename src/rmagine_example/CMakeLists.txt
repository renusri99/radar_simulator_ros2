cmake_minimum_required(VERSION 3.8)
project(rmagine_example)

add_compile_options(-std=c++20)
set(CMAKE_CXX_STANDARD 20)

# Find the rmagine package
find_package(rmagine 2.2.5
  COMPONENTS
    core 
    embree
)

# Add your executable
add_executable(vlp16_example src/vlp16_example.cpp)

# Link against rmagine targets
target_link_libraries(vlp16_example
    rmagine::core
    rmagine::embree
)

# Install the executable
install(TARGETS
  vlp16_example
  DESTINATION lib/${PROJECT_NAME})


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
