cmake_minimum_required(VERSION 3.8)
project(diffbot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Tìm các gói cần thiết
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED) # Thêm dòng này
find_package(rosidl_default_generators REQUIRED)

# Cài đặt các tệp dữ liệu (launch, urdf, config)
# CMAKE sẽ lo việc này
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Thêm lệnh này để gọi tệp setup.py
# Nó sẽ lo việc cài đặt các node Python
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
