cmake_minimum_required(VERSION 3.8)
project(action_cleaning_robot)

# Проверка версии компилятора
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Поиск необходимых пакетов
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)

# Генерация интерфейсов из action файла
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CleaningTask.action"
  DEPENDENCIES action_msgs
)

# Установка Python пакета
# ament_python_install_package(${PROJECT_NAME})

# Установка action файлов
install(
  DIRECTORY action
  DESTINATION share/${PROJECT_NAME}
)

# Установка запускаемых файлов
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/cleaning_action_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cleaning_action_server
)

install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/cleaning_action_client.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cleaning_action_client
)

# Объявление зависимостей для ament
ament_export_dependencies(
  rclpy 
  rosidl_default_runtime
)

# Объявление зависимостей для сборки
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Тестирование (опционально)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()