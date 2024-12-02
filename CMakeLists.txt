cmake_minimum_required(VERSION 3.5)
project(eureka_wtimu_2)

# Найти необходимые пакеты
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Установка Python-скриптов
install(PROGRAMS
src/wt901_node.py
src/wt901_calibration.py
DESTINATION lib/${PROJECT_NAME}
)

ament_package()