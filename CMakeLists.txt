cmake_minimum_required(VERSION 3.5)
project(my_bot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)  # Dependencia de rclpy para los nodos de Python
find_package(geometry_msgs REQUIRED)  # Dependencia de geometry_msgs para el manejo de poses
find_package(OpenCV REQUIRED)  # Añadir OpenCV como dependencia
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Añadir la ruta de la carpeta gazetracking al PYTHONPATH
set(GAZETRACKING_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/GazeTracking")
set(MODELS_PATH "${CMAKE_CURRENT_SOURCE_DIR}/models")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY ${GAZETRACKING_PATH}/
  DESTINATION lib/${PROJECT_NAME}/gazetracking
)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/models
  DESTINATION lib/${PROJECT_NAME}/models
)

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Añadir el ejecutable de Python para el filtro de LIDAR
install(
  PROGRAMS src/filtro_lidar.py  # Cambia al nombre correcto de tu archivo si es necesario
  DESTINATION lib/${PROJECT_NAME}
)

# Añadir el ejecutable de Python para la interfaz de selección de goal pose
install(
  PROGRAMS src/goal_pose_interface.py  # Cambia al nombre correcto de tu archivo si es necesario
  DESTINATION lib/${PROJECT_NAME}
)

# Añadir el ejecutable de Python para el eyetracking
install(
  PROGRAMS src/eyetracking.py  # Añadir el archivo de tu script de eyetracking
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS src/deteccion.py  # Añadir el archivo de tu script de eyetracking
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
