cmake_minimum_required(VERSION 3.8)
project(iic_lab1)

# Buscar ament_cmake (solo esto es necesario)
find_package(ament_cmake REQUIRED)

# Instalar scripts Python
install(PROGRAMS
  scripts/dead_reckoning_nav.py
  scripts/obstacle_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar archivos launch y config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()