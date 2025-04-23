cmake_minimum_required(VERSION 3.8)
project(lab1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)





# Instalación de scripts de Python
install(
	PROGRAMS
	nodes/nodo_traductor.py
	nodes/nodo_dead_reckoning_nav.py
	nodes/nodo_pose_loader.py
  	#nodes/otro_nodo.py
	DESTINATION lib/${PROJECT_NAME}
)



# Hace accesible la carpeta launch
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )



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
