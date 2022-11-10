cmake_minimum_required(VERSION 3.0.2)
project(ar_ik)
add_compile_options(-std=c++17)
include_directories(/usr/local/include/eigen3)
# find_package(PythonLibs REQUIRED)
# include_directories(${PYTHON_INCLUDE_DIRS})

add_executable(ar_ik ar_ik_task.cpp)
add_executable(user user.cpp ar_ik_api.cpp)

# target_link_libraries(user ${PYTHON_LIBRARIES})