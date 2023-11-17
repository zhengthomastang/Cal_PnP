cmake_minimum_required(VERSION 3.18)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

project(cal_pnp)

find_package(OpenCV REQUIRED)

unset(CMAKE_CXX_FLAGS CACHE)

set(CMAKE_CXX_STANDARD 17)

include_directories(${OpenCV_INCLUDE_DIRS})

set(LIBS ${OpenCV_LIBS})

add_executable(${PROJECT_NAME} src/CamCal.cpp src/CamCal.h src/Cfg.cpp src/Cfg.h src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${LIBS})
