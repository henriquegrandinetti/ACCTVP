cmake_minimum_required(VERSION 2.8)

project(ACCTVP)

find_package(OpenCV)

include_directories( ${OpenCV_INCLUDE_DIRS} )

file(GLOB ACCTBP_SCR
    "src/*.h"
    "src/*.cpp"
    "src/*.c"
)

add_executable(ACCTVP ${ACCTBP_SCR})

target_link_libraries(ACCTVP ${OpenCV_LIBS})