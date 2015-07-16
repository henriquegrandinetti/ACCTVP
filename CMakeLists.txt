cmake_minimum_required(VERSION 2.8)

project(planeProjection)

find_package(OpenCV)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( 
	planeProjection 
	main.cpp
	MSAC.cpp
	MSAC.h
	lmmin.h
	lmmin.c
	vanishingPoint.h
	vanishingPoint.cpp
	geometry.h
	geometry.cpp
	TopView.h
	TopView.cpp
)

target_link_libraries(planeProjection ${OpenCV_LIBS})