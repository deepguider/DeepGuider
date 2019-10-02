cmake_minimum_required(VERSION 2.6)

set(CMAKE_CXX_STANDARD 11)
set(BINDIR	"${CMAKE_SOURCE_DIR}/../../bin")
set(AUTHOR_NAME	"jaeyeong.lee")

get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId} C CXX)
find_package( OpenCV REQUIRED )

INCLUDE_DIRECTORIES ( ${CMAKE_SOURCE_DIR}/../../src )

file(GLOB SOURCES ${CMAKE_SOURCE_DIR}/../../src/*.cpp ${CMAKE_SOURCE_DIR}/../../src/core/*.cpp *.cpp)
 
add_executable(${PROJECT_NAME} ${SOURCES}  )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )

install(TARGETS ${PROJECT_NAME} DESTINATION ${BINDIR})
