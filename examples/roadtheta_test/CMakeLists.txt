cmake_minimum_required(VERSION 2.8)
get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId} C CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g")
set(BINDIR	"${CMAKE_SOURCE_DIR}/../../bin")
set(SRCDIR	"${CMAKE_SOURCE_DIR}/../../src")

find_package( OpenCV REQUIRED )

INCLUDE_DIRECTORIES ( ${SRCDIR} )

file(GLOB SOURCES ${SRCDIR}/core/*.cpp ${SRCDIR}/roadtheta/*.cpp ${SRCDIR}/utils/*.cpp *.cpp)
 
add_executable( ${PROJECT_NAME} ${SOURCES} )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )

install( TARGETS ${PROJECT_NAME} DESTINATION ${BINDIR} )
