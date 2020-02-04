cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(BINDIR	"${CMAKE_SOURCE_DIR}/../../bin")
set(SRCDIR	"${CMAKE_SOURCE_DIR}/../../src")

get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId} C CXX)

find_package( PythonInterp 3.6 REQUIRED )
find_package( PythonLibs 3.6 REQUIRED )
find_package( OpenCV 4.0 REQUIRED )

INCLUDE_DIRECTORIES ( ${SRCDIR} ${PYTHON_INCLUDE_DIRS} )

file(GLOB SOURCES ${SRCDIR}/core/*.cpp ${SRCDIR}/map_manager/*.cpp *.cpp)
 
add_executable( ${PROJECT_NAME} ${SOURCES} )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${PYTHON_LIBRARIES} )

install( TARGETS ${PROJECT_NAME} DESTINATION ${BINDIR} )