cmake_minimum_required(VERSION 2.8.3)
get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId} C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g")
set(BINDIR	"${CMAKE_SOURCE_DIR}/../../bin")
set(SRCDIR	"${CMAKE_SOURCE_DIR}/../../src")
set(FS_LIBRARIES "-lstdc++fs")

find_package( PythonInterp 3.6 REQUIRED )
find_package( PythonLibs 3.6 REQUIRED )
find_package( OpenCV REQUIRED )

INCLUDE_DIRECTORIES ( ${SRCDIR} ${PYTHON_INCLUDE_DIRS} )

file(GLOB SOURCES ${SRCDIR}/intersection3camera_cls/*.cpp ${SRCDIR}/utils/*.cpp *.cpp)
 
add_executable( ${PROJECT_NAME} ${SOURCES} )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${PYTHON_LIBRARIES} )

install( TARGETS ${PROJECT_NAME} DESTINATION ${BINDIR} )
