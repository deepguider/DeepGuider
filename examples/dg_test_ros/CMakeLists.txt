cmake_minimum_required(VERSION 2.8.3)
project(dg_ros)

# set Python path
set(PYTHON_INCLUDE_DIRS "/usr/include/python3.6m")
set(PYTHON_LIBRARIES "/usr/lib/x86_64-linux-gnu/libpython3.6m.so")

# set DeepGuider path
set(DGDIR   "/work/deepguider")
set(BINDIR	"${DGDIR}/bin")
set(SRCDIR	"${DGDIR}/src")
set(EXTDIR	"${DGDIR}/EXTERNAL")
set(RAPIDJSON_INC "${EXTDIR}/rapidjson/include")
set(CURL_LIBRARIES "-lcurl")
set(JSONCPP_LIBRARIES "-ljsoncpp")

add_compile_options(-std=c++14)
add_compile_options(-pthread)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
)

#find_package( PythonInterp 3.6 REQUIRED )
#find_package( PythonLibs 3.6 REQUIRED )
find_package( OpenCV REQUIRED )
find_package( CURL REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dg_ros
   CATKIN_DEPENDS cv_bridge roscpp std_msgs
#  DEPENDS system_lib
)

include_directories( ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${SRCDIR} ${RAPIDJSON_INC} ${PYTHON_INCLUDE_DIRS} ${DGDIR}/examples/dg_test)

file(GLOB SOURCES ${SRCDIR}/core/*.cpp ${SRCDIR}/exploration/*.cpp ${SRCDIR}/guidance/*.cpp ${SRCDIR}/localizer/*.cpp ${SRCDIR}/map_manager/*.cpp ${SRCDIR}/poi_recog/*.cpp ${SRCDIR}/road_recog/*.cpp ${SRCDIR}/vps/*.cpp ${SRCDIR}/utils/*.cpp ${EXTDIR}/qgroundcontrol/*.cpp *.cpp)

add_executable(${PROJECT_NAME}_node src/dg_ros.cpp ${SOURCES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PYTHON_LIBRARIES}
  ${CURL_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)

