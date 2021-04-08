cmake_minimum_required(VERSION 2.8.3)
project(dg_simple_ros)

# set Python path
set(PYTHON_INCLUDE_DIRS "/usr/include/python3.6m")
set(PYTHON_LIBRARIES "/usr/lib/x86_64-linux-gnu/libpython3.6m.so")

# set DeepGuider path
set(DGDIR   "/home/jsh/workspace/deepguider/DeepGuider")
set(BINDIR  "${DGDIR}/bin")
set(SRCDIR  "${DGDIR}/src")
set(EXTDIR  "${DGDIR}/EXTERNAL")
set(RAPIDJSON_INC "${EXTDIR}/rapidjson/include")
set(CURL_LIBRARIES "-lcurl")
set(JSONCPP_LIBRARIES "-ljsoncpp")
set(FS_LIBRARIES "-lstdc++fs")

add_compile_options(-std=c++14)
add_compile_options(-pthread)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
  message_generation
)

#find_package( PythonInterp 3.6 REQUIRED )
#find_package( PythonLibs 3.6 REQUIRED )
find_package( OpenCV 3.2 REQUIRED )
find_package( CURL REQUIRED )

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ocr.msg
  ocr_info.msg
  action.msg
  guidance.msg
  path_element.msg
  path_info.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dg_simple_ros
   CATKIN_DEPENDS cv_bridge roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories( ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${SRCDIR} ${RAPIDJSON_INC} ${PYTHON_INCLUDE_DIRS} ${DGDIR}/examples/dg_simple)

file(GLOB SOURCES ${SRCDIR}/core/*.cpp ${SRCDIR}/exploration/*.cpp ${SRCDIR}/guidance/*.cpp ${SRCDIR}/localizer/*.cpp ${SRCDIR}/intersection_cls/*.cpp ${SRCDIR}/logo_recog/*.cpp ${SRCDIR}/map_manager/*.cpp ${SRCDIR}/ocr_recog/*.cpp ${SRCDIR}/poi_recog/*.cpp ${SRCDIR}/road_recog/*.cpp ${SRCDIR}/vps/*.cpp ${SRCDIR}/utils/*.cpp ${EXTDIR}/qgroundcontrol/*.cpp *.cpp)

# dg_simple_ros
add_executable(${PROJECT_NAME} src/dg_simple_ros.cpp ${SOURCES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PYTHON_LIBRARIES}
  ${CURL_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  ${FS_LIBRARIES}
)

# dg_ocr
add_executable(dg_ocr src/dg_ocr.cpp ${SOURCES})
add_dependencies(dg_ocr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  dg_ocr
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PYTHON_LIBRARIES}
  ${CURL_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  ${FS_LIBRARIES}
)


install(TARGETS ${PROJECT_NAME} dg_ocr
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
