cmake_minimum_required(VERSION 2.8.3)
project(dg_test)

add_compile_options(-std=c++11)
add_compile_options(-pthread)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
)

#find_package( PythonInterp 3.6 REQUIRED )
#find_package( PythonLibs 3.6 REQUIRED )
#find_package( OpenCV 4.2 REQUIRED )
find_package( OpenCV 3 REQUIRED )
find_package( CURL REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dg_test
   CATKIN_DEPENDS cv_bridge roscpp std_msgs
#  DEPENDS system_lib
)

## Your package locations should be listed before other locations
set(DGDIR   "/work/deepguider")
set(BINDIR	"${DGDIR}/bin")
set(SRCDIR	"${DGDIR}/src")
set(EXTDIR	"${DGDIR}/EXTERNAL")
set(RAPIDJSON_INC "${EXTDIR}/rapidjson/include")
set(PYTHON_INCLUDE_DIRS "/usr/include/python3.6m")
set(PYTHON_LIBRARIES "/usr/lib/x86_64-linux-gnu/libpython3.6m.so")

include_directories( ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${SRCDIR} ${RAPIDJSON_INC} ${PYTHON_INCLUDE_DIRS} )

file(GLOB SOURCES ${SRCDIR}/core/*.cpp ${SRCDIR}/exploration/*.cpp ${SRCDIR}/guidance/*.cpp ${SRCDIR}/localizer/*.cpp ${SRCDIR}/map_manager/*.cpp ${SRCDIR}/poi_recog/*.cpp ${SRCDIR}/road_recog/*.cpp ${SRCDIR}/vps/*.cpp ${EXTDIR}/qgroundcontrol/*.cpp *.cpp)

add_executable(${PROJECT_NAME}_node src/dg_test.cpp ${SOURCES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  ${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PYTHON_LIBRARIES}
  ${CURL_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dg_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
