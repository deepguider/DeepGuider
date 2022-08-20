cmake_minimum_required(VERSION 2.8.3)
get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId} C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g")
set(BINDIR	"${CMAKE_SOURCE_DIR}/../../bin")
set(SRCDIR	"${CMAKE_SOURCE_DIR}/../../src")
set(EXTDIR	"${CMAKE_SOURCE_DIR}/../../EXTERNAL")
set(RAPIDJSON_INC "${EXTDIR}/rapidjson/include")
set(CURL_INC "${EXTDIR}/curl/include")
set(CURL_LIBRARIES "-lcurl")
set(JSONCPP_LIBRARIES "-ljsoncpp")
set(FS_LIBRARIES "-lstdc++fs")

find_package( PythonInterp 3.6 REQUIRED )
find_package( PythonLibs 3.6 REQUIRED )
find_package( OpenCV REQUIRED )
find_package( CURL REQUIRED )

INCLUDE_DIRECTORIES ( ${SRCDIR} ${RAPIDJSON_INC} ${CURL_INC} ${PYTHON_INCLUDE_DIRS} )

file(GLOB SOURCES ${SRCDIR}/core/*.cpp ${SRCDIR}/exploration/*.cpp ${SRCDIR}/guidance/*.cpp ${SRCDIR}/localizer/*.cpp ${SRCDIR}/intersection_cls/*.cpp ${SRCDIR}/intersection3camera_cls/*.cpp ${SRCDIR}/logo_recog/*.cpp ${SRCDIR}/map_manager/*.cpp ${SRCDIR}/ocr_recog/*.cpp ${SRCDIR}/poi_recog/*.cpp ${SRCDIR}/roadtheta/*.cpp ${SRCDIR}/vps/*.cpp ${SRCDIR}/utils/*.cpp ${EXTDIR}/qgroundcontrol/*.cpp *.cpp)

add_executable( ${PROJECT_NAME} ${SOURCES} )
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} ${PYTHON_LIBRARIES} ${CURL_LIBRARIES} ${JSONCPP_LIBRARIES} ${FS_LIBRARIES} )

install( TARGETS ${PROJECT_NAME} DESTINATION ${BINDIR} )
