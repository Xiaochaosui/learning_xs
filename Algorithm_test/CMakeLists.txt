cmake_minimum_required(VERSION 3.26)
project(Algorithm_test)

set(CMAKE_CXX_STANDARD 11)
#set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../Algorithm_test-bin )

add_executable(
        Algorithm_test
        main.cpp
        src/Algorithm.cpp
        src/Algorithm.h
#        ${PROJECT_SOURCE_DIR}/src/cyber/CyberMsg.cpp
        src/cyber/CyberMsg.cpp

)

#OpenCV, 指定版本3.1
SET(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV 3.1 ${VERSION_REQ_OCV} REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
#FFTW库链接
SET(FFTW /usr/local/fftw)
set(FFTW_INCLUDE_DIRS "/usr/local/fftw/include")
set(FFTW_LIBRARY_DIRS "/usr/local/fftw/lib")
include_directories( ${FFTW_INCLUDE_DIRS} )

#find_package(FFTW REQUIRED)
#target_link_libraries(Algorithm_test PRIVATE FFTW::FFTW)


#添加include目录：设置头文件的搜索目录
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/xsproto_include
        ${PROJECT_SOURCE_DIR}/src/cyber

)



#声明link目录位置
link_directories(
        #${PCL_LIBRARY_DIRS}
        ${PYTHON_LIBRARY}
        ${OpenCV_LIBRARY_DIRS}
        /usr/local/lib

)

target_link_libraries( Algorithm_test
        ${OpenCV_LIBS}
        /usr/local/fftw/lib/libfftw3.a
        -L/usr/local/lib -lprotobuf -lfastrtps -lfastcdr -lxscom
        -L/usr/lib/x86_64-linux-gnu -lpthread -lboost_thread
        ${PROJECT_SOURCE_DIR}/lib/18.04_x86_64/libxsproto.a
#        ${PROJECT_SOURCE_DIR}/lib/18.04_x86_64/libProtoMsg.a
#        ${PROJECT_SOURCE_DIR}/lib/18.04_x86_64/libNmlMsg.a



)

