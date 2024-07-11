project(CodeCraftSDK)
cmake_minimum_required (VERSION 3.8)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

if (!WIN32)
    link_libraries(pthread rt m)
endif (!WIN32)







set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build) 
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found.")
endif()
message("OPENCV VERSION:" ${OpenCV_VERSION} )



AUX_SOURCE_DIRECTORY(. src)
ADD_EXECUTABLE(main ${src})



target_link_libraries(main 
    ${OpenCV_LIBS})