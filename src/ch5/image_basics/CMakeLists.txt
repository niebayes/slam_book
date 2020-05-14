project(ImageBasics)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/ch5)
add_executable(image_basics image_basics.cc)
target_link_libraries(image_basics ${OpenCV_LIBRARIES})