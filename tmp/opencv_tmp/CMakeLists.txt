project(opencv_tmp)

add_executable(sobel sobel.cc)
target_link_libraries(sobel
  ${OpenCV_LIBRARIES}
)