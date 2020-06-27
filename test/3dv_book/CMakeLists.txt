project(test_3dv_book)

add_executable(test_sobel test_sobel.cc)
target_link_libraries(test_sobel
  ${OpenCV_LIBRARIES}
)
