project(utils)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/lib)
add_library(generate_data SHARED generate_data.cc)
target_link_libraries(generate_data
  ${OpenCV_LIBRARIES}
)


add_library(plot2d_curve SHARED plot2d_curve.cc)
target_link_libraries(plot2d_curve 
  ${OpenCV_LIBRARIES}
)