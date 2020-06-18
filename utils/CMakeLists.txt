project(utils)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/lib)
# add_library(generate_data SHARED generate_data.cc)
# target_link_libraries(generate_data
#   ${OpenCV_LIBRARIES}
# )


# add_library(plot2d_curve SHARED plot2d_curve.cc)
# target_link_libraries(plot2d_curve 
#   ${OpenCV_LIBRARIES}
# )

# add_library(three_d_vision SHARED three_d_vision.cc)
# target_link_libraries(three_d_vision 
#   ${OpenCV_LIBRARIES}
# )

add_executable(pcd_io_utils pcd_io_utils.cc)
target_link_libraries(pcd_io_utils
  ${PCL_LIBRARIES}
)