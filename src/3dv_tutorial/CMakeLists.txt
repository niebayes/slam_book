project(src)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)

set(G2O_LIBRARIES "")
list(APPEND G2O_LIBRARIES ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})

message(${CMAKE_HOME_DIRECTORY})

# Notes about link_directories: 
# Specify directories in which the linker will look for libraries.
# link_directories(directory1 directory2 ...)
# Specify the paths in which the linker should search for libraries. The command will apply only to targets created after it is called. Relative paths given to this command are interpreted as relative to the current source directory, see CMP0015.
# Note that this command is rarely necessary. Library locations returned by find_package() and find_library() are absolute paths. Pass these absolute library file paths directly to the target_link_libraries() command. CMake will ensure the linker finds them.
# link_directories(${CMAKE_HOME_DIRECTORY}/lib)
# add_executable(g2o_curve_fitting g2o_curve_fitting.cc)
# target_link_libraries(g2o_curve_fitting 
#   ${G2O_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   libplot2d_curve.dylib
#   -lgenerate_data
# )

# add_executable(image_formation image_formation.cc)
# target_link_libraries(image_formation 
#   ${OpenCV_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${CMAKE_HOME_DIRECTORY}/lib/libthree_d_vision.dylib
#   ${CMAKE_HOME_DIRECTORY}/lib/libpcd_io_utils.dylib
# )

# add_executable(object_localization object_localization.cc)
# target_link_libraries(object_localization 
#   ${OpenCV_LIBRARIES}
#   ${CMAKE_HOME_DIRECTORY}/lib/libvision_utils.dylib
# )

# add_executable(distortion_correction distortion_correction.cc)
# target_link_libraries(distortion_correction
#   ${OpenCV_LIBRARIES}
# )

# add_executable(camera_calibration camera_calibration.cc)
# target_link_libraries(camera_calibration
#   ${OpenCV_LIBRARIES}
# )

# add_executable(pose_estimation_chessboard pose_estimation_chessboard.cc)
# target_link_libraries(pose_estimation_chessboard
#     ${OpenCV_LIBRARIES}
# )

# add_executable(pose_estimation_book1 pose_estimation_book1.cc)
# target_link_libraries(pose_estimation_book1
#     ${OpenCV_LIBRARIES}
# )

# add_executable(pose_estimation_book2 pose_estimation_book2.cc)
# target_link_libraries(pose_estimation_book2
#     ${OpenCV_LIBRARIES}
# )

# add_executable(pose_estimation_book3 pose_estimation_book3.cc)
# target_link_libraries(pose_estimation_book3
#     ${OpenCV_LIBRARIES}
# )

# add_executable(image_stitching image_stitching.cc)
# target_link_libraries(image_stitching
#     ${OpenCV_LIBRARIES}
# )

# add_executable(video_stabilization video_stabilization.cpp)
# target_link_libraries(video_stabilization
#     ${OpenCV_LIBRARIES}
# )

add_executable(visual_odometry_epipolar visual_odometry_epipolar.cc)
target_link_libraries(visual_odometry_epipolar
  ${OpenCV_LIBRARIES}
)