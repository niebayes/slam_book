project(TrajectoryError)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/ch4)
add_executable(trajectory_error trajectory_error.cc)
target_link_libraries(trajectory_error
    ${Pangolin_LIBRARIES}
)

