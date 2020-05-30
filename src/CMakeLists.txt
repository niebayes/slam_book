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
link_directories(${CMAKE_HOME_DIRECTORY}/lib)
add_executable(g2o_curve_fitting g2o_curve_fitting.cc)
target_link_libraries(g2o_curve_fitting 
  ${G2O_LIBRARIES}
  ${OpenCV_LIBRARIES}
  libplot2d_curve.dylib
  -lgenerate_data
)

