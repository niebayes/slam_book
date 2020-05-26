project(src)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)

set(G2O_LIBRARIES "")
list(APPEND G2O_LIBRARIES ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})

link_directories(${CMAKE_HOME_DIRECTORY}/lib)
add_executable(g2o_curve_fitting g2o_curve_fitting.cc)
target_link_libraries(g2o_curve_fitting 
  ${G2O_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

