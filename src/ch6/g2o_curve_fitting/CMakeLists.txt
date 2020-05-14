project(g2oCurveFitting)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/ch6)
# link_libraries(${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${OpenCV_LIBRARIES})
add_executable(g2o_curve_fitting g2o_curve_fitting.cc)

# 通过这种方式，使 link g2o_lib 的语句与 link 其他库 的语句一致。
set(G2O_LIBRARIES "")
list(APPEND G2O_LIBRARIES ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
target_link_libraries(g2o_curve_fitting
    ${G2O_CORE_LIBRARY} 
    ${G2O_STUFF_LIBRARY}
    ${OpenCV_LIBRARIES}
    # ${G2O_LIBRARIES}
)