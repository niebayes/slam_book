project(test)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/lib)

# add_executable(kahan_test kahan_test.cc)
add_executable(const_iter_test const_iter_test.cc)