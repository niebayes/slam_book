# 添加额外的头文件的查找目录（除编译器默认的查找目录之外），命令中含 directories：
# 以下两条命令相当于 -I 参数。
# 如果你只想使用外部库，且外部库已经编译好了，提供了相应的库文件。那么就不需要用这个命令，而只需要用 link 相关的命令将当前项目构建的 target 与外部库的库文件进行链接就好了。
# include_directories() 添加额外的查找头文件的目录，不会recursive search。该命令用得较多。通过AFTER 或者 BEFORE 参数（放在第一个位置），可以控制这些头文件的查找顺序是在默认查找目录的之后还是之前。

# target_include_directories() 添加额外的查找头文件的目录。需要指定 target，因此应用在某个 target 之后，即add_exetutable() 或 ada_libraries() 命令之后。一般不用这个命令。


# 添加额外的库文件的查找目录，命令中含 directories：
# 以下两条命令相当于 -L 参数。
# link_directories() 添加链接时，库文件的查找目录，不会recursive search。一般不用这个命令。

# target_link_directories() 设置要链接的库的查找目录。与 link_directories() 的区别在于，该命令需指定 target，因此应写在某个 target 之后，即add_exetutable() 或 ada_libraries() 命令之后。一般不用这个命令。


# 添加要链接的库文件，命令中含 libraries（注意目录和文件名称的区别）：
# 以下两条命令相当于 -l 参数。
# link_libraries() 设置要链接的库的名称，不加后缀的话，默认优先链接同名的共享库。与 target_link_libraries() 的区别在于，link_libraries()命令需写在 target 前。target通常指由源文件生成的可执行文件或库文件，即 add_executable() 或 add_libraries() 命令设置的 target。 一个CMakeLists.txt文件中可出现多个 add_executable(), add_libraries()命令，而 link_libraries() 会将传入该命令的库，链接到该文件后续的所有targets 中。因此不推荐使用！ 

# target_link_libraries() 设置要链接的库的名称，不加后缀的话，默认优先链接同名的共享库。该命令需指定 target，因此应写在某个 target 之后，即add_exetutable() 或 ada_libraries() 命令之后。该命令用得较多。


# 设置需要生成的库文件和可执行文件：
# 本质上库文件与可执行文件没有不同。唯一的（？？）的区别在于：一般来说（？？），库文件不能独立运行，只能被其他可执行文件“调用”；而可执行文件能够独立运行。这是因为可执行文件的源码中有 main() 函数存在，该函数充当入口函数，能被操作系统调用。
# 设置需要生成的库文件的名称，库文件的类型（静态库/共享库/模块（一般不用）），生成该库文件的源文件。如果在该命令中指定多个源文件，那么它们生成的目标文件将被归档为一个库文件。一个 CMakeLists.txt 文件中可以有多个 add_libraries() 命令。
# add_library()


# 设置需要生成的可执行文件的名称，生成该可执行文件的源文件名称。在某些情况下（如只需编译得到库文件，然后供其他项目使用），则该命令可以省略。一个 CMakeLists.txt 文件中可以有多个 add_exetutable() 命令，例如在编译测试用的可执行文件时。 
# Adds an executable target called <name> to be built from the source files listed in the command invocation. (The source files can be omitted here if they are added later using target_sources().) 在指定多个源文件时（其中一个文件中必须包含 main() 函数），这种情况相当于将这些源文件组合生成一个可执行文件，则没有相应的可复用的库生成。
# add_executable()


# 载入外部项目并加载该项目的设置（比如某些变量名称）
# 可选参数：REQUIRED 则找不到就报错，不继续编译； QUIET 找不到也不报错，继续编译。该命令一般结合以上命令使用，通过配置多个变量，从而加强项的可移植性。
# find_package() 的两个模式 Module 和 Config，参考：https://zhuanlan.zhihu.com/p/97369704 。Module模式可以这样引入外部库： 
# 该命令查找的对象为:
# Module mode: In Module mode, CMake searches for a file called Find<PackageName>.cmake. The file is first searched in the CMAKE_MODULE_PATH, then among the Find Modules(名称为 Find<PackageName>.cmake 的files) provided by the CMake installation.
# Config mode: The command searches for a file called <PackageName>Config.cmake or <lower-case-package-name>-config.cmake for each name specified。
# LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules) 
# Config模式的引入参考： https://kb.kutu66.com/cmake/post_723600 
# User should use Module mode, and project maintainers wishing to provide a package to be found by this command shoud read on the Config mode signature.
# find_package() 


# 设置编译生成的 target 的目录和
# CMake通过 add_library() 和 add_exetutable() 命令生成 targets。
# CMake将 targets 分为三类：Runtime output artifacts, Library output artifacts, 
# 和 archive output artifacts。

# Runtime包括 add_exetutable() 生成的可执行文件(.out，.exe 等) 和 add_library() 生成的共享库(.dll，仅在DLL平台上(即Windows)有效，即在UNIX平台上，Runtime仅包括可执行文件)
# 设置Runtime的生成目录和名称
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY <path>) # 设置目录 
# set(RUNTIME_OUTPUT_NAME <name>) # 设置名称。或参考这个命令：https://stackoverflow.com/a/57064041

# Library包括 add_library() 生成的模块(.so, .dll，通过设置 MODULE option)，以及 add_library() 生成的共享库(.so, .dylib，通过设置 SHARED option)
# 设置Library的生成目录和名称
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY <path>) # 设置目录
# set(LIBRARY_OUTPUT_NAME <name>) # 设置名称 

# Archive包括 add_library() 生成的静态库(.a, .lib，通过设置 STATIC option 或默认)及其他DLL平台生成的一些文件。
# 设置Archive的生成目录和名称
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY <path>) # 设置目录
# set(ARCHIVE_OUTPUT_NAME <name>) # 设置名称


# 其他一些命令：
# 将 path 中找到的所有源文件的名称 保存到 name 中。以便其他命令用简短的形式调用。
# CMake官方并不推荐用，原因是：如果在 src 目录中新添加其他源文件的话，CMake并不能提示我在 CMakeLists.txt 中进行相应的添加和修改，以致产生某些错误。
# 该命令适用于将多个源文件一起打包生成一个可执行文件或一个库文件。
# aux_source_directory(<path> <name>)

# 添加外部模块（类似与库的可复用的文件）
# LIST() 命令在 列表 中添加元素
# LIST(APPEND CMAKE_MODULE_PATH <module_path>)

# 添加子目录到当前项目中，一起编译（Build）
# 其作用简单地可理解为：将子目录的 CMakeLists.txt 文件的内容在 这个命令 处展开，类似 #include（虽然实际并不是）。
# 这个命令适用于：你有一个大项目，而你想同时发布多个项目，并且这些项目可能都用到了这个大项目的某些文件。因此，为了能一次将这些项目全部编译，可以在每个子项目的 源代码目录 下构建一个 CMakeLists.txt 文件，指定子项目的具体编译过程，如需要生成的可执行文件、库文件和相应的源文件，需要链接的库文件等。大项目的 CMakeLists.txt 文件则配置通用的编译选项，如 设置编译器选项，添加头文件、库文件搜寻路径，查找第三方库等。
# 子目录的 CMakeLists.txt 一般来说应该有自己 project() 命令，这样CMake才会把它们当作一个完整的CMake项目。当然也可以没有。
# 编译时，注意当前打开的 CMakeLists.txt 是哪个。只有打开的是 大项目 的CMakeLists.txt，才能正常编译。
# add_subdirectory(<source_dir> <binary_dir>) # 通常来说，只需使用<source_dir>来指定子项目的 CMakeLists.txt 的所在目录（通常和子项目的源文件放在同一个目录。但如果不是，只要在子项目的 CMakeLists.txt 中配置好相应的路径即可。）

# 设置编译生成的 target 的一些属性，如：OUTPUT_NAME, VERSION 等。该命令用得不多。
# 该命令可同时对多个 targets 设置属性。
# set_target_properties()

################################################################################

# 外部库：
# Eigen3 got 
# Sophus got

cmake_minimum_required(VERSION 3.0)
project(HelloSLAM) 

# 设定编译类型：Debug, Release, etc.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
message("Build type: " ${CMAKE_BUILD_TYPE})

# 设置编译器 options：
# CXXFLAGS 传给 编译器和汇编器；LDFLAGS 传给 链接器。
# -Wall 开启所有警告；-O3 开启3级 optimization。开启后，编译时间和空间均增加，而生成的代码大小和运行时间均降低。大多数程序和库在编译器选择的是默认的 -O2 级别，这已经启用绝大多数安全的优化选项了; -march=native 按照特定的CPU进行优化，native表示当前CPU。
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# 检查编译器是否支持C++11特性
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
#    更推荐使用这个方式添加C++11特性支持
#    set(CMAKE_CXX_STANDARD 11)
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# 设置一些东西
# 为什么没用？
# LIST(APPEND CMAKE_MODULE_PATH "/Users/bayes/slam_lib/Sophus/build")

# 添加外部库
# 查找库时，可以用库的原名称; 使用随着 find_pacakge() 命令加载的变量时，使用全大写，如 EIGEN3, SOPHUS, PANGOLIN 等
# eigen
find_package(Eigen3 REQUIRED)
if (EIGEN3_FOUND)
    message("Found Eigen3")
    # 使用 message 打印路径或带有空格的字符串时，注意加 双引号 " "
    message("Path to Eigen3 header files: ${EIGEN3_INCLUDE_DIRS}")
else() 
    message("Not Found Eigen3")
endif()

# sophus 
find_package(Sophus REQUIRED)
if (Sophus_FOUND)
    message("Found Sophus")
    # 使用 message 打印路径或带有空格的字符串时，注意加 双引号 " "
    message("Path to Sophus header files: ${SOPHUS_INCLUDE_DIRS}")
else()
    message("Not Found Sophus")
endif()

# 添加额外的头文件搜寻目录
include_directories(BEFORE
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    # ${Sophus_INCLUDE_DIRS}
    "/Users/bayes/slam_lib/Sophus"
) # path relative to this CMakeLists.txt



# 设置编译生成的共享库的存放目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/shared)

# 一种看起来更贴近CMake的命名方式
# add_library(${PROJECT_NAME} SHARED
#     src/hello_slam.cc
# )
# add_executable(main src/main.cc)
# target_link_libraries(main ${PROJECT_NAME})

add_library(hello_slam SHARED
    src/hello_slam.cc
)
add_executable(main src/main.cc)
target_link_libraries(main
    hello_slam
    # ${EIGEN3_LIBS} # eigen库只有头文件（确实是这样，为什么可以这样呢？），没有库文件。因此不需要进行链接。
    ${Sophus_LIBRARIES}
)

