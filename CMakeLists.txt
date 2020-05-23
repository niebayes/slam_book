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


# 一些可能有用的变量 
# 与项目及 CMakeLists.txt 文件有关的 目录 变量
# CMAKE_SOURCE_DIR # 源码树的顶级源文件目录
# CMAKE_BINARY_DIR # 源码数的顶级构建目录
# CMAKE_CURRENT_BINARY_DIR # 当前正在处理的构建目录。每个由add_subdirectory() 添加的目录将会在构建树中创建一个构建目录。
# CMAKE_CURRENT_LIST_DIR # 当前处理的CMakeLists.txt文件所在的目录。
# CMAKE_CURRENT_SOURCE_DIR # 当前处理的源码目录。
# CMAKE_HOME_DIRECTORY # 源码树的顶级目录的路径。

# CMake 版本号 
# CMAKE_VERSION # 全版本号
# CMAKE_MAJOR_VERSION # 主版本号
# CMAKE_MINOR_VERSION # 次版本号

# 与项目名称有关的变量
# CMAKE_PROJECT_NAME # the name from the last project call from the root CMakeLists.txt
# PROJECT_NAME # the name from the last project call, regardless from the location of the file containing the command. More info, see: https://stackoverflow.com/a/38940669
# PROJECT_SOURCE_DIR # 当前工程的源文件目录
# PROJECT_BINARY_DIR # 当前工程的构建目录
# <project_name>_SOURCE_DIR # <project_name> 的源文件目录
# <project_name>_BINARY_DIR # <project_name> 的构建目录

# 与编译有关的
# 当开启时创建verbose的构建文件。默认为false.当设置为真时，CMake会产生verbose构建文件，它会显示使用到的每个命令行。
# CMAKE_VERBOSE_MAKEFILE # set(CMAKE_VERBOSE_MAKEFILE ON)

# More info about cmake package and making package, see: https://cmake.org/cmake/help/git-master/manual/cmake-packages.7.html#cmake-packages-7

################################################################################

# The FATAL_ERROR option is accepted but ignored by CMake 2.6 and higher. It should be specified so CMake versions 2.4 and lower fail with an error instead of just a warning.
# cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
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
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
set(CMAKE_CXX_STANDARD 11)

# 检查编译器是否支持C++11特性
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-dev") 
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
#    更推荐使用这个方式添加C++11特性支持
#    set(CMAKE_CXX_STANDARD 11)
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# 添加外部库
# eigen got。不需要 link eigen_lib
# sophus got (not by find_package() however)。好像不需要 link sophus_lib
# pangolin got。需要 link pangolin_lib
# opencv got。需要 link opencv_lib
# ceres got。需要 link ceres_lib 
# g2o got。需要 link G2O_CORE_LIBRARY and G2O_STUFF_LIBRARY （可能有时候只需要一个？？）。经测试，不能将 g2o 的 LIBRARY 写成 LIBRARIES 
# pcl got。 
# googletest got。

# 查找库时，可以用库的原名称; 使用随着 find_pacakge() 命令加载的变量时，使用全大写，如 EIGEN3, SOPHUS, PANGOLIN 等
# eigen
find_package(Eigen3 REQUIRED)
if (EIGEN3_FOUND)
    message("Found Eigen3" " Path ${EIGEN3_INCLUDE_DIRS}")
    # 使用 message 打印路径或带有空格的字符串时，注意加 双引号 " "
else() 
    message("Not Found Eigen3")
endif()

# sophus 
# To use Sophus, follow the instructions below. 
# find_package(Sophus REQUIRED)
# target_link_libraries(target Sophus::Sophus)
find_package(Sophus REQUIRED)
if (Sophus_FOUND)
    message("Found Sophus")
    include_directories(${Sophus_INCLUDE_DIRS})
    # 使用 message 打印路径或带有空格的字符串时，注意加 双引号 " "
else()
    message("Not Found Sophus. Manually add it.")
    include_directories("/Users/bayes/slam_lib/Sophus")
endif()

# Panlogin
find_package(Pangolin REQUIRED)
if (Pangolin_FOUND)
    message("Found Pangolin" " Path ${Pangolin_INCLUDE_DIRS}")
else()
    message("Not Found Pangolin")
endif()

# OpenCV 
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    message("Found OpenCV" " Path ${OpenCV_INCLUDE_DIRS}")
else()
    message("Not Found OpenCV")
endif()

# Ceres
find_package(Ceres REQUIRED)
if (CERES_FOUND)
    message("Found Ceres" " Path ${CERES_INCLUDE_DIRS}")
else()
    message("Not Found Ceres")
endif()

# g2o
# brew安装的版本是 2017/07/30 发布的老版本 g2o。 新版本 g2o 相较于老版本有一定的变化：把原来的祼指针统一改成了智能指针。因此不要用brew安装，而自己从 Github pull 下来再编译。
# g2o 链接库时，需使用 G2O_CORE_LIBRARY and G2O_STUFF_LIBRARY，而不是一般的写法：G2O_LIBRARIES
# 这条命令在某些时候有用(??)
# list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(G2O REQUIRED)
if (G2O_FOUND)
    message("Found g2o" " Path ${G2O_INCLUDE_DIRS}")
else()
    message("Not Found g2o")
endif()

# pcl 
# pcl是一个模块组成的库，因此在 find_pacakge() 时需指定查找的模块(??一定要吗)：
# find_package(PCL 1.3 REQUIRED COMPONENTS common io)
find_package(PCL REQUIRED)
if (PCL_FOUND)
    message("Found pcl" " Path ${PCL_INCLUDE_DIRS}")
else()
    message("Not Found pcl")
endif()

# glog 
# To use glog, follow instructions below.
# find_package(glog REQUIRED) 
# target_link_libraries(target glog::glog)

# 添加额外的头文件搜寻目录
include_directories(BEFORE
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    # ${Sophus_INCLUDE_DIRS}
    "/Users/bayes/slam_lib/Sophus"
    ${Pangolin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    # "/Users/bayes/slam_lib/g2o"
    # ${PCL_INCLUDE_DIRS}
) # path relative to this CMakeLists.txt



# 设置编译生成的共享库的存放目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/shared)

# 一种看起来更贴近CMake的命名方式
# add_library(${PROJECT_NAME} SHARED
#     src/hello_slam.cc
# )
# add_executable(main src/main.cc)
# target_link_libraries(main ${PROJECT_NAME})

# 作为主项目编译时的配置：
# add_library(hello_slam SHARED
#     src/hello_slam.cc
# )
# add_executable(main src/main.cc)
# target_link_libraries(main
#     ${Pangolin_LIBRARIES}
    # ${CERES_LIBRARIES}
# )

# 添加子项目，将以上的配置应用到子项目中 
# add_subdirectory(<source_dir> <binary_dir>) # 指定子项目的 CMakeLists.txt 的所在目录 及 子项目将要生成的可执行文件的目录。如未指定 <binary_dir> 那么 <binary_dir> = <source_dir> 
# 也可以在子项目通过 set(CMAKE_RUNTIME_OUTPUT_DIRECTORY <dir>) 来设置 <binary_dir>
# 以上两种方法并不一样：如果未指定 <binary_dir> 且 设置了 CMAKE_RUNTIME_OUTPUT_DIRECTORY。那么 <source_dir> 和CMAKE_RUNTIME_OUTPUT_DIRECTORY 这两个位置都会有 binary output files。
# 因此，（暂时）还是需要在 add_subdirectory() 指定 <binary_dir> 而不是在子项目中设置变量。

# 检测各个开源库是否能被正确找到并编译
# add_subdirectory(src/ch4/trajectory_error)
# add_subdirectory(src/ch5/image_basics)
# add_subdirectory(src/ch6/ceres_curve_fitting)
# add_subdirectory(src/ch6/g2o_curve_fitting)
# add_subdirectory(tmp/pcl_write)

# My codes 
add_subdirectory(src/ch6/my_codes ${CMAKE_BINARY_DIR}/_my_codes_build)
