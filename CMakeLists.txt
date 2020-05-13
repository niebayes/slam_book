# 添加额外的头文件的查找目录（除编译器默认的查找目录之外），命令中含 directories：
# 以下两条命令相当于 -I 参数。
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
# 该命令查找的对象为:The command searches for a file called <PackageName>Config.cmake or <lower-case-package-name>-config.cmake for each name specified。
# 可选参数：REQUIRED 则找不到就报错，不继续编译； QUIET 找不到也不报错，继续编译。该命令一般结合以上命令使用，通过配置多个变量，从而加强项的可移植性。
# find_package() 的两个模式 Module 和 Config，参考：https://zhuanlan.zhihu.com/p/97369704 。Module模式可以这样引入外部库： 
# LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules) 
# Config模式的引入参考： https://kb.kutu66.com/cmake/post_723600 
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





################################################################################

cmake_minimum_required(VERSION 3.0)
project(HelloSLAM) 


if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif(NOT CMAKE_BUILD_TYPE)


include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
) # path relative to this CMakeLists.txt

# 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
    src/hello_slam.cc
)

add_executable(main src/main.cc)
target_link_libraries(main hello_slam)

