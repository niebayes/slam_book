CMake 项目结构 
---
- workspace 
  - bin             , 编译得到的二进制文件
  - build           , 编译生成的文件
  - data            , 项目使用的文本数据
  - doc             , 项目文档
  - images          , 项目使用的图片数据
  - include         , 头文件
  - lib             , 本项目编译得到的库
  - modules         , 存放 Find<package_name>.cmake 文件，以找到所需的外部库
  - src             , 项目源代码
  - tmp             , 临时文件
  - test            , 用于项目测试的源代码
  - utils           , 通用辅助函数的源代码
  - .gitignore      , git .gitignore 配置文件
  - build.sh        , 用于一键编译项目的 Shell 脚本
  - CMakeLists.txt  , CMake 配置文件
  - README.MD       , 项目说明