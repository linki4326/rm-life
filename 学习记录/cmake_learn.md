# cmake使用指南
## 注释
使用#进行注释 多行注释使用 #[[]] 内容写在两个中括号当中

## 三连(定义一些基本的东西 完成最基础的构建)
cmake_minimum_required(VERSION 3.0)  //指定使用的最低版本
project() //给项目命名
add_executable(可执行文件名称 源文件名/变量名)

## set的使用
### 定义变量
set(变量名 源文件)//可用set定义变量来装源文件
如set(SRC main.cpp camra.cpp)
add_executable(main ${SRC}) //这时候就不用将所有源文件都一一列举
### 指定使用的c++标准
由于在编写程序的时候 会用到不同的c++标准 不同的版本都有新特性 所以需要在编译时指定使用哪个标准
set(CMAKE_CXX_STANDARD 11) //使用c++11作为标准
ps:也可以在使用cmake命令构建makefile时在后面指定标准 如 cmake .. -DCMAKE_CXX_STANDARD=11
### 指定输出的路径
set(EXECUTABLE_OUTPUT_PATH 指定的路径)//路径可以是相对路径 也可以是绝对路径 还可以set一个变量来装这个路径

## 搜索文件
### 使用aux_source_directory(搜索路径 变量名)
在这个路径下寻找所有源文件 并将这些源文件列表储存在这个变量里面
### 使用file(GLOB 变量名 搜索路径/*.cpp等)
没什么好说的 第一个位置表示按通配符去磁盘扫描文件，把符合条件的文件路径放到变量里

## 指定头文件路径
### 一种是直接在源文件的头文件中使用相对路径
### 一种是在cmake文件中包含
使用 include_directories(头文件的路径)

## 制作库
库分为动态库(.so/.dll 为后缀) 静态库(.a/.lib 为后缀)
使用add_library(库名 SHARED/STATIC 要生成库的源文件)//使用SHARED为动态 STATIC为静态
同时可以使用set(LIBRARY_OUTPUT_PATH 存放生成的库的路径)

## 链接库//一般写在生成可执行文件之后
### 静态库
使用link_libraries(STATIC 库名)//一般用于链接系统自带的库

link_directories(库名 库的路径)
### 动态库
target_link_libraries(目标文件 PUBLIC/PRIVATE 要连接的库)
