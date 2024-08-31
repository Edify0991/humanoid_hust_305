# SV660P_Ros_Contorl
Using ros1 to realize communicating between PC and SV660P motor driver.

## 编译准备

### serial第三方库

* 从github上clone到本地，无需与本项目在同一工作空间

``` shell
git clone https://github.com/wjwwood/serial.git
```

* 进入上述功能包中后，利用cmake进行编译

``` cmake
make
```

* Build and run the tests:

``` cmake
make test
```

* Build the documentation:

``` cmake
make doc
```

* Install:

``` cmake
make install
```

### 修改功能包CMakelists.txt文件

* 设置serial库的cmake配置文件所在地址路径，文件名为`serialConfig.cmake`，`serial_DIR`变量定义可在该文件中查找得到

``` cmake
set(serial_DIR /home/edifykd/Code/serial/build/devel/share/serial/cmake)
```

* 利用`Cmake`的`find_package`寻找第三方依赖包的 `.cmake`文件，并根据`.cmake`文件生成依赖包的 _头文件目录_ 和 _库文件路径_ 等。由于第三方库不一定在默认查找路径中，故需要先进行上一步设置路径

``` cmake
find_package(serial REQUIRED)
```

* 添加头文件路径 （Additional Include Directories），头文件路径变量名`serial_INCLUDE_DIRS`可在`serialConfig.cmake`中找到

```cmake
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
)
```

*添加库文件路径   (Additional Library Directories），库文件路径变量名`serial_LIBRARY_DIRS`可在`serialConfig.cmake`中找到

``` cmake
link_directories(
  ${catkin_LIB_DIRS}
  ${serial_LIBRARY_DIRS}
)
```

* 链接库文件 （Link Library Dependencies），库文件变量名`serial_LIBRARIES`可在`serialConfig.cmake`中找到

``` cmake
target_link_libraries(wit_sensor_node
  ${catkin_LIBRARIES}
  ${serial_LIBRARIES}
)
```

### 存疑问题

* 两台控制器以网线相连，算作两个CAN通道吗？

答：两台控制器以串联方式，串联在一条CAN通道上

试试fetch，再试试，改改

lallala
kjjjjjjjjjjjjj