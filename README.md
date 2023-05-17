
### 纳百机器人OS - 机架驱动

 - 对接主控的传感器数据，转换为ROS消息
 - 接收算法指令，发布给主控
 - 实现调试的其他功能


### 编译

```sh
mkdir build 
cd build
cmake ..

# build 
cmake --build .

# install
cmake --install . [--prefix /usr/local]

```

