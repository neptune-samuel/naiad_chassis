
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

### 功能

- 提供一个TCP端口实时返回所有串口数据，供调试使用
- 提供一个配置入口，可以将指定的属性通过映射到指定的TCP端口上
  如： 9601: 400,401,402,403,404
- 提供一个SACP客户端类，解析SACP属性上报，并将它打印出来，后续将它转成ROS消息


