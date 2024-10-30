# ROS2 Basics

>在Ubuntu22.04安装对应版本的ROS2以后，添加ROS2到环境变量  
>主要参考：https://docs.ros.org   
>本文档主要是对ROS2学习的一个总结以及指令速查

## Configuration Parts

### Nodes 节点

节点是ROS2里面独立运行的一个单元，主要执行独立的程序。例如，对于机器人的视觉识别与运动而言，这是两个不同的部分，又不同的节点负责。
两个节点会通过某些特定的方式进行信息交换，从而更好的执行特定的任务。

在终端使用ros2运行特定程序，
```
ros2 run <package_name> <executable_name>
```
可以通过
```
ros2 node list
```
查看当前运行的节点，终端会返回
```
/node_name_1
...
/node_name_n
```
其他节点操作：
```
ros2 run <package_name> <executable_name> --ros-args --remap __node:=my_node //对节点进行重命名
ros2 node info <node_name> //返回节点信息，没有特定节点名称则会返回大量节点相关的信息
```
### Topics 话题

话题是ROS2当中重要的消息形式，接受者节点可以通过话题接受来自其他节点（发布者）的信息，同一节点下可能存在多个发布者和接受者。
话题操作
```
ros2 topic list //返回当前运行程序的话题信息（名称）
ros2 topic list -t //返回当前运行程序的话题名称和类型
ros2 topic echo <topic_name> //返回当前话题接受到的信息
ros2 topic info <topic_name> //返回当前话题的类型，发布者数量和接受者数量
ros2 topic find <topic_type> //返回该话题类型的话题
```
其中比较重要的是话题的类型，这个决定了节点交换的信息的形式；
我们可以认为节点通过某个接口发送某种特定类型的信息到对应话题下被其他节点接受，此处涉及一个新的概念```interface```
```
ros2 interface show <topic_type> //可以查看具体的话题类型下的变量，也就实际传递的消息类型
ros2 topic pub <topic_name> <msg_type> '<args>'//根据上面的信息，用户可以充当发布者可以根据话题内容发送特定的信息到节点从而改变接受者的行为
ros2 topic hz <node/node_parameter> //可以查看接受者接受信息的频率
ros2 topic bw <node/node_parameter> //可以查看接受者接受话题信息的带宽
```
### Parameters 参数

参数是节点内重要的数据信息，直接影响节点内的程序运行和状态。
参数操作
```
ros2 param list //查看当前执行程序的所有参数
ros2 param get <node_name> <parameter_name> //查看某节点某数据的类型和值、
ros2 param set <node_name> <parameter_name> <value> //透过终端对执行程序中的参数值进行修改
ros2 param dump <node_name> 查看某一节点的所有参数和参数值
ros2 param load <node_name> <parameter_file> 从文件中读取参数并对节点参数进行修改
```

### Actions 动作

### Services 服务

### Subscriber

### Publisher

### 

### 
