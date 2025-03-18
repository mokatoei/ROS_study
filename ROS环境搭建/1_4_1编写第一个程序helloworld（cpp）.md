## **1. 介绍 ROS（Robot Operating System）**
**ROS（Robot Operating System，机器人操作系统）** 是一个用于机器人开发的开源软件框架。它提供了一系列工具、库和功能，让开发者可以更轻松地开发机器人应用程序。

### **ROS 的特点：**
- **模块化设计**：程序分为多个“节点”（Node），各自执行特定任务，如传感器读取、运动控制等。
- **消息传递机制**：不同节点可以通过“话题”（Topic）、“服务”（Service）或“动作”（Action）进行通信。
- **硬件抽象**：支持多种机器人硬件，如传感器、执行器、摄像头等。
- **丰富的工具**：包含调试、仿真、可视化等工具，如 `rviz`、`rqt`、`rosbag` 等。
- **支持多种编程语言**：主要支持 **C++（roscpp）** 和 **Python（rospy）**。

---

## **ROS 工作空间（Workspace）**
**工作空间（Workspace）** 是开发 ROS 项目的目录，所有的 ROS 代码和包都应该放在工作空间内。

### **ROS 工作空间的结构**
一个典型的 ROS 工作空间如下：
```
ros_ws/                 # 工作空间根目录
 ├── src/               # 源代码目录，所有 ROS 包存放在这里
 │   ├── my_hello_pkg/  # 这是一个 ROS 包
 │   ├── ...            # 其他 ROS 包
 │   └── CMakeLists.txt # 用于 catkin 的 CMake 配置
 ├── devel/             # 编译后生成的文件（类似于 build 目录）
 ├── build/             # 编译过程中产生的临时文件
 └── CMakeLists.txt     # 顶级 CMake 配置文件
```
其中：
- **`src/`**：存放所有 ROS 包的源码。
- **`build/`**：存放编译过程中产生的临时文件（一般不需要手动修改）。
- **`devel/`**：存放编译后的可执行文件、库文件等（用于开发环境）。
- **`CMakeLists.txt`（顶层）**：工作空间的 CMake 配置文件。


## **ROS 包（Package）**
在 ROS 里，每个程序都是一个“包”（Package），我们需要创建一个新的包来存放 Hello World 节点。
**ROS 包（Package）** 是 ROS 中的基本单位，每个包包含一个独立的功能模块，如控制算法、驱动程序、传感器处理等。

### **ROS 包的结构**
一个典型的 ROS 包如下：
```
my_hello_pkg/            # ROS 包
 ├── src/                # 源代码（.cpp 或 .py）
 │   ├── hello_world.cpp # 示例 C++ 代码
 ├── include/            # 头文件目录（C++ 需要）
 ├── launch/             # 启动文件（可选）
 ├── msg/                # 自定义消息（可选）
 ├── srv/                # 自定义服务（可选）
 ├── CMakeLists.txt      # CMake 配置文件
 ├── package.xml         # 包的元数据（依赖、版本等）
 └── README.md           # 说明文件（可选）
```

### **如何创建 ROS 工作空间**
**创建工作空间**
   ```bash
   mkdir -p ~/ros_ws/src
   cd ~/ros_ws
   catkin_make
   ```
---

### **如何创建 ROS 包**
1. **进入工作空间的 `src/` 目录**
   ```bash
   cd ~/ros_ws/src
   ```

2. **创建 ROS 包**
   ```bash
   catkin_create_pkg my_hello_pkg roscpp rospy std_msgs
   ```
   - **catkin_create_pkg**
    ROS 提供的命令，用于创建一个新的 Catkin 结构的 ROS 包。
   - **my_hello_pkg**
    这是新建的 ROS 包名，你可以换成任何你想要的名字（只能使用小写字母、数字和下划线 _）。
   - **roscpp**
    这是 C++ 版本的 ROS 客户端库，用于 C++ 编写的 ROS 代码（如果只用 Python，则不需要它）。
   - **rospy**
    这是 python 版本的 ROS 客户端库，用于 python 编写的 ROS 代码（如果只用 C++，则不需要它）。
   - **std_msgs**
    这是 ROS 标准消息类型库，包含 std_msgs/String、std_msgs/Int32 等基础消息类型，在节点间传递消息时常用。

3. **进入新创建的包**
    ``` bash
    cd ~/ros_ws/src/my_hello_pkg
    ```
4. **编写 Hello World 节点**
💡 ROS 节点是 C++ 或 Python 程序，可以用 C++ 或 python 实现一个最简单的 Hello World 节点。
    - 进入src目录，创建hello_world.cpp文件。
    ``` bash
        cd src
        vim hello_world.cpp
    ```
    - 编辑hello_world.cpp文件，输入以下内容
    ``` cpp
        #include "ros/ros.h"
        int main(int argc, char **argv)
        {
            // 初始化 ROS 节点
            ros::init(argc, argv, "hello_world_node");

            // 创建 ROS 句柄
            ros::NodeHandle nh;

            // 打印 Hello World
            ROS_INFO("Hello, ROS World!");

            return 0;
        }
    ```

5. **配置CMakelist.txt文件**
    编译对应包内的CMakelist.txt文件。
    ``` bash
     ~/ros_ws/src/my_hello_pkg/CMakeLists.txt
    ```
    在该文件内加入以下几行
    ``` bash
    add_executable(hello_world_node src/hello_world.cpp)
    target_link_libraries(hello_world_node ${catkin_LIBRARIES})
    ```
   
6. **编译**
    回到工作空间根目录
    ```bash
    cd ~/ros_ws
    catkin_make 
    ```
    执行以下命令检查编译是否成功
    ``` bash
        ls devel/lib/my_hello_pkg/
    ```

7. **运行 Hello World 程序**
    1. 首先启动 ROS  
        ``` bash
        roscore
        ```
     不要关闭这个终端，在新终端执行下面的命令。  
    2. 运行 Hello World 节点
     ``` bash
        source ./devel/setup.bash 
        rosrun my_hello_pkg hello_world_node
     ```
     第一条语句的作用是加载ROS工作空间的环境变量，运行 source ./devel/setup.bash 之后，ROS环境变量，使 `rosrosrun 和roslaunch 能够找到你的包。
     终端应该输出：
     ``` css 
      [ INFO] [时间戳]: Hello, ROS World!
     ```
---
