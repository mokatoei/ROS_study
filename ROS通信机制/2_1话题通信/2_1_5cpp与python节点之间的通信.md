## **📌 ROS 中 C++ 节点和 Python 节点的通信**
在 ROS 中，C++ (`roscpp`) 和 Python (`rospy`) 的节点是**完全兼容**的，它们可以通过 **话题（Topics）、服务（Services）、动作（Actions）、参数服务器（Parameter Server）** 等方式相互通信。

---

## **1️⃣ 通过话题（Topic）通信**
话题通信是**发布/订阅模型（Pub/Sub）**，适用于**持续发送数据**的场景。

### **📌 C++ 发布，Python 订阅**
#### **📍 1.1 C++ 发布者**
📁 `talker.cpp`
```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "cpp_talker");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10);

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "Hello from C++!";

        pub.publish(msg);
        ROS_INFO("C++ Published: %s", msg.data.c_str());

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
```
---
#### **📍 1.2 Python 订阅者**
📁 `listener.py`
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo("Python Received: %s", msg.data)

rospy.init_node("python_listener")
rospy.Subscriber("chatter", String, callback)

rospy.spin()
```
---
#### **📍 1.3 运行**
```bash
source ~/catkin_ws/devel/setup.bash
roscore
```
**终端 1**（运行 C++ 发布者）：
```bash
rosrun my_pkg talker
```
**终端 2**（运行 Python 订阅者）：
```bash
rosrun my_pkg listener.py
```
---
### **📌 Python 发布，C++ 订阅**
如果反过来，只需要：
- 用 Python 编写发布者
- 用 C++ 编写订阅者

Python 版 `talker.py`：
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

rospy.init_node("python_talker")
pub = rospy.Publisher("chatter", String, queue_size=10)

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    msg = String()
    msg.data = "Hello from Python!"
    pub.publish(msg)
    rospy.loginfo("Python Published: %s", msg.data)
    rate.sleep()
```

C++ 版 `listener.cpp`：
```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

void callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("C++ Received: %s", msg->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cpp_listener");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("chatter", 10, callback);

    ros::spin();
    return 0;
}
```
---
## **2️⃣ 通过服务（Service）通信**
服务通信是**请求/响应（Request/Response）模型**，适用于**需要等待应答**的场景。

### **📌 C++ 服务器，Python 客户端**
#### **📍 2.1 C++ 服务器**
📁 `add_two_ints_server.cpp`
```cpp
#include "ros/ros.h"
#include "ros_tutorials/AddTwoInts.h"

bool add(ros_tutorials::AddTwoInts::Request &req,
         ros_tutorials::AddTwoInts::Response &res) {
    res.sum = req.a + req.b;
    ROS_INFO("C++ Server: %ld + %ld = %ld", req.a, req.b, res.sum);
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cpp_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("add_two_ints", add);

    ros::spin();
    return 0;
}
```
---
#### **📍 2.2 Python 客户端**
📁 `add_two_ints_client.py`
```python
#!/usr/bin/env python3
import rospy
from ros_tutorials.srv import AddTwoInts

rospy.init_node("python_client")
rospy.wait_for_service("add_two_ints")

try:
    add_two_ints = rospy.ServiceProxy("add_two_ints", AddTwoInts)
    response = add_two_ints(5, 3)
    rospy.loginfo("Python Client: 5 + 3 = %d", response.sum)
except rospy.ServiceException as e:
    rospy.logerr("Service call failed: %s", e)
```
---
#### **📍 2.3 运行**
```bash
rosrun my_pkg add_two_ints_server
rosrun my_pkg add_two_ints_client.py
```
**Python 客户端**会向**C++ 服务器**发送请求，并收到求和结果。

---

## **3️⃣ 通过参数服务器通信**
参数服务器（Parameter Server）可以存储和获取全局参数。

### **📌 C++ 设置参数，Python 读取参数**
#### **📍 3.1 C++ 设置参数**
📁 `set_param.cpp`
```cpp
#include "ros/ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "cpp_set_param");
    ros::NodeHandle nh;

    nh.setParam("robot_name", "ROS_Bot");
    ROS_INFO("C++ Set Param: robot_name = ROS_Bot");

    return 0;
}
```
---
#### **📍 3.2 Python 读取参数**
📁 `get_param.py`
```python
#!/usr/bin/env python3
import rospy

rospy.init_node("python_get_param")

robot_name = rospy.get_param("robot_name", "Unknown")
rospy.loginfo("Python Get Param: robot_name = %s", robot_name)
```
---
#### **📍 3.3 运行**
```bash
rosrun my_pkg set_param
rosrun my_pkg get_param.py
```
Python 端会获取到 C++ 设置的参数 `robot_name = ROS_Bot`。

---

## **📌 总结**
| **通信方式** | **模型** | **适用场景** | **C++ vs Python** |
|------------|--------|----------|----------------|
| 话题（Topic） | 发布/订阅 | 持续数据流 | `roscpp`、`rospy` |
| 服务（Service） | 请求/响应 | 需等待应答 | `roscpp`、`rospy` |
| 参数服务器（Parameter Server） | 读取/存储参数 | 全局参数共享 | `roscpp`、`rospy` |

在 ROS 中，**C++ 和 Python 节点完全兼容**，可以通过**话题、服务、参数服务器**等方式进行通信 🚀