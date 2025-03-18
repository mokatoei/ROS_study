## **📌 Python 实现 ROS Hello World 节点**
在 **C++** 里，我们用 `roscpp` 编写 ROS 节点，而在 **Python** 里，我们使用 `rospy` 进行开发。

---

### **1️⃣ Python 版 Hello World**
创建一个 **Python 版本的 ROS 发布者**，发布 `Hello ROS!` 消息。

📁 **创建 Python 脚本**
```bash
cd ~/catkin_ws/src/my_hello_pkg/scripts
touch hello_node.py
chmod +x hello_node.py
```

✏️ **编辑 `hello_node.py`**
```python
#!/usr/bin/env python3
import rospy  # 导入 ROS Python 库
from std_msgs.msg import String  # 导入 String 消息类型

def main():
    rospy.init_node("hello_node")  # 初始化节点
    pub = rospy.Publisher("chatter", String, queue_size=10)  # 创建发布者
    rate = rospy.Rate(10)  # 10Hz 发送数据

    while not rospy.is_shutdown():  # 循环运行，直到按 Ctrl+C 退出
        msg = String()
        msg.data = "Hello ROS!"
        pub.publish(msg)  # 发布消息
        rospy.loginfo("已发布: %s", msg.data)  # 终端打印
        rate.sleep()  # 按照 10Hz 发送数据

if __name__ == "__main__":
    main()
```

---

### **2️⃣ 运行 Python 版 ROS 节点**
编译 `catkin_ws`：
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
然后运行节点：
```bash
rosrun my_hello_pkg hello_node.py
```
如果 `rosrun` 提示找不到命令，确保 `hello_node.py` 可执行：
```bash
chmod +x ~/catkin_ws/src/my_hello_pkg/scripts/hello_node.py
```

---

## **📌 C++ 和 Python 的区别**
| **对比项** | **C++ (`roscpp`) | **Python (`rospy`)** |
|------------|------------------|------------------|
| **执行速度** | 快，适合实时应用 | 慢，适合非实时任务 |
| **开发难度** | 复杂，需要编译 | 简单，无需编译 |
| **依赖库** | `roscpp` | `rospy` |
| **适用场景** | 机器人控制、图像处理等高性能任务 | 传感器数据处理、调试脚本 |
| **消息发布** | `ros::Publisher` | `rospy.Publisher` |
| **日志打印** | `ROS_INFO("msg")` | `rospy.loginfo("msg")` |

### **✅ 什么时候选 C++，什么时候选 Python？**
- **用 C++**（`roscpp`）：需要高性能、实时性，如 **机器人控制**、**图像处理**。
- **用 Python**（`rospy`）：开发快、调试方便，如 **数据分析**、**调试脚本**、**非实时任务**。
