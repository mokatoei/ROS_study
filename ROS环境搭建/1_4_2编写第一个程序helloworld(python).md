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

✏️ **编辑 `CMakeList.txt`**
找到对应的语句进行添加
``` bash
catkin_install_python(PROGRAMS
    scripts/hello_node.py  # 这里的路径指向你的 Python 代码
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
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


有时候我们执行rosrun时会碰到错误 `bad interpreter: No such file or directory` 中，**根本原因是 `#!/usr/bin/env python` 找不到 `python` 解释器**。  

**创建软链接的原因**：
1. **兼容性问题**：
   - 有些系统默认安装 **Python 3**，但没有 `python`（只有 `python3`）。
   - 旧代码可能使用 `#!/usr/bin/env python`，但 `python` 在你的系统里可能不存在。

2. **解决 `rosrun` 无法找到 Python 解释器的问题**：
   - ROS 旧版本（如 Noetic）默认使用 **Python 2**，但 Ubuntu 20.04 以后默认安装 **Python 3**，可能没有 `python` 命令。
   - 你可以用软链接让 `python` 指向 `python3`，这样旧脚本也能运行：
     ```bash
     sudo ln -s /usr/bin/python3 /usr/bin/python
     ```
   - **这样，`#!/usr/bin/env python` 也能找到 `python3`，不会报错**。

---

## **🛠 是否需要创建软链接？**
- **如果你的 `hello_vscode_test.py` 里是 `#!/usr/bin/env python3`，就不需要软链接**。
- **如果是 `#!/usr/bin/env python`，但 `python` 命令不存在，你就需要创建软链接**。

---

## **📌 总结**
💡 **当 `python` 命令不存在，但 `python3` 存在时，你可以创建软链接**：
```bash
sudo ln -s /usr/bin/python3 /usr/bin/python
```
这样旧的 `python` 代码仍然可以运行，避免 `bad interpreter` 错误。

你可以先运行：
```bash
which python
```
如果没有输出，说明 `python` 解释器不存在，你就需要创建软链接。