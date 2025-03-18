## **ROS Launch 文件 (`.launch`)**
在 ROS 里，**launch 文件**（扩展名 `.launch`）用于同时启动多个节点，并配置参数、日志、环境变量等。  

---

## **📌 1. 创建 `launch` 文件**
假设你的 Python 代码是 `hello_vscode_test.py`，你可以创建一个 `launch` 文件，名字如 `hello_vscode.launch`。

**🔹 目录结构**：
```bash
demo02_ws/
├── src/
│   ├── hello_vscode/
│   │   ├── scripts/
│   │   │   ├── hello_vscode_test.py  # Python 代码
│   │   ├── launch/
│   │   │   ├── hello_vscode.launch   # Launch 文件
```

---

## **📌 2. 编写 `hello_vscode.launch`**
创建 `hello_vscode/launch/hello_vscode.launch`，写入以下内容：
```xml
<launch>
    <!-- 启动 Python 节点 -->
    <node name="hello_node" pkg="hello_vscode" type="hello_vscode_test.py" output="screen"/>
</launch>
```
🔍 **解析**：
- `<node>` 代表一个 ROS 节点
- `name="hello_node"`：ROS 节点的名字   //可以随便取名但是不能重复
- `pkg="hello_vscode"`：你的 ROS **包名**
- `type="hello_vscode_test.py"`：你的 Python **脚本文件**；如果你的节点是 C++ 可执行文件：type 是编译后的 可执行文件名（通常在 devel/lib/your_pkg/ 目录）
- `output="screen"`：让日志信息打印到终端

---

## **📌 3. 运行 `launch` 文件**
在 **工作空间** 目录下运行：
```bash
source devel/setup.bash
roslaunch hello_vscode hello_vscode.launch
```
💡 这会 **自动运行你的 Python 代码**！

---

## **📌 4. `launch` 文件的常见功能**
### **✅ 设置参数**
```xml
<param name="message" value="Hello from ROS!"/>
```
在 Python 里获取：
```python
import rospy
msg = rospy.get_param("message", "default_value")
print(msg)
```

### **✅ 启动多个节点**
```xml
<launch>
    <node name="node1" pkg="hello_vscode" type="script1.py"/>
    <node name="node2" pkg="hello_vscode" type="script2.py"/>
</launch>
```
这会 **同时启动 `script1.py` 和 `script2.py`**。

---

## **📌 5. 遇到 `ERROR: cannot launch node`？**
如果你 `roslaunch` 时报错：
```
ERROR: cannot launch node of type [hello_vscode/hello_vscode_test.py]: can't locate node
```
可能是因为 **Python 脚本没有可执行权限**，执行：
```bash
chmod +x ~/demo02_ws/src/hello_vscode/scripts/hello_vscode_test.py
```
然后重新 `roslaunch`！🚀

---

## **✅ 结论**
`launch` 文件让你可以 **同时启动多个 ROS 节点**，并进行 **参数配置** 和 **日志管理**。  
创建 `hello_vscode.launch`，然后运行：
```bash
roslaunch hello_vscode hello_vscode.launch
```
**这样 Python 代码就能自动运行了！**