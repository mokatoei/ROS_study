### **📌 Terminator：强大的多窗口终端工具**
**Terminator** 是一个支持 **多窗口**、**分屏**、**标签页** 的终端模拟器，非常适合 **ROS 开发**，因为我们通常需要同时打开多个终端窗口来运行不同的 ROS 节点、指令等。

---

## **1️⃣ 安装 Terminator**
如果你的系统没有安装，可以用以下命令安装：
```bash
sudo apt update
sudo apt install terminator -y
```
然后可以直接在终端输入：
```bash
terminator
```
或者按 `Alt + F2`，输入 `terminator` 启动。

---

## **2️⃣ Terminator 基本用法**
| **操作** | **快捷键** |
|----------|------------|
| **水平分屏** | `Ctrl + Shift + O` |
| **垂直分屏** | `Ctrl + Shift + E` |
| **关闭当前终端** | `Ctrl + Shift + W` |
| **切换窗口** | `Ctrl + Tab` 或 `Alt + 箭头键` |
| **全屏模式** | `F11` |
| **打开新 Terminator 窗口** | `Ctrl + Shift + I` |
| **复制** | `Ctrl + Shift + C` |
| **粘贴** | `Ctrl + Shift + V` |

---

## **3️⃣ 适用于 ROS 的 Terminator 终端布局**
在 ROS 开发中，通常需要多个终端，比如：
1. **主终端**：运行 `roscore`
2. **第二个终端**：运行你的 ROS 节点
3. **第三个终端**：查看话题 `rostopic echo /chatter`
4. **第四个终端**：监控节点 `rosnode list`

使用 Terminator，可以**一次性打开所有窗口**，不用来回切换终端：
1. **打开 Terminator**
2. **按 `Ctrl + Shift + O`** 水平分割
3. **按 `Ctrl + Shift + E`** 垂直分割
4. **在不同窗口输入不同的 ROS 命令**

最终你会得到一个 **四分屏** 终端，方便 ROS 调试！

---

## **4️⃣ 自动启动多个终端（使用 Terminator 配置）**
如果你不想每次手动分屏，可以**创建 Terminator 配置文件**，让它自动打开多个窗口并运行特定命令。

### **📁 创建 Terminator 配置**
编辑配置文件：
```bash
nano ~/.config/terminator/config
```
添加以下内容：
```ini
[global_config]
  title_hide_size = 0

[layouts]
  [[ros]]
    [[[child1]]]
      type = Terminal
      command = roscore
      parent = window0
    [[[child2]]]
      type = Terminal
      command = rosrun my_hello_pkg hello_node.py
      parent = window0
    [[[child3]]]
      type = Terminal
      command = rostopic echo /chatter
      parent = window0
    [[[child4]]]
      type = Terminal
      command = rosnode list
      parent = window0
  [[window0]]
    type = Window
    parent = ""

[plugins]
```
然后运行：
```bash
terminator -l ros
```
它会自动打开四个窗口，并运行 **roscore、rosrun、rostopic、rosnode list**！🚀

---

## **📌 总结**
✅ **Terminator 是 ROS 开发的神器**，可以：
- **多终端分屏**
- **同时运行多个 ROS 命令**
- **使用快捷键快速切换**
- **自动启动多个 ROS 终端**

💡 **建议**：用 Terminator 管理你的 ROS 开发环境，提高工作效率！🚀