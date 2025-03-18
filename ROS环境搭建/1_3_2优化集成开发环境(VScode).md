### **📌 VS Code 在 ROS 开发中的使用**  
VS Code（Visual Studio Code）是一个轻量级的代码编辑器，支持 **C++ 和 Python** 开发，非常适合 **ROS 编程**。  

---

## **1. 安装 VS Code**
如果你的系统还没有安装 VS Code，可以使用以下命令安装：

```bash
sudo apt update
sudo apt install code -y
```
可能会报找不到code，可以从 [VS Code 官网](https://code.visualstudio.com/) 下载并安装。
### VS code版本选择
#### **✅ 1. 确认你的系统架构**
运行以下命令检查 CPU 架构：
```bash
uname -m
```
常见的结果：
- **`x86_64`** → 你需要 `amd64` 版本  
- **`armv7l` / `aarch64`** → 你需要 **ARM 版本** 

---

#### **✅ 2. 下载适用于 ARM 设备的 VS Code**
VS Code 的官方 `.deb` 版本 **不支持 ARM**，但微软提供了 **VS Code 的 ARM 版本**，你可以从官方 GitHub 下载安装：

##### **（1）如果你的系统是 32 位 ARM（armhf / armv7l）**
下载 `armhf` 版本。 

##### **（2）如果你的系统是 64 位 ARM（aarch64 / arm64）**
下载 `arm64` 版本。

---

## **2. 安装 VS Code 相关插件**
打开 **VS Code**，然后安装以下 **ROS 开发插件**：
1. **ROS**（官方插件，提供代码补全、启动调试等）
2. **C/C++**（用于 C++ 代码语法检查）
3. **Python**（用于 Python 代码语法检查）
4. **CMake**（用于 CMake 语法高亮）

📌 **安装方法**：
- 在 **VS Code 的扩展市场（Ctrl + Shift + X）** 搜索这些插件，点击 **安装**。

---

## **3. 使用 VS Code 编写 ROS 代码**
### ** 创建 ROS 工作空间**
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### ** 在 VS Code 打开 ROS 工作空间**
```bash
code ~/catkin_ws
```
这样就可以在 **VS Code** 里直接管理 ROS 代码了。

---

## **4. VS Code 调试 ROS 代码**
### **1. 运行 ROS 节点**
打开 **VS Code 终端**（`Ctrl + ~`）：
```bash
roscore
```
然后在 **另一个终端** 运行 ROS 节点：
```bash
rosrun my_hello_pkg hello_node.py
```

### **2. 配置 ROS C++ 调试**
如果你想在 **C++ 代码** 里添加断点并进行调试：
1. **安装 GDB 调试工具**：
   ```bash
   sudo apt install gdb -y
   ```
2. **在 `.vscode/launch.json` 中配置 GDB 调试**：
   ```json
   {
       "version": "0.2.0",
       "configurations": [
           {
               "name": "ROS C++ Debug",
               "type": "cppdbg",
               "request": "launch",
               "program": "${workspaceFolder}/devel/lib/my_hello_pkg/hello_node",
               "args": [],
               "stopAtEntry": false,
               "cwd": "${workspaceFolder}",
               "environment": [],
               "externalConsole": false,
               "MIMode": "gdb"
           }
       ]
   }
   ```
3. **编译代码并启动调试**：
   ```bash
   cd ~/catkin_ws
   catkin_make -DCMAKE_BUILD_TYPE=Debug
   ```

---

## **5. VS Code 高效 ROS 开发技巧**
### **🔹 快捷键**
| 功能 | 快捷键 |
|------|-------|
| 打开终端 | `Ctrl + ~` |
| 文件搜索 | `Ctrl + P` |
| 代码跳转 | `F12` |
| 代码格式化 | `Shift + Alt + F` |
| 终端切换 | `Ctrl + Tab` |

### **🔹 自动格式化 C++ 代码**
在 `settings.json` 中添加：
```json
{
    "editor.formatOnSave": true,
    "C_Cpp.clang_format_fallbackStyle": "Google"
}
```
这样 **每次保存代码时都会自动格式化**，保证代码风格一致。

---