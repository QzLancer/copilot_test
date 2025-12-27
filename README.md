# copilot_test

一个简单的C++项目，支持在Codespaces中调试，并配置了GitHub Actions CI。

## 项目结构

```
.
├── main.cpp                    # 主程序源文件
├── CMakeLists.txt             # CMake构建配置
├── .vscode/
│   ├── launch.json            # 调试配置
│   └── tasks.json             # 构建任务配置
└── .github/workflows/
    └── build.yml              # GitHub Actions CI配置
```

## 功能特性

- ✅ 简单的C++ Hello World程序
- ✅ 使用CMake构建系统
- ✅ VS Code调试配置（适用于Codespaces）
- ✅ GitHub Actions自动构建和测试

## 本地构建

### 前置要求

- CMake 3.10 或更高版本
- C++编译器（g++或clang++）
- GDB（用于调试）

### 构建步骤

```bash
# 配置项目
cmake -B build -DCMAKE_BUILD_TYPE=Debug

# 编译
cmake --build build

# 运行
./build/copilot_test
```

## 在Codespaces中使用

1. 打开此仓库的Codespace
2. 安装GDB（如果尚未安装）：
   ```bash
   sudo apt-get update && sudo apt-get install -y gdb
   ```
3. 按 `F5` 或使用调试面板启动调试
4. 在 `main.cpp` 中设置断点进行调试

## 使用VS Code构建任务

- 按 `Ctrl+Shift+B`（或 `Cmd+Shift+B` on Mac）选择构建任务
- 选择 "build" 任务来配置和编译项目

## GitHub Actions

每次推送到 `main` 或 `master` 分支，或创建针对这些分支的Pull Request时，GitHub Actions会自动：

1. 检出代码
2. 安装必要的依赖
3. 配置CMake
4. 编译项目
5. 运行可执行文件

查看Actions选项卡可以查看构建状态和日志。

## 开发

修改 `main.cpp` 来添加你自己的C++代码。项目使用C++11标准。

## License

MIT