#!/bin/bash

# 自动寻找src目录的函数
find_src_directory() {
    local current_dir="$PWD"
    
    # 在当前目录及其父目录中寻找src目录
    while [[ "$current_dir" != "/" ]]; do
        if [ -d "$current_dir/src" ]; then
            echo "$current_dir/src"
            return 0
        fi
        current_dir=$(dirname "$current_dir")
    done
    
    # 检查当前目录是否就是src目录
    if [[ $(basename "$PWD") == "src" ]] && [ -d "$PWD" ]; then
        echo "$PWD"
        return 0
    fi
    
    return 1
}

# 如果没有提供目录参数，自动寻找src目录
if [ $# -eq 0 ]; then
    echo "未指定目录参数，正在自动寻找src目录..."
    target_dir=$(find_src_directory)
    
    if [ $? -eq 0 ] && [ -n "$target_dir" ]; then
        echo "找到src目录: $target_dir"
    else
        echo "错误: 未找到src目录，请手动指定目录路径"
        echo "用法: $0 [目录路径]"
        echo "示例: $0 /path/to/your/workspace/src"
        exit 1
    fi
else
    target_dir="$1"
    echo "使用指定的目录: $target_dir"
fi

# 检查目录是否存在
if [ ! -d "$target_dir" ]; then
    echo "错误: 目录 '$target_dir' 不存在"
    exit 1
fi

# 获取所有子目录
subdirs=()
while IFS= read -r -d '' dir; do
    subdirs+=("$dir")
done < <(find "$target_dir" -maxdepth 1 -mindepth 1 -type d -print0)

# 检查是否有子目录
if [ ${#subdirs[@]} -eq 0 ]; then
    echo "在目录 '$target_dir' 中没有找到任何子目录"
    exit 1
fi

# 显示子目录列表
echo "找到以下子目录:"
echo "=================="
for i in "${!subdirs[@]}"; do
    dir_name=$(basename "${subdirs[$i]}")
    printf "%2d) %s\n" $((i+1)) "$dir_name"
done
echo " 0) 编译所有包 (all)"
echo "=================="

# 获取用户选择
while true; do
    read -p "请选择要编译的包编号 (0-${#subdirs[@]}): " choice
    
    # 检查输入是否为空
    if [ -z "$choice" ]; then
        echo "输入不能为空，请重新输入"
        continue
    fi
    
    # 检查输入是否为数字
    if ! [[ "$choice" =~ ^[0-9]+$ ]]; then
        echo "请输入有效的数字"
        continue
    fi
    
    # 检查数字范围
    if [ "$choice" -eq 0 ]; then
        # 编译所有包
        echo "开始编译所有包..."
        colcon build
        break
    elif [ "$choice" -ge 1 ] && [ "$choice" -le "${#subdirs[@]}" ]; then
        selected_index=$((choice-1))
        selected_dir="${subdirs[$selected_index]}"
        package_name=$(basename "$selected_dir")
        echo "开始编译包: $package_name"
        colcon build --packages-select "$package_name"
        break
    else
        echo "无效的选择，请输入 0-${#subdirs[@]} 之间的数字"
    fi
done

# 检查编译结果
if [ $? -eq 0 ]; then
    echo "编译成功完成!"
else
    echo "编译失败!"
    exit 1
fi
