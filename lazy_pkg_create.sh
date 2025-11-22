#!/bin/bash

# ROS2 包创建助手脚本

echo "=========================================="
echo "      ROS2 包创建助手"
echo "=========================================="

echo "您要保证在工作目录例如ros_ws,我将要进入src目录"
cd src

# 获取包名
read -p "请输入包名: " PKG_NAME

if [ -z "$PKG_NAME" ]; then
    echo "错误：包名不能为空！"
    exit 1
fi

# 选择构建类型
echo ""
echo "请选择构建类型:"
echo "1) ament_python (Python)"
echo "2) ament_cmake (C++)"
read -p "请输入选择 (1 或 2): " BUILD_TYPE_CHOICE

case $BUILD_TYPE_CHOICE in
    1)
        BUILD_TYPE="ament_python"
        DEFAULT_DEPS="rclpy"
        ;;
    2)
        BUILD_TYPE="ament_cmake"
        DEFAULT_DEPS="rclcpp"
        ;;
    *)
        echo "错误：无效的选择，使用默认值 ament_python"
        BUILD_TYPE="ament_python"
        DEFAULT_DEPS="rclpy"
        ;;
esac

# 获取依赖项
echo ""
echo "默认依赖: $DEFAULT_DEPS"
read -p "请输入额外的依赖项（用空格分隔，直接回车跳过）: " EXTRA_DEPS

if [ -n "$EXTRA_DEPS" ]; then
    DEPENDENCIES="$DEFAULT_DEPS $EXTRA_DEPS"
else
    DEPENDENCIES="$DEFAULT_DEPS"
fi

# 获取节点名
echo ""
read -p "请输入节点名 [默认: $PKG_NAME]: " NODE_NAME
NODE_NAME=${NODE_NAME:-$PKG_NAME}

# 获取维护者信息
echo ""
read -p "请输入维护者姓名 [默认: ROS Developer]: " MAINTAINER_NAME
MAINTAINER_NAME=${MAINTAINER_NAME:-"ROS Developer"}

read -p "请输入维护者邮箱 [默认: rosdev@example.com]: " MAINTAINER_EMAIL
MAINTAINER_EMAIL=${MAINTAINER_EMAIL:-"rosdev@example.com"}

# 获取描述
echo ""
read -p "请输入包描述 [默认: no description]: " DESCRIPTION
DESCRIPTION=${DESCRIPTION:-"no description"}

# 设置许可证
LICENSE="Apache-2.0"

# 显示汇总信息
echo ""
echo "=========================================="
echo "           创建参数汇总"
echo "=========================================="
echo "包名: $PKG_NAME"
echo "构建类型: $BUILD_TYPE"
echo "依赖项: $DEPENDENCIES"
echo "节点名: $NODE_NAME"
echo "维护者: $MAINTAINER_NAME <$MAINTAINER_EMAIL>"
echo "描述: $DESCRIPTION"
echo "许可证: $LICENSE"
echo "=========================================="

# 确认创建
read -p "是否创建包？(y/n): " CONFIRM

if [[ $CONFIRM != "y" && $CONFIRM != "Y" ]]; then
    echo "操作已取消。"
    exit 0
fi

# 创建 ROS2 包
echo ""
echo "正在创建 ROS2 包..."
echo "=========================================="

ros2 pkg create $PKG_NAME \
    --build-type $BUILD_TYPE \
    --dependencies $DEPENDENCIES \
    --node-name $NODE_NAME \
    --maintainer-name "$MAINTAINER_NAME" \
    --maintainer-email "$MAINTAINER_EMAIL" \
    --description "$DESCRIPTION" \
    --license "$LICENSE"

# 检查创建是否成功
if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✅ 包创建成功！"
    echo "=========================================="
    echo "包位置: $(pwd)/$PKG_NAME"
    echo ""
    echo "下一步操作:"
    echo "1. cd $PKG_NAME"
    echo "2. 编辑节点代码"
    echo "3. colcon build --packages-select $PKG_NAME"
    echo "4. ros2 run $PKG_NAME $NODE_NAME"
else
    echo ""
    echo "=========================================="
    echo "❌ 包创建失败！"
    echo "=========================================="
    exit 1
fi

cd ..

echo "结束"

