#!/bin/bash
# 开发同步脚本（作者自用，路径需按自己的环境改）
#
# 背景：这个项目的固件源码在 stm32/Arm/ 里开发（用 Keil 编译），文档在另一个目录。
# 开源仓库只是"发布镜像"，不参与日常开发。改完代码跑一下这个脚本把改动同步过来，
# 再用 git 提交。
#
# 为什么不直接在仓库里开发：
#   1. Keil 工程的路径、CubeMX 的生成目录都不方便搬
#   2. 仓库里不能有编译产物（30MB+），开发目录里必须有
#   3. 用同步脚本可以明确控制"哪些文件对外可见"

set -e

SRC_ROOT="E:/a2_QianRuShi"
REPO="$(cd "$(dirname "$0")/.." && pwd)"

echo "源目录：$SRC_ROOT"
echo "仓库：  $REPO"
echo ""

# ---------- 1. 固件源码 ----------
rm -rf "$REPO/firmware/User" "$REPO/firmware/Core"
cp -r "$SRC_ROOT/stm32/Arm/User"  "$REPO/firmware/"
cp -r "$SRC_ROOT/stm32/Arm/Core"  "$REPO/firmware/"
cp    "$SRC_ROOT/stm32/Arm/Arm.ioc" "$REPO/firmware/"
cp    "$SRC_ROOT/stm32/Arm/MDK-ARM/Arm.uvprojx" "$REPO/firmware/MDK-ARM/" 2>/dev/null || true
echo "✓ 固件已同步"

# ---------- 2. 设计文档 01~09（10、11 是本仓库原创，不在同步范围） ----------
for f in "$SRC_ROOT/立柱码垛机械臂项目/软件设计/"0*.md; do
    [ -e "$f" ] || continue
    cp "$f" "$REPO/docs/"
done
echo "✓ 文档已同步"

# ---------- 3. 上位机与标定工具 ----------
cp "$SRC_ROOT/stm32/tools/arm_teacher.py"  "$REPO/tools/" 2>/dev/null || true
cp "$SRC_ROOT/stm32/tools/servo_calib.py"  "$REPO/tools/" 2>/dev/null || true
cp "$SRC_ROOT/stm32/tools/demo_seq.txt"    "$REPO/tools/" 2>/dev/null || true
echo "✓ 工具已同步"

# ---------- 4. 硬件接线核对表 ----------
cp "$SRC_ROOT/立柱码垛机械臂项目/接线核对表-机械臂整套.md" "$REPO/hardware/接线核对表.md" 2>/dev/null || true
echo "✓ 硬件文档已同步"

# ---------- 5. 显示变更，交给人工审 ----------
echo ""
cd "$REPO"
if [ -d .git ]; then
    echo "=== 改动列表（确认无误后自行 git commit） ==="
    git status --short
    echo ""
    echo "提示：docs/10 和 docs/11 是本仓库原创内容，不在同步范围，不会被覆盖。"
    echo "     assets/ 里的图片也需要人工维护。"
else
    echo "（仓库还没 git init）"
fi
