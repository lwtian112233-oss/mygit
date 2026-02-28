# Lane Detection (C++ / OpenCV)

简单的车道线识别示例，使用 OpenCV 的 Canny 边缘检测 + Hough 直线变换，并对左右车道线做平均合成。

快速开始：

1. 安装依赖（Ubuntu 示例）:

```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev
```

2. 构建:

```bash
mkdir -p build && cd build
cmake ..
make -j
```

3. 运行:

```bash
# 对图片
./lane_detector ../test.jpg

# 对视频
./lane_detector ../video.mp4

# 打开摄像头
./lane_detector 0
```

文件说明:
- `CMakeLists.txt`：构建脚本，依赖 OpenCV。
- `src/main.cpp`：程序入口，支持图片/视频/摄像头输入。
- `src/lane_detector.h` / `src/lane_detector.cpp`：检测逻辑（Canny -> ROI -> Hough -> 平均线并绘制）。

说明与限制：
- 这是一个教学示例，未做相机矫正、复杂拟合或稳定跟踪。适合用作起点用于改进（如透视变换、聚类、卡尔曼滤波等）。

### 新增：自动测试与增强检测说明

本仓库已新增增强功能与测试工具：

- 透视变换（bird's-eye）用于改善车道线拟合和可视化。
- 使用二次多项式拟合（x = a*y^2 + b*y + c）来表示车道曲线，并对左右车道分别做简单的指数平滑（单图像演示仍可使用先前值）。
- 新增测试图像生成脚本：`tools/generate_test_image.py`，用于快速生成合成车道图像进行验证。

快速验证命令：

```bash
python3 tools/generate_test_image.py test.jpg
mkdir -p build && cd build && cmake .. && make -j
./lane_detector ../test.jpg
```

无显示环境验证（headless）：

```bash
mkdir -p build && cd build && cmake .. && make -j headless_runner
./headless_runner ../test.jpg ../test_out.jpg
```

新的 headless 参数：

- `--config <file.yml>`：从 YAML 文件加载检测参数（参考下面的默认字段名）。
- `--offset-file <out.json>`：把偏差写入 JSON 文件，格式 `{ "offset": <value|null> }`。
- `--print-offset`：在 stdout 打印 `offset=<value>`。

新增 CLI：

- `--lane-width <meters>`：手动指定车道实际宽度（米），用于从像素宽度估算像素/米（若未设置 `pixels_per_meter`）。
- `--pixels-per-meter <value>`：直接指定像素到米的比例（px/m），用于把偏差转换为米。

输出 JSON 现在包含：

```json
{
	"offset_px": -123.4,
	"offset_m": -1.23
}
```

示例配置（YAML，可直接用 OpenCV `FileStorage` 读取）：

```yaml
src_top_left_x: 0.45
src_top_right_x: 0.55
src_top_y: 0.62
src_bottom_left_x: 0.05
src_bottom_right_x: 0.95
dst_left_x: 0.2
dst_right_x: 0.8
smoothing_alpha: 0.2
canny_low: 50
canny_high: 150
hough_threshold: 50
hough_min_line_len: 30
hough_max_line_gap: 10
```

示例：

```bash
python3 tools/generate_test_image.py test.jpg
mkdir -p build && cd build && cmake .. && make -j headless_runner
./headless_runner ../test.jpg ../test_out.jpg --config ../example_config.yml --offset-file ../offset.json --print-offset
```


处理完成后，输出文件为 `test_out.jpg`。

代码和注释主要变更位置：`src/lane_detector.h` 与 `src/lane_detector.cpp`。建议在真实应用中补充：

- 相机标定（去畸变）和更精细的透视区域选择。
- 将拟合结果从 bird's-eye 空间映射回原始图像并对多帧间进行更稳健的跟踪（例如卡尔曼滤波或滑动窗口检测）。
