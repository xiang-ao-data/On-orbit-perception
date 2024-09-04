# 说明

本项目是使用一个rslidar-m1和一个奥比中光的tof相机fetmo-mega进行多传感器融合感知，包括三个部分，一个基于多传感器融合的三维点云重构，一个是基于yolov8的目标检测，一个是基于deeplabv3+的语义分割。

## 目录结构

```
├── 3D_reconstruction
│   ├── data_fusion # 获取深度相机和雷达点云比融合
    |—— 3D_reconstruction.py # 多帧点云配准融合并重构
├── object_detection
│   ├── yolov8 # 目标检测
    |—— yolov8_inference.py # yolov8目标检测
├── semantic_segmentation
│   ├── deeplabv3+ # 语义分割
    |—— deeplabv3_inference.py # deeplabv3+语义分割
├── README.md # 说明文档
├── requirements.txt # 依赖包
├── test_data # 测试数据集
│   ├── 20211111 # 测试数据集1