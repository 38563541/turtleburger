# Lane Follower with Fuzzy Control

## 📦 目錄結構
- `/scripts`: 所有 ROS nodes 程式碼
- `/video`: 成果影片
- `/fuzzy_config`: 模糊邏輯設定與圖
- `README.md`: 執行說明與設計描述

## 🚀 執行方式
請確認已安裝 ROS，進入 `catkin_ws` 後：
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch lane_follower lane_follower.launch
```

## 🤖 使用 ROS Nodes
- camera_node：發佈 /camera/image_raw
- lane_detection_node：偵測車道並發佈 /lane/offset 與 /lane/image_annotated
- fuzzy_control_node：根據 /lane/offset 控制方向
- (可選) sign_recognition_node：識別交通標誌

## ⚙️ 環境需求
- Ubuntu 20.04 + ROS Noetic
- Python 3 / OpenCV / cv_bridge / rospy

## 🔍 模糊邏輯設計
見 /fuzzy_config 資料夾內文件與圖示。
