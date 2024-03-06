# bbox2d_to_3d

<img width="682" alt="スクリーンショット 2024-02-12 12 22 29" src="https://github.com/StrayedCats/bbox2d_to_3d/assets/67567093/a5e1c559-ef86-430d-9e1a-c2d5f65ea56f">

## Example usage
```bash
# YOLOX-TRT
ros2 run detector2d_node detector2d_node_exec --ros-args -p load_target_plugin:=detector2d_plugins::YoloxTrt  -p yolox_trt_plugin.model_path:=/home/core2024/yolox_tiny.trt -p yolox_trt_plugin.imshow_isshow:=false -r image_raw:=/color/image_raw

# ByteTrack
ros2 run bytetrack_cpp_node bytetrack_cpp_node

# bbox2d-to-3d
ros2 run bbox2d_to_3d_node bbox2d_to_3d_node_exec --ros-args -r camera_info:=color/camera_info -r color:=/color/image_raw -r depth:=/aligned_depth_to_color/image_raw -r bbox2d:=/bytetrack/bounding_boxes

# bytrack-viewr
ros2 run bytetrack_viewer bytetrack_viewer --ros-args -r image_raw:=/color/image_raw
```
