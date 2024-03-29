// Copyright 2024 StrayedCats.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/opencv.hpp>

namespace bbox2d_to_3d_node
{
    typedef sensor_msgs::msg::Image Image;
    typedef vision_msgs::msg::Detection2DArray Detection2DArray;
    class BBox2DTo3DQuickNode : public rclcpp::Node
    {
    public:
        BBox2DTo3DQuickNode(const rclcpp::NodeOptions &);

    private:
        rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr bbox2d_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        rclcpp::Subscription<Image>::SharedPtr color_sub_;
        rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr bbox3d_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

        cv::Mat1f depth_;
        cv::Mat3b color_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        float fx_;
        float fy_;
        float cx_;
        float cy_;

        float min_depth_;
        float max_depth_;
        bool imshow_isshow_;
        bool broadcast_tf_;
        std::string base_frame_id_;

        Detection2DArray::ConstSharedPtr bbox2d_msg_;

        void bbox2dCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr &);
        void depthCallback(const Image::ConstSharedPtr &);

        void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &);
        void colorCallback(const Image::ConstSharedPtr &);


        cv::Vec3b depth2hue(float);
    };
}
