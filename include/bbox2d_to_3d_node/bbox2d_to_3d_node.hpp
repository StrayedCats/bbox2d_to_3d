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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <vision_msgs/msg/bounding_box3_d_array.hpp>

#include <opencv2/opencv.hpp>

namespace bbox2d_to_3d_node
{
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, vision_msgs::msg::Detection2DArray>
        SyncPolicy;
    typedef sensor_msgs::msg::Image Image;
    typedef vision_msgs::msg::Detection2DArray Detection2DArray;
    class BBox2DTo3DNode : public rclcpp::Node
    {
    public:
        BBox2DTo3DNode(const rclcpp::NodeOptions &);

    private:
        message_filters::Synchronizer<SyncPolicy> sync_;
        message_filters::Subscriber<Image> depth_sub_;
        message_filters::Subscriber<Detection2DArray> bbox2d_sub_;

        rclcpp::Publisher<vision_msgs::msg::BoundingBox3DArray>::SharedPtr bbox3d_pub_;

        cv::Mat1f depth_;

        void callback(const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
                      const vision_msgs::msg::Detection2DArray::ConstSharedPtr &bbox2d_msg);
    };
}
