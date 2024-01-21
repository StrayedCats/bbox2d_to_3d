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

#include <bbox2d_to_3d_node/bbox2d_to_3d_node.hpp>

namespace bbox2d_to_3d_node {

BBox2DTo3DNode::BBox2DTo3DNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("bbox2d_to_3d_node", options),
    sync_(10)
{
    this->camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 1, std::bind(&BBox2DTo3DNode::cameraInfoCallback, this, std::placeholders::_1));

    this->depth_sub_.subscribe(this, "depth", rmw_qos_profile_sensor_data);
    this->bbox2d_sub_.subscribe(this, "bbox2d", rmw_qos_profile_sensor_data);
    this->sync_.connectInput(this->depth_sub_, this->bbox2d_sub_);
    this->sync_.registerCallback(&BBox2DTo3DNode::callback, this);
    this->bbox3d_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("bbox3d", 1);
}

void BBox2DTo3DNode::callback(const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
                              const vision_msgs::msg::Detection2DArray::ConstSharedPtr & bbox2d_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);
    }
    catch (cv_bridge::Exception & e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    vision_msgs::msg::Detection3DArray bbox3d_msg;
    bbox3d_msg.header = depth_msg->header;
    for (auto bbox2d : bbox2d_msg->detections)
    {
        uint16_t depth_mm = cv_ptr->image.at<uint16_t>(bbox2d.bbox.center.position.y, bbox2d.bbox.center.position.x);
        float depth_m = depth_mm / 1000.0;
        if (std::isnan(depth_mm))
        {
            RCLCPP_WARN(this->get_logger(), "depth is nan");
            continue;
        }
        RCLCPP_INFO(this->get_logger(), "depth: %d", depth_mm);

        vision_msgs::msg::Detection3D bbox3d;
        bbox3d.header = depth_msg->header;
        bbox3d.results = bbox2d.results;

        float position_x = bbox2d.bbox.center.position.x;
        float position_y = bbox2d.bbox.center.position.y;
        float bbox_size_x = bbox2d.bbox.center.position.x;
        float bbox_size_y = bbox2d.bbox.center.position.y;
        RCLCPP_INFO(this->get_logger(), "bbox2d center: (%f, %f), size: (%f, %f)",
                    position_x, position_y, bbox_size_x, bbox_size_y);

        bbox3d.bbox.center.position.x = (position_x - this->cx_) * depth_m / this->fx_;
        bbox3d.bbox.center.position.y = (position_y - this->cy_) * depth_m / this->fy_;
        bbox3d.bbox.center.position.z = depth_m;
        bbox3d.bbox.size.x = bbox_size_x * depth_m / this->fy_;
        bbox3d.bbox.size.y = bbox_size_y * depth_m / this->fy_;
        bbox3d.bbox.size.z = 0.0;

        // print
        RCLCPP_INFO(this->get_logger(), "bbox3d center: (%f, %f, %f), size: (%f, %f, %f)",
                    bbox3d.bbox.center.position.x, bbox3d.bbox.center.position.y, bbox3d.bbox.center.position.z,
                    bbox3d.bbox.size.x, bbox3d.bbox.size.y, bbox3d.bbox.size.z);

        bbox3d_msg.detections.push_back(bbox3d);
    }

    this->bbox3d_pub_->publish(bbox3d_msg);
}

void BBox2DTo3DNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
    // stop subscribing camera_info
    this->camera_info_sub_.reset();
    this->fx_ = camera_info_msg->k[0];
    this->fy_ = camera_info_msg->k[4];
    this->cx_ = camera_info_msg->k[2];
    this->cy_ = camera_info_msg->k[5];

    RCLCPP_INFO(this->get_logger(), "camera_info: fx: %f, fy: %f, cx: %f, cy: %f",
                this->fx_, this->fy_, this->cx_, this->cy_);
}

} // namespace bbox2d_to_3d_node

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(bbox2d_to_3d_node::BBox2DTo3DNode)