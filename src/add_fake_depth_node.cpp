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

#include <bbox2d_to_3d_node/add_fake_depth_node.hpp>

namespace bbox2d_to_3d_node {

AddFakeDepthNode::AddFakeDepthNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("add_fake_depth_node", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
{
    this->declare_parameter("base_frame_id", "camera_link");
    this->declare_parameter("broadcast_tf", true);

    this->get_parameter("base_frame_id", this->base_frame_id_);
    this->get_parameter("broadcast_tf", this->broadcast_tf_);

    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    this->bbox2d_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "bbox2d", 1, std::bind(&AddFakeDepthNode::bbox2dCallback, this, std::placeholders::_1));

    this->bbox3d_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("bbox3d", 1);
}

void AddFakeDepthNode::bbox2dCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr & bbox2d_msg)
{
    vision_msgs::msg::Detection3DArray bbox3d_msg;
    bbox3d_msg.header = bbox2d_msg->header;

    float fx = 635.8640747070312;
    float fy = 635.1402587890625;
    float cx = 628.0241088867188;
    float cy = 373.9369201660156;
    for (auto bbox2d : bbox2d_msg->detections)
    {

        vision_msgs::msg::Detection3D bbox3d;
        bbox3d.header = bbox2d.header;
        bbox3d.results = bbox2d.results;

        float position_x = bbox2d.bbox.center.position.x;
        float position_y = bbox2d.bbox.center.position.y;
        float bbox_size_x = bbox2d.bbox.size_x;
        float bbox_size_y = bbox2d.bbox.size_y;

        bbox3d.bbox.center.position.x = (position_x - cx) * this->depth_ / fx;
        bbox3d.bbox.center.position.y = this->depth_;
        bbox3d.bbox.center.position.z = (position_y - cy) * this->depth_ / fy;
        bbox3d.bbox.size.x = bbox_size_x * this->depth_ / fy;
        bbox3d.bbox.size.y = 0.0;
        bbox3d.bbox.size.z = bbox_size_y * this->depth_ / fy;

        bbox3d_msg.detections.push_back(bbox3d);

        if (this->broadcast_tf_)
        {
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = bbox2d.header.stamp;
            transform.header.frame_id = this->base_frame_id_;
            transform.child_frame_id = bbox2d.id;
            transform.transform.translation.x = bbox3d.bbox.center.position.x;
            transform.transform.translation.y = bbox3d.bbox.center.position.y;
            transform.transform.translation.z = bbox3d.bbox.center.position.z;
            transform.transform.rotation.x = 0;
            transform.transform.rotation.y = 0;
            transform.transform.rotation.z = 0;
            transform.transform.rotation.w = 1;
            this->tf_broadcaster_->sendTransform(transform);
        }
    }

    this->bbox3d_pub_->publish(bbox3d_msg);
}

} // namespace bbox2d_to_3d_node

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(bbox2d_to_3d_node::AddFakeDepthNode)