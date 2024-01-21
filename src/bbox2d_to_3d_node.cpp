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

BBox2DTo3DNode::BBox2DTo3DNode(const rclcpp::NodeOptions & options) :
    Node("bbox2d_to_3d_node", options),
    sync_(1)
{
    this->depth_sub_.subscribe(this, "depth", rmw_qos_profile_sensor_data);
    this->bbox2d_sub_.subscribe(this, "bbox2d", rmw_qos_profile_sensor_data);
    this->sync_.connectInput(this->depth_sub_, this->bbox2d_sub_);
    this->sync_.registerCallback(&BBox2DTo3DNode::callback, this);
    this->bbox3d_pub_ = this->create_publisher<vision_msgs::msg::BoundingBox3DArray>("bbox3d", 10);
}

void BBox2DTo3DNode::callback(const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
                              const vision_msgs::msg::Detection2DArray::ConstSharedPtr & bbox2d_msg)
{
// TDDO: Implement this function
}

} // namespace bbox2d_to_3d_node

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(bbox2d_to_3d_node::BBox2DTo3DNode)