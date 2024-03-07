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

#include <bbox2d_to_3d_node/bbox2d_to_3d_quick_node.hpp>

namespace bbox2d_to_3d_node {

BBox2DTo3DQuickNode::BBox2DTo3DQuickNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("bbox2d_to_3d_node", options),
    // sync_(10),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
{
    this->declare_parameter("base_frame_id", "camera_link");
    this->declare_parameter("min_depth", 0.05);
    this->declare_parameter("max_depth", 3.0);
    this->declare_parameter("imshow_isshow", true);
    this->declare_parameter("broadcast_tf", true);

    this->get_parameter("base_frame_id", this->base_frame_id_);
    this->get_parameter("min_depth", this->min_depth_);
    this->get_parameter("max_depth", this->max_depth_);
    this->get_parameter("imshow_isshow", this->imshow_isshow_);
    this->get_parameter("broadcast_tf", this->broadcast_tf_);


    this->camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 1, std::bind(&BBox2DTo3DQuickNode::cameraInfoCallback, this, std::placeholders::_1));

    this->color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "color", 1, [this](const sensor_msgs::msg::Image::ConstSharedPtr & color_msg) {
            try
            {
                this->color_ = cv_bridge::toCvCopy(color_msg, color_msg->encoding)->image;
                cv::cvtColor(this->color_, this->color_, cv::COLOR_BGR2RGB);
            }
            catch (cv_bridge::Exception & e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
        });

    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    this->depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "depth", 1, std::bind(&BBox2DTo3DQuickNode::depthCallback, this, std::placeholders::_1));
    this->bbox2d_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "bbox2d", 1, std::bind(&BBox2DTo3DQuickNode::bbox2dCallback, this, std::placeholders::_1));

    this->bbox3d_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>("bbox3d", 1);
}

cv::Vec3b BBox2DTo3DQuickNode::depth2hue(float depth)
{
    if (depth < this->min_depth_)
    {
        return cv::Vec3b(0, 0, 0);
    }
    else if (depth > this->max_depth_)
    {
        return cv::Vec3b(0, 0, 0);
    }

    float hue = (depth - this->min_depth_) / (this->max_depth_ - this->min_depth_) * 180;
    cv::Mat3b hsv(1, 1, cv::Vec3b(hue, 255, 255));
    cv::Mat3b bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    return bgr.at<cv::Vec3b>(0, 0);
}

void BBox2DTo3DQuickNode::bbox2dCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr & bbox2d_msg)
{
    this->bbox2d_msg_ = bbox2d_msg;
}

void BBox2DTo3DQuickNode::depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
{
    // get time
    auto time = this->get_clock()->now();
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

    if (this->bbox2d_msg_ == nullptr)
    {
        RCLCPP_WARN(this->get_logger(), "bbox2d_msg is null");
        return;
    }

    cv::Mat3b color = this->color_.clone();

    vision_msgs::msg::Detection3DArray bbox3d_msg;
    bbox3d_msg.header = depth_msg->header;
    // for (auto bbox2d : bbox2d_msg->detections)
    for (auto bbox2d : this->bbox2d_msg_->detections)
    {
        uint16_t depth_mm = cv_ptr->image.at<uint16_t>(bbox2d.bbox.center.position.y, bbox2d.bbox.center.position.x);
        float depth_m = depth_mm / 1000.0;
        if (std::isnan(depth_mm))
        {
            RCLCPP_WARN(this->get_logger(), "depth is nan");
            continue;
        } else if (depth_m < this->min_depth_ || depth_m > this->max_depth_)
        {
            RCLCPP_WARN(this->get_logger(), "depth is out of range: %f", depth_m);
            continue;
        }

        vision_msgs::msg::Detection3D bbox3d;
        bbox3d.header = depth_msg->header;
        bbox3d.results = bbox2d.results;

        float position_x = bbox2d.bbox.center.position.x;
        float position_y = bbox2d.bbox.center.position.y;
        float bbox_size_x = bbox2d.bbox.size_x;
        float bbox_size_y = bbox2d.bbox.size_y;

        bbox3d.bbox.center.position.x = (position_x - this->cx_) * depth_m / this->fx_;
        bbox3d.bbox.center.position.y = depth_m;
        bbox3d.bbox.center.position.z = (position_y - this->cy_) * depth_m / this->fy_;
        bbox3d.bbox.size.x = bbox_size_x * depth_m / this->fy_;
        bbox3d.bbox.size.y = 0.0;
        bbox3d.bbox.size.z = bbox_size_y * depth_m / this->fy_;

        bbox3d_msg.detections.push_back(bbox3d);

        if (this->broadcast_tf_)
        {
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = depth_msg->header.stamp;
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

        if (!color.empty() && this->imshow_isshow_)
        {
            int left = position_x - (bbox_size_x / 2);
            int top = position_y - (bbox_size_y / 2);
            int right = position_x + (bbox_size_x / 2);
            int bottom = position_y + (bbox_size_y / 2);

            for (int y = top; y < bottom; y++)
                for (int x = left; x < right; x++)
                    color.at<cv::Vec3b>(y, x) = depth2hue(cv_ptr->image.at<uint16_t>(y, x) / 1000.0);

            cv::circle(color, cv::Point(position_x, position_y), 5, cv::Scalar(0, 0, 255), -1);
            std::stringstream ss;
            ss << depth_m << "m";
            cv::putText(color, ss.str(), cv::Point(position_x, position_y), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);


            cv::rectangle(color, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 255, 0), 2);

        }
    }

    if (!color.empty() && this->imshow_isshow_)
    {
        cv::imshow("color", color);
        cv::waitKey(1);
    }

    this->bbox3d_pub_->publish(bbox3d_msg);

    auto end = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "callback: %f ms", (end - time).seconds() * 1000);
}

void BBox2DTo3DQuickNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
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
RCLCPP_COMPONENTS_REGISTER_NODE(bbox2d_to_3d_node::BBox2DTo3DQuickNode)