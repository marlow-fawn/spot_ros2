// Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

#include <spot_driver_cpp/interfaces/rclcpp_tf_interface.hpp>

namespace spot_ros2 {
RclcppTfInterface::RclcppTfInterface(const std::shared_ptr<rclcpp::Node>& node) : static_tf_broadcaster_{node} {}

tl::expected<void, std::string> RclcppTfInterface::updateStaticTransforms(
    const std::vector<geometry_msgs::msg::TransformStamped>& transforms) {
  bool has_new_frame = false;
  for (const auto& transform : transforms) {
    // If one of the transforms is to a new child frame, flag that a new transform needs to be published and add the
    // child frame to the set of currently-published frames.
    if (current_static_child_frames_.count(transform.child_frame_id) == 0) {
      has_new_frame = true;
      current_static_child_frames_.insert(current_static_child_frames_.end(), transform.child_frame_id);
    }
  }

  // Only publish if there is a new transform.
  // Note that unlike in Python, the rclcpp StaticTransformPublisher will correctly re-publish all previous transforms.
  if (has_new_frame) {
    static_tf_broadcaster_.sendTransform(transforms);
  }
  return {};
}
}  // namespace spot_ros2
