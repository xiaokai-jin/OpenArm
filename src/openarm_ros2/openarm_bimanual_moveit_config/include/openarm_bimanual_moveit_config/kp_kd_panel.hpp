#pragma once

#include <array>
#include <memory>
#include <vector>

#include <QCheckBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QWidget>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace openarm_bimanual_moveit_config {

class KpKdPanel : public rviz_common::Panel {
 public:
  explicit KpKdPanel(QWidget* parent = nullptr);

 private:
  void onAnySliderChanged();
  void onPublishClicked();
  void onResetClicked();
  void onZeroAllClicked();

  void initializeRosInterfaces();
  void buildUi();
  void setDefaults();
  void refreshValueLabels();
  void publishCommands();
  std::array<double, 7> readKpValues() const;
  std::array<double, 7> readKdValues() const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr stiffness_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr damping_pub_;

  std::vector<QSlider*> kp_sliders_;
  std::vector<QSlider*> kd_sliders_;
  std::vector<QLabel*> kp_value_labels_;
  std::vector<QLabel*> kd_value_labels_;

  QCheckBox* auto_publish_checkbox_{nullptr};
  QPushButton* publish_button_{nullptr};
  QPushButton* reset_button_{nullptr};
  QPushButton* zero_all_button_{nullptr};

  static constexpr int kJointCount = 7;
  static constexpr int kKpScale = 10;     // 0.1 granularity
  static constexpr int kKdScale = 100;    // 0.01 granularity
  static constexpr int kKpMin = 0;
  static constexpr int kKpMax = 5000;     // 0 ~ 500.0
  static constexpr int kKdMin = 0;
  static constexpr int kKdMax = 500;     // 0 ~ 5.0
};

}  // namespace openarm_bimanual_moveit_config
