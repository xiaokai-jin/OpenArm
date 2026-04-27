#include "openarm_bimanual_moveit_config/kp_kd_panel.hpp"

#include <iomanip>
#include <sstream>
#include <string>

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"

namespace openarm_bimanual_moveit_config {

namespace {
constexpr std::array<double, 7> kDefaultKp = {400.0, 400.0, 150.0, 350.0, 100.0, 100.0, 100.0};
constexpr std::array<double, 7> kDefaultKd = {4.0, 4.0, 2.0, 3.2, 1.5, 1.5, 1.5};
constexpr std::array<double, 7> kImpedanceKp = {12.5, 12.5, 3.0, 5.0, 2.3, 2.3, 2.3};
constexpr std::array<double, 7> kImpedanceKd = {0.06, 0.06, 0.05, 0.05, 0.03, 0.03, 0.03};

std::string formatDouble(double value, int precision) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}
}  // namespace

KpKdPanel::KpKdPanel(QWidget* parent) : rviz_common::Panel(parent) {
  initializeRosInterfaces();
  buildUi();
  setDefaults();
  refreshValueLabels();
  publishCommands();
}

void KpKdPanel::initializeRosInterfaces() {
  node_ = std::make_shared<rclcpp::Node>("openarm_kp_kd_panel");
  stiffness_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/stiffness_controller/commands", rclcpp::QoS(10));
  damping_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/damping_controller/commands", rclcpp::QoS(10));
}

void KpKdPanel::buildUi() {
  auto* root_layout = new QVBoxLayout();

  auto* hint = new QLabel(
      "Adjust Kp/Kd for one arm; values are mirrored to both arms (left/right).\n"
      "Publish topics: /stiffness_controller/commands and /damping_controller/commands");
  hint->setWordWrap(true);
  root_layout->addWidget(hint);

  auto* kp_group = new QGroupBox("Kp (Joint1~Joint7)");
  auto* kp_layout = new QGridLayout();
  for (int index = 0; index < kJointCount; ++index) {
    auto* name_label = new QLabel(QString("J%1").arg(index + 1));
    auto* slider = new QSlider(Qt::Horizontal);
    slider->setMinimum(kKpMin);
    slider->setMaximum(kKpMax);
    slider->setSingleStep(1);
    slider->setPageStep(10);
    auto* value_label = new QLabel("0.0");
    kp_sliders_.push_back(slider);
    kp_value_labels_.push_back(value_label);
    connect(slider, &QSlider::valueChanged, this, &KpKdPanel::onAnySliderChanged);

    kp_layout->addWidget(name_label, index, 0);
    kp_layout->addWidget(slider, index, 1);
    kp_layout->addWidget(value_label, index, 2);
  }
  kp_group->setLayout(kp_layout);
  root_layout->addWidget(kp_group);

  auto* kd_group = new QGroupBox("Kd (Joint1~Joint7)");
  auto* kd_layout = new QGridLayout();
  for (int index = 0; index < kJointCount; ++index) {
    auto* name_label = new QLabel(QString("J%1").arg(index + 1));
    auto* slider = new QSlider(Qt::Horizontal);
    slider->setMinimum(kKdMin);
    slider->setMaximum(kKdMax);
    slider->setSingleStep(1);
    slider->setPageStep(10);
    auto* value_label = new QLabel("0.00");
    kd_sliders_.push_back(slider);
    kd_value_labels_.push_back(value_label);
    connect(slider, &QSlider::valueChanged, this, &KpKdPanel::onAnySliderChanged);

    kd_layout->addWidget(name_label, index, 0);
    kd_layout->addWidget(slider, index, 1);
    kd_layout->addWidget(value_label, index, 2);
  }
  kd_group->setLayout(kd_layout);
  root_layout->addWidget(kd_group);

  auto* control_layout = new QVBoxLayout();
  auto_publish_checkbox_ = new QCheckBox("Auto Publish");
  auto_publish_checkbox_->setChecked(false);
  publish_button_ = new QPushButton("Publish Once");
  reset_button_ = new QPushButton("Reset Defaults");
  zero_all_button_ = new QPushButton("Zero All");
  impedance_button_ = new QPushButton("Set Impedance");

  connect(publish_button_, &QPushButton::clicked, this, &KpKdPanel::onPublishClicked);
  connect(reset_button_, &QPushButton::clicked, this, &KpKdPanel::onResetClicked);
  connect(zero_all_button_, &QPushButton::clicked, this, &KpKdPanel::onZeroAllClicked);
  connect(impedance_button_, &QPushButton::clicked, this, &KpKdPanel::onImpedanceClicked);

  control_layout->addWidget(auto_publish_checkbox_);
  control_layout->addWidget(publish_button_);
  control_layout->addWidget(reset_button_);
  control_layout->addWidget(zero_all_button_);
  control_layout->addWidget(impedance_button_);
  root_layout->addLayout(control_layout);

  root_layout->addStretch();
  setLayout(root_layout);
}

void KpKdPanel::setDefaults() {
  for (int index = 0; index < kJointCount; ++index) {
    kp_sliders_[index]->setValue(static_cast<int>(kDefaultKp[index] * kKpScale));
    kd_sliders_[index]->setValue(static_cast<int>(kDefaultKd[index] * kKdScale));
  }
}

void KpKdPanel::refreshValueLabels() {
  for (int index = 0; index < kJointCount; ++index) {
    const double kp_value = static_cast<double>(kp_sliders_[index]->value()) / kKpScale;
    const double kd_value = static_cast<double>(kd_sliders_[index]->value()) / kKdScale;
    kp_value_labels_[index]->setText(QString::fromStdString(formatDouble(kp_value, 1)));
    kd_value_labels_[index]->setText(QString::fromStdString(formatDouble(kd_value, 2)));
  }
}

std::array<double, 7> KpKdPanel::readKpValues() const {
  std::array<double, 7> values{};
  for (int index = 0; index < kJointCount; ++index) {
    values[index] = static_cast<double>(kp_sliders_[index]->value()) / kKpScale;
  }
  return values;
}

std::array<double, 7> KpKdPanel::readKdValues() const {
  std::array<double, 7> values{};
  for (int index = 0; index < kJointCount; ++index) {
    values[index] = static_cast<double>(kd_sliders_[index]->value()) / kKdScale;
  }
  return values;
}

void KpKdPanel::publishCommands() {
  auto stiffness_msg = std_msgs::msg::Float64MultiArray();
  auto damping_msg = std_msgs::msg::Float64MultiArray();
  stiffness_msg.data.reserve(14);
  damping_msg.data.reserve(14);

  const auto kp_values = readKpValues();
  const auto kd_values = readKdValues();

  // Left arm 7 joints
  for (double value : kp_values) {
    stiffness_msg.data.push_back(value);
  }
  for (double value : kd_values) {
    damping_msg.data.push_back(value);
  }

  // Right arm 7 joints (mirrored values)
  for (double value : kp_values) {
    stiffness_msg.data.push_back(value);
  }
  for (double value : kd_values) {
    damping_msg.data.push_back(value);
  }

  stiffness_pub_->publish(stiffness_msg);
  damping_pub_->publish(damping_msg);
  rclcpp::spin_some(node_);
}

void KpKdPanel::onAnySliderChanged() {
  refreshValueLabels();
  if (auto_publish_checkbox_ != nullptr && auto_publish_checkbox_->isChecked()) {
    publishCommands();
  }
}

void KpKdPanel::onPublishClicked() {
  publishCommands();
}

void KpKdPanel::onResetClicked() {
  setDefaults();
  refreshValueLabels();
  publishCommands();
}

void KpKdPanel::onZeroAllClicked() {
  for (int index = 0; index < kJointCount; ++index) {
    kp_sliders_[index]->setValue(kKpMin);
    kd_sliders_[index]->setValue(kKdMin);
  }
  refreshValueLabels();
  publishCommands();
}

void KpKdPanel::onImpedanceClicked() {
  for (int index = 0; index < kJointCount; ++index) {
    kp_sliders_[index]->setValue(static_cast<int>(kImpedanceKp[index] * kKpScale));
    kd_sliders_[index]->setValue(static_cast<int>(kImpedanceKd[index] * kKdScale));
  }
  refreshValueLabels();
  publishCommands();
}

}  // namespace openarm_bimanual_moveit_config

PLUGINLIB_EXPORT_CLASS(openarm_bimanual_moveit_config::KpKdPanel, rviz_common::Panel)
