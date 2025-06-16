#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

#include <qobjectdefs.h>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>

#include <QCheckBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

namespace hexapod_rviz_plugins {
class Joystick : public QWidget {
  Q_OBJECT
public:
  explicit Joystick(QWidget *parent = nullptr, int radius = 17,
                    int padding = 20);
  void setInteractive(bool interactive);

Q_SIGNALS:
  void joystickMoved(const float axis_x, const float axis_y);

public:
  double radius();
  double padding();
  void setValue(float x, float y);

protected:
  void paintEvent(QPaintEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;

private:
  bool dragging_ = false;
  bool is_interactive_ = false;
  int padding_ = 40;
  int radius_ = 17;
  int center_x_;
  int center_y_;
  float axis_x = 0;
  float axis_y = 0;
};

class ControllerWidget : public QWidget {
  Q_OBJECT
private:
  Joystick *joystick_left_;
  Joystick *joystick_right_;
  bool interaction_enabled_;
  float axes_values_[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

Q_SIGNALS:
  void controllerStateChanged(const float axes_values[8]);

public:
  void setValue(std::vector<float> axes);
  void setInteractionEnabled(bool enable);
  void setupUi();
  ControllerWidget();
};
} // namespace hexapod_rviz_plugins

#endif
