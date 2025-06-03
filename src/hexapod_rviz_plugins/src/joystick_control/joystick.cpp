#include "joystick_control/rviz_panel.hpp"
#include <QMouseEvent>
#include <QtDebug>
#include <cstdint>
#include <joystick_control/joystick.hpp>
#include <qchar.h>
#include <qcolor.h>
#include <qglobal.h>
#include <qobjectdefs.h>
#include <qpainter.h>
#include <qregion.h>
#include <qrgb.h>
#include <qwidget.h>
#include <rclcpp/logging.hpp>
#include <string>

namespace hexapod_rviz_plugins {
Joystick::Joystick(QWidget *parent, int radius, int padding) : QWidget(parent) {
  setGeometry(0, 0, 40, 40);
  setInteractive(true);
  setMinimumSize(200, 100);
  setMouseTracking(true);
  radius_ = radius;
  padding_ = padding;
  center_x_ = radius_ + padding_;
  center_y_ = radius_ + padding_;
}

double Joystick::radius() { return radius_; };
double Joystick::padding() { return padding_; };

void Joystick::paintEvent(QPaintEvent *) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // Draw left joystick (outer circle)
  painter.setBrush(QColor::fromRgb(qRgb(40, 45, 46)));
  painter.drawEllipse(radius(), radius(), padding() * 2, padding() * 2);

  // Draw left stick (inner circle) using image
  float x = axis_x * radius();
  float y = axis_y * radius();
  QPixmap stick_pixmap(":/assets/stick.png");

  if (!stick_pixmap.isNull()) {
    painter.drawPixmap(center_x_ + x - radius(), center_y_ - y - radius(),
                       radius() * 2, radius() * 2, stick_pixmap);

  } else {
    // Fallback if image fails to load
    painter.setBrush(QColor::fromRgb(qRgb(200, 50, 50))); // red fallback
    painter.drawEllipse(QPointF(center_x_ + x, center_y_ - y), radius_,
                        radius_);
  }
}

void Joystick::mousePressEvent(QMouseEvent *event) {
  if (!is_interactive_)
    return; // Disable dragging in Subscriber Mode
  QPointF pos = event->pos();
  // Check if click is within left joystick
  if (QPointF(pos - QPointF(center_x_, center_y_)).manhattanLength() <
      padding_ * 2 - radius_ / 2.0) {
    dragging_ = true;
  }
}

void Joystick::mouseMoveEvent(QMouseEvent *event) {
  if (!is_interactive_)
    return; // Disable dragging in Subscriber Mode
  if (dragging_) {
    QPointF pos = event->pos();
    // Calculate new axes values for left stick
    float dx = (pos.x() - center_x_) / 20.0f; // Invert X-axis
    float dy = (center_y_ - pos.y()) / 20.0f;
    // Clamp to unit circle
    float mag = std::sqrt(dx * dx + dy * dy);
    if (mag > 1.0f) {
      dx /= mag;
      dy /= mag;
    }
    axis_x = dx;
    axis_y = dy;
    Q_EMIT joystickMoved(axis_x, axis_y);
  }
  update(); // Trigger repaint
}

void Joystick::mouseReleaseEvent(QMouseEvent *) {
  if (!is_interactive_)
    return; // Disable dragging in Subscriber Mode
  dragging_ = false;
  // Reset sticks to center in Publisher Mode
  axis_x = 0.0f;
  axis_y = 0.0f;
  update(); // Trigger repaint
  Q_EMIT joystickMoved(axis_x, axis_y);
}

void Joystick::setInteractive(bool interactive) {
  is_interactive_ = interactive;
}

void ControllerWidget::setInteractionEnabled(bool enable) {
  interaction_enabled_ = enable;
  joystick_left_->setInteractive(enable);
  joystick_right_->setInteractive(enable);
}

void ControllerWidget::setupUi() {
  // Controller background image
  QLabel *background_image = new QLabel(this);
  QPixmap original(":/assets/controller.png");
  background_image->setPixmap(original);
  background_image->setScaledContents(true); // Resize to fit
  background_image->setSizePolicy(QSizePolicy::Expanding,
                                  QSizePolicy::Expanding);
  background_image->setGeometry(0, 0, width(), height());

  double mid_x = width() / 2.0 - 39;

  joystick_left_ = new Joystick(this);
  joystick_left_->setStyleSheet("background: transparent;");
  joystick_left_->move(mid_x - 33, 80 - joystick_left_->radius());

  // joystick_left_->setSize(20, 20, 20);

  // Joystick visualization widget (overlaid)
  joystick_right_ = new Joystick(this);
  joystick_right_->setStyleSheet("background: transparent;");
  joystick_right_->move(mid_x + 33, 80 - joystick_right_->radius());

  // Connect joystick movement to a lambda that emits your custom signal
  connect(joystick_left_, &Joystick::joystickMoved, this,
          [this](const double axis_x, const double axis_y) {
            axes_values_[0] = axis_x;
            axes_values_[1] = axis_y;
            Q_EMIT controllerStateChanged(axes_values_);
          });
  connect(joystick_right_, &Joystick::joystickMoved, this,
          [this](const double axis_x, const double axis_y) {
            axes_values_[3] = axis_x;
            axes_values_[4] = axis_y;
            Q_EMIT controllerStateChanged(axes_values_);
          });

  double right_pad_mid_x = width() - 63;
  double right_pad_mid_y = 50;
  double button_radius = 20;
  QPushButton *a_button = new QPushButton("A", this);
  QPushButton *y_button = new QPushButton("Y", this);
  QPushButton *x_button = new QPushButton("X", this);
  QPushButton *b_button = new QPushButton("B", this);

  a_button->setStyleSheet("QPushButton {"
                          "  background-color: #4CAF50;" // Green
                          "  color: #C8FACC;"            // Light green text
                          "  border: 2px solid #2E7D32;" // Darker green border
                          "  border-radius: 10px;"
                          "  font-size: 8pt;"
                          "  font-weight: bold;"
                          "}"
                          "QPushButton:hover {"
                          "  background-color: #81C784;" // Lighter green
                          "}"
                          "QPushButton:pressed {"
                          "  background-color: #388E3C;" // Darker pressed
                          "}");

  b_button->setStyleSheet("QPushButton {"
                          "  background-color: #F44336;" // Red
                          "  color: #FFCDD2;"            // Light red text
                          "  border: 2px solid #B71C1C;" // Dark red border
                          "  border-radius: 10px;"
                          "  font-size: 8pt;"
                          "  font-weight: bold;"
                          "}"
                          "QPushButton:hover {"
                          "  background-color: #EF9A9A;" // Lighter red
                          "}"
                          "QPushButton:pressed {"
                          "  background-color: #C62828;" // Darker red
                          "}");

  x_button->setStyleSheet("QPushButton {"
                          "  background-color: #2196F3;" // Blue
                          "  color: #BBDEFB;"            // Light blue text
                          "  border: 2px solid #0D47A1;" // Dark blue border
                          "  border-radius: 10px;"
                          "  font-size: 8pt;"
                          "  font-weight: bold;"
                          "}"
                          "QPushButton:hover {"
                          "  background-color: #64B5F6;" // Lighter blue
                          "}"
                          "QPushButton:pressed {"
                          "  background-color: #1565C0;" // Darker blue
                          "}");

  y_button->setStyleSheet("QPushButton {"
                          "  background-color: #FFEB3B;" // Yellow
                          "  color: #F57F17;"            // Pale yellow text
                          "  border: 2px solid #F57F17;" // Dark yellow border
                          "  border-radius: 10px;"
                          "  font-size: 8pt;"
                          "  font-weight: bold;"
                          "}"
                          "QPushButton:hover {"
                          "  background-color: #FFF176;" // Lighter yellow
                          "}"
                          "QPushButton:pressed {"
                          "  background-color: #FBC02D;" // Darker yellow
                          "}");

  y_button->setGeometry(right_pad_mid_x, right_pad_mid_y - 17, button_radius,
                        button_radius);
  b_button->setGeometry(right_pad_mid_x + 17, right_pad_mid_y, button_radius,
                        button_radius);
  x_button->setGeometry(right_pad_mid_x - 17, right_pad_mid_y, button_radius,
                        button_radius);
  a_button->setGeometry(right_pad_mid_x, right_pad_mid_y + 17, button_radius,
                        button_radius);
}
void ControllerWidget::setControllerState(const float axes[8]) {
  for (int i = 0; i < 8; i++) {
    axes_values_[i] = axes[i];
  }
  joystick_left_->setX(axes[0]);
  joystick_left_->setY(axes[1]);

  joystick_right_->setX(axes[3]);
  joystick_right_->setY(axes[4]);
}
ControllerWidget::ControllerWidget() {
  setGeometry(0, 0, 240, 150);
  setupUi();
  setInteractionEnabled(true);
}
} // namespace hexapod_rviz_plugins
