// TODO:
//- [ ] Clean up class
//- [ ] Generalize it (remove static base link, paramaterize size)
//- [ ] Add callback methods
//- [ ] Add Context menu
//- [ ] Add buttons
//- [ ] Add hide buttons
//- [ ] Add Color to marker

#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <interactive_markers/interactive_marker_client.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <math.h>

using namespace visualization_msgs::msg;

class TranslationMarker : public InteractiveMarker {

private:
  bool fixed = true;
  bool show_6dof = false;

  Marker makeSphereMarker(float size = 0.25) {
    Marker marker;
    marker.type = Marker::SPHERE;
    marker.scale.x = scale * size;
    marker.scale.y = scale * size;
    marker.scale.z = scale * size;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;
    return marker;
  }
  InteractiveMarkerControl &makeSphereControl() {
    // Making the Center Sphere

    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeSphereMarker(0.15));
    controls.push_back(control);

    return controls.back();
  }

  // FIXME: Fix this
  //  void makeButtonMarker(const Eigen::Vector3d position) {
  //    InteractiveMarker int_marker;
  //    header.frame_id = "base_link";
  //    scale = 1;
  //
  //    name = "button";
  //    description = "Button\n(Left Click)";
  //
  //    InteractiveMarkerControl control;
  //
  //    control.interaction_mode = InteractiveMarkerControl::BUTTON;
  //    control.name = "button_control";
  //
  //    Marker marker = makeBox(int_marker);
  //    control.markers.push_back(marker);
  //    control.always_visible = true;
  //    controls.push_back(control);
  //  }

  void setup6DofControl() {
    InteractiveMarkerControl control;

    if (fixed) {
      name += "_fixed";
      description += "\n(fixed orientation)";
      control.orientation_mode = InteractiveMarkerControl::FIXED;
    }

    if (control.interaction_mode != InteractiveMarkerControl::NONE) {
      std::string mode_text;
      if (control.interaction_mode == InteractiveMarkerControl::MOVE_3D)
        mode_text = "MOVE_3D";
      if (control.interaction_mode == InteractiveMarkerControl::ROTATE_3D)
        mode_text = "ROTATE_3D";
      if (control.interaction_mode == InteractiveMarkerControl::MOVE_ROTATE_3D)
        mode_text = "MOVE_ROTATE_3D";
      name += "_" + mode_text;
      description = std::string("3D Control") +
                    (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
    }

    if (show_6dof) {
      control.orientation.w = 1;
      control.orientation.x = 1;
      control.orientation.y = 0;
      control.orientation.z = 0;
      control.name = "rotate_x";
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      controls.push_back(control);
      control.name = "move_x";
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 1;
      control.orientation.z = 0;
      control.name = "rotate_z";
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      controls.push_back(control);
      control.name = "move_z";
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 0;
      control.orientation.z = 1;
      control.name = "rotate_y";
      control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
      controls.push_back(control);
      control.name = "move_y";
      control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
      controls.push_back(control);
    }
  }

  void setupContextMenu() {
    InteractiveMarkerControl control;
    control.interaction_mode = InteractiveMarkerControl::MENU;
    control.name = "menu_only_control";
    control.always_visible = true;

    // Adding Menu Items

    controls.push_back(control);
  }

public:
  interactive_markers::MenuHandler menu_handler;
  TranslationMarker() {
    header.frame_id = "base_link";
    // TODO: Figure out what this does and fix it
    scale = 0.1;
    name = "simple_6dof";

    makeSphereControl();
    controls[0].interaction_mode = InteractiveMarkerControl::MOVE_ROTATE_3D;

    setupContextMenu();
    setup6DofControl();
  }
};
