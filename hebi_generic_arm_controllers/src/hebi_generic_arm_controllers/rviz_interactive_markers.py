import math as m
import copy

import rospy

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerInit, InteractiveMarkerControl, \
    InteractiveMarkerPose, InteractiveMarkerUpdate, InteractiveMarkerFeedback, Marker


def make_box_marker(msg):
    # TODO: parameterize to support other shapes/settings
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.1
    marker.scale.y = msg.scale * 0.1
    marker.scale.z = msg.scale * 0.1
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 0.9
    return marker


def make_box_control_marker(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(make_box_marker(msg))
    msg.controls.append(control)
    return control


class InteractiveMarkerManager(object):
    def __init__(self, server_ns='interactive_markers'):

        self.server = InteractiveMarkerServer(server_ns)

        self._menu_handlers = {}
        self._menu_cmds = {}

        self.markers_created_cnt = 0
        self.marker_cnt = 0
        self._int_marker_name_list = []
        self.name_to_marker_dict = {}
        self.name_to_position_only_flag = {}

    def _process_feedback(self, feedback):
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.logdebug(s + ": button click" + mp + ".")
            pass
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            rospy.logdebug(s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + ".")
            self._process_menu_select(feedback)
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:  # TODO: I really want to use the other type of feedback
            rospy.logdebug(s + ": pose changed")
            self._update_marker_pose(feedback.marker_name, feedback.pose)
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.logdebug(s + ": mouse down" + mp + ".")
            pass
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.logdebug(s + ": mouse up" + mp + ".")
            pass
        self.server.applyChanges()

    def _process_menu_select(self, feedback):
        menu_entry_id = feedback.menu_entry_id
        if menu_entry_id == self._menu_cmds['position_only']:
            self._set_position_only_checkmark(feedback)
        else:
            pass

    def _set_position_only_checkmark(self, feedback):
        menu_entry_handle = feedback.menu_entry_id
        menu_handler = self._menu_handlers[feedback.marker_name]
        check_state = menu_handler.getCheckState(menu_entry_handle)
        if check_state == MenuHandler.UNCHECKED:
            menu_handler.setCheckState(menu_entry_handle, MenuHandler.CHECKED)
            self.name_to_position_only_flag[feedback.marker_name] = True
        else:
            menu_handler.setCheckState(menu_entry_handle, MenuHandler.UNCHECKED)
            self.name_to_position_only_flag[feedback.marker_name] = False
        # Apply marker changes
        menu_handler.reApply(self.server)
        self.server.applyChanges()

    def change_marker_color(self, marker_name, color, opacity=1.0):
        assert 0.0 <= opacity <= 1.0
        # set color
        int_marker = self.server.get(marker_name)
        marker = int_marker.controls[0].markers[0]
        replacement_int_marker = copy.deepcopy(int_marker)
        replacement_marker = replacement_int_marker.controls[0].markers[0]
        rgb = None
        if color == "white":
            rgb = (1.0, 1.0, 1.0)
        elif color == "red":
            rgb = (1.0, 0.0, 0.0)
        # TODO: Additional colors here
        changed = False
        if rgb is not None:
            if marker.color.r != rgb[0]:
                replacement_marker.color.r = rgb[0]
                changed = True
                print("change to ", rgb)
            if marker.color.g != rgb[1]:
                replacement_marker.color.g = rgb[1]
                changed = True
            if marker.color.b != rgb[2]:
                replacement_marker.color.b = rgb[2]
                changed = True
            if marker.color.a != opacity:
                replacement_marker.color.a = opacity
                changed = True
        # update marker
        if changed:
            self._replace_marker(int_marker, replacement_int_marker)

    def _replace_marker(self, old_int_marker, replacement_int_marker):
        # Update dictionaries
        self.name_to_marker_dict[old_int_marker.name] = replacement_int_marker
        # Erase marker from server
        self.server.erase(old_int_marker.name)
        self.server.insert(replacement_int_marker, self._process_feedback)
        self.server.applyChanges()

    def add_marker(self, initial_pose, frame="base_link", description=None):
        self.markers_created_cnt += 1
        self.marker_cnt += 1

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame
        int_marker.pose.position = initial_pose.position
        int_marker.scale = 0.1
        int_marker.name = rospy.get_name() + str(self.markers_created_cnt)
        if description is None:
            int_marker.description = int_marker.name
        else:
            int_marker.description = description
        self.name_to_marker_dict[int_marker.name] = int_marker
        self.name_to_position_only_flag[int_marker.name] = True

        # insert a box  # TODO: may change geometry / eff mesh
        make_box_control_marker(int_marker)
        int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        self.server.insert(int_marker, self._process_feedback)

        # add menus
        menu_handler = MenuHandler()
        pos_opt_handle = menu_handler.insert("Position-Only Ctrl", callback=self._process_feedback)
        self._menu_cmds['position_only'] = pos_opt_handle
        menu_handler.setCheckState(pos_opt_handle, MenuHandler.CHECKED)

        menu_handler.apply(self.server, int_marker.name)
        self._menu_handlers[int_marker.name] = menu_handler
        self.server.applyChanges()
        self._int_marker_name_list.append(int_marker.name)

        return int_marker.name

    def _update_marker_pose(self, marker_name, new_pose):
        self.name_to_marker_dict[marker_name].pose = new_pose

    def get_marker_pose(self, marker_name):
        return self.name_to_marker_dict[marker_name].pose

    def clear_markers(self):
        self.server.clear()
        self.server.applyChanges()

        self._menu_handlers = {}
        self._menu_cmds = {}

        self.markers_created_cnt = 0
        self.marker_cnt = 0
        self._int_marker_name_list = []
        self.name_to_marker_dict = {}
        self.name_to_position_only_flag = {}
