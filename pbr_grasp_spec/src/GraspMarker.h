#pragma once

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_listener.h>

#include <vector>
#include <functional>

using namespace visualization_msgs;
using namespace interactive_markers;


// just visualization of recorded grasps
class GraspMarker {
public:
    static InteractiveMarker create(geometry_msgs::Pose pre, geometry_msgs::Pose grasp, int id)
    {
        Marker vis_pre, vis_grasp;
        
        vis_pre.type = Marker::ARROW;
        vis_pre.pose = pre;
        vis_pre.color.a = 1;
        vis_pre.color.g = 1;
        vis_pre.scale.x = 0.15;
        vis_pre.scale.y = vis_pre.scale.z = 0.02;
        
        vis_grasp = vis_pre;
        vis_grasp.pose = grasp;
        
        InteractiveMarkerControl control;
        control.always_visible = true;
        control.interaction_mode = InteractiveMarkerControl::MENU;
        control.markers.push_back( vis_pre );
        control.markers.push_back( vis_grasp );
        
        InteractiveMarker int_marker;
        int_marker.name = "grasp_" + std::to_string(id);
        int_marker.header.frame_id = "gripper_link";
        int_marker.controls.push_back( control );
        
        return int_marker;
    }
};
