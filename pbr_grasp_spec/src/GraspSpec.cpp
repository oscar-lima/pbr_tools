#include <ros/ros.h>
#include <ros/package.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <map>
#include <functional>
#include <iostream>
#include <fstream>

#include "GraspMarker.h"

using namespace visualization_msgs;
using namespace interactive_markers;


class ObjectGraspManager {
private:
    std::string name;
    std::string meshfolder, graspfolder;
    
    InteractiveMarkerServer* server = NULL;
    GraspMarker int_marker;
    Marker mesh_marker;
    InteractiveMarkerControl mesh_control; // just to put the mesh into the interactive marker
    InteractiveMarkerControl mx, my, mz; // move controls
    InteractiveMarkerControl rx, ry, rz; // rotate controls
    
    // menu to add grasps
    MenuHandler menu;
    MenuHandler::EntryHandle menu_set_pregrasp;
    MenuHandler::EntryHandle menu_add_grasp;
    MenuHandler::EntryHandle menu_export_grasps;
    
    // menu to remove grasps
    MenuHandler menu_rm;
    
    // list of already known grasps for this object
    // std::vector<InteractiveMarker> knownGrasps;
    std::map<std::string, GraspMarker> knownGrasps; // map for name->marker association
    int countGrasps;
    
    // information about the current grasp and pregrasp
    geometry_msgs::Pose currentPose;
    geometry_msgs::Pose pregrasp;
    geometry_msgs::Pose grasp;
    
    // publish current pose of the object as tf
    tf::TransformBroadcaster tfBroadcast;
    ros::Timer tfTimer;
    ros::NodeHandle nh;
    
public:
    ObjectGraspManager(const std::string& meshfolder, const std::string& graspfolder) 
        : nh("~") 
    {
        this->meshfolder = meshfolder;
        this->graspfolder = graspfolder;
        
        menu_set_pregrasp = menu.insert( "set pregrasp", 
            std::bind(&ObjectGraspManager::setPregrasp, this, std::placeholders::_1) );
        menu_add_grasp = menu.insert( "add grasp", 
            std::bind(&ObjectGraspManager::addGrasp, this, std::placeholders::_1) );
        menu_export_grasps = menu.insert( "export grasps",
            std::bind(&ObjectGraspManager::exportGrasps, this, std::placeholders::_1) );
            
        menu_rm.insert( "remove", 
            std::bind(&ObjectGraspManager::removeGrasp, this, std::placeholders::_1) );
        
        countGrasps = 0;
    }
    
    
    void init(std::string name, std::string frame_id) {
        this->name = name;        
        // important: relative to gripper_link, that way the inverse of the
        // current pose is the desired pose of the gripper_link to the object!
        // int_marker.header.frame_id = "gripper_link";
        int_marker.header.frame_id = frame_id;
        
        int_marker.header.stamp = ros::Time::now();
        int_marker.name = name;
        int_marker.description = "";
        
        mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        mesh_marker.scale.x = mesh_marker.scale.y = mesh_marker.scale.z = 1.;
        mesh_marker.color.r = 0.5;
        mesh_marker.color.a = 1.;
        mesh_control.always_visible = true;
        mesh_control.name = "mesh_marker";
        mesh_control.interaction_mode = InteractiveMarkerControl::MENU;
        
        mesh_marker.mesh_resource = "file://" + this->meshfolder + this->name;
        mesh_marker.pose.orientation.w = 1;
        mesh_control.markers.push_back( mesh_marker );
        
        int_marker.controls.push_back( mesh_control );
        
        mx.name = "move_x";
        mx.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        mx.orientation.w = 1;
        mx.orientation.x = 1;
        int_marker.controls.push_back( mx );
        
        my.name = "move_y";
        my.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        my.orientation.w = 1;
        my.orientation.y = 1;
        int_marker.controls.push_back( my );
        
        mz.name = "move_z";
        mz.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        mz.orientation.w = 1;
        mz.orientation.z = 1;
        int_marker.controls.push_back( mz );
        
        rx = mx;
        rx.name = "rotate_x";
        rx.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back( rx );
        
        ry = my;
        ry.name = "rotate_y";
        ry.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back( ry );
        
        rz = mz;
        rz.name = "rotate_z";
        rz.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back( rz );
        
        // setup tf
        this->tfTimer = this->nh.createTimer(ros::Duration(0.5),
            boost::bind(&ObjectGraspManager::pubTF, this, 0, _1));
    }
    
    void pubTF(int, const ros::TimerEvent& event){
        tf::Transform transform;
        transform.setIdentity();
        
        tf::poseMsgToTF(this->currentPose, transform);
        this->tfBroadcast.sendTransform(
            // tf::StampedTransform(transform, ros::Time::now(), "gripper_link", "object"));
            tf::StampedTransform(transform, ros::Time::now(), int_marker.header.frame_id, "object"));
    }
    
    InteractiveMarker& getIntMarker() { 
        this->int_marker.header.stamp = ros::Time::now();
        return this->int_marker; 
    }
    
    
    void addToServer(InteractiveMarkerServer& server) {
        server.insert(
            this->getIntMarker(),
            std::bind(&ObjectGraspManager::processFeedback, this, std::placeholders::_1)
        );
        
        this->server = &server;
        this->menu.apply(server, this->int_marker.name);
    }
    
    void processFeedback( const InteractiveMarkerFeedbackConstPtr &feedback )
    {
        this->currentPose = feedback->pose;
    }
    
    void setPregrasp( const InteractiveMarkerFeedbackConstPtr &feedback )
    {
        this->pregrasp = this->currentPose;
    }
    
    void addGrasp( const InteractiveMarkerFeedbackConstPtr &feedback )
    {
        this->grasp = this->currentPose;
        
        // TODO: add to known grasps, add as a marker, visualize,
        // be able to remove a grasp, show/hide all grasps ...
        
        geometry_msgs::Pose msgGrasp, msgPregrasp;
        
        tf::Pose pre, grasp;
        tf::poseMsgToTF(this->pregrasp, pre);
        tf::poseMsgToTF(this->grasp, grasp);
        
        // define rotation to point the arrow along the z axis
        tf::Quaternion quat;
        // quat.setRPY(0, -3.1415/2., 0);
        quat.setRPY(0, 0, 0); // ?? todo: adjust for arm_tool_link
        tf::Transform rot;
        rot.setRotation(quat);
        
        pre = (rot * pre).inverse();
        grasp = (rot * grasp).inverse();
        tf::poseTFToMsg(pre, msgPregrasp);
        tf::poseTFToMsg(grasp, msgGrasp);
        
        GraspMarker int_grasp = 
            GraspMarker::create( msgPregrasp, msgGrasp, this->countGrasps++ );
        int_grasp.originalPre = this->pregrasp; // save the not-rotated poses
        int_grasp.originalGrasp = this->grasp;
            
        this->knownGrasps[int_grasp.name] = int_grasp;
        
        this->server->insert( int_grasp );
        this->menu_rm.apply( *this->server, int_grasp.name );
        this->server->applyChanges();
        
        // TODO need to save inverted grasp and pregrasp (to specify the position
        // of the gripper relative to the object, not vice versa)
        // but visualization should be easier with the non-inverted poses
    }
    
    
    void removeGrasp( const InteractiveMarkerFeedbackConstPtr &feedback )
    {
        ROS_WARN( "TODO: remove %s", feedback->marker_name.c_str() );
        this->server->erase(feedback->marker_name);
        this->knownGrasps.erase(feedback->marker_name);
        this->server->applyChanges();
    }
    
    
    void exportGrasps( const InteractiveMarkerFeedbackConstPtr &feedback )
    {
        std::cout << "------------------------------------------------" << '\n';
        std::cout << "------------------------------------------------" << '\n';
        std::string all_grasps = "";
        for (auto it = this->knownGrasps.begin(); it != this->knownGrasps.end(); it++)
        {
            // geometry_msgs::Pose pre = it->second.controls[0].markers[0].pose;
            // geometry_msgs::Pose grasp = it->second.controls[0].markers[1].pose;
            geometry_msgs::Pose pre = it->second.originalPre;
            geometry_msgs::Pose grasp = it->second.originalGrasp;
            std::string graspString =
                "- grasp_setting:"
                "\n    name: " + it->first +
                "\n    grasp: "
                "\n        position: "
                "\n            x: " + std::to_string(grasp.position.x) +
                "\n            y: " + std::to_string(grasp.position.y) +
                "\n            z: " + std::to_string(grasp.position.z) +
                "\n        orientation: "
                "\n            x: " + std::to_string(grasp.orientation.x) +
                "\n            y: " + std::to_string(grasp.orientation.y) +
                "\n            z: " + std::to_string(grasp.orientation.z) +
                "\n            w: " + std::to_string(grasp.orientation.w) +
                "\n    pregrasp: "
                "\n        position: "
                "\n            x: " + std::to_string(pre.position.x) +
                "\n            y: " + std::to_string(pre.position.y) +
                "\n            z: " + std::to_string(pre.position.z) +
                "\n        orientation: "
                "\n            x: " + std::to_string(pre.orientation.x) +
                "\n            y: " + std::to_string(pre.orientation.y) +
                "\n            z: " + std::to_string(pre.orientation.z) +
                "\n            w: " + std::to_string(pre.orientation.w) +
                "\n";
                all_grasps += graspString;
        }
        std::cout << all_grasps;
        std::ofstream grasp_file;
        // grasp_file.open( "tmp_markers.yml" );
        grasp_file.open( this->graspfolder + this->name + ".yml" );
        grasp_file << all_grasps;
        grasp_file.close();
        std::cout << "------------------------------------------------" << '\n';
        std::cout << "------------------------------------------------" << '\n';
    }
    
};

int main(int argc, char** args) {
    ros::init(argc, args, "GraspSpec");
    ros::NodeHandle n("~");
    
    std::string pkg, meshFolder, graspsFolder, mesh, target_frame;
    n.param<std::string>("resource_pkg", pkg, "pbr_resources");
    n.param<std::string>("mesh_folder", meshFolder, "meshes");
    n.param<std::string>("grasps_folder", graspsFolder, "grasps");
    n.param<std::string>("mesh", mesh, "coke_can.dae");
    n.param<std::string>("target_frame", target_frame, "arm_tool_link");
    
    InteractiveMarkerServer server("markers");
    
    std::string pkgPath = ros::package::getPath(pkg);
    meshFolder = pkgPath + "/" + meshFolder + "/";
    graspsFolder = pkgPath + "/" + graspsFolder + "/";
    
    ROS_WARN("pbr_resources: %s", pkgPath.c_str());
    
    // std::string mesh = "Axe.dae";
    // std::string mesh = "coke_can.dae";
    
    ObjectGraspManager gm(meshFolder, graspsFolder);
    gm.init(mesh, target_frame);
    gm.addToServer(server);
    
    server.applyChanges();
        
    ros::spin();
    return 0;
}
