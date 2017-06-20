#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_listener.h>

#include <vector>
#include <map>
#include <functional>

#include "GraspMarker.h"

using namespace visualization_msgs;
using namespace interactive_markers;


class ObjectGraspManager {
private:
    std::string name;
    
    InteractiveMarkerServer* server = NULL;
    InteractiveMarker int_marker;
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
    std::map<std::string, InteractiveMarker> knownGrasps; // map for name->marker association
    int countGrasps;
    
    // information about the current grasp and pregrasp
    geometry_msgs::Pose currentPose;
    geometry_msgs::Pose pregrasp;
    geometry_msgs::Pose grasp;
    
    
public:
    ObjectGraspManager() {
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
    
    void init(std::string cadfile) {
        std::vector<std::string> v;
        v.push_back(cadfile);
        std::vector<geometry_msgs::Pose> p;
        geometry_msgs::Pose pose;
        pose.orientation.w = 1;
        p.push_back(pose);
        
        init(v, p, cadfile);
    }
    
    void init(std::vector<std::string> cadfiles, 
              std::vector<geometry_msgs::Pose> poses, std::string name) {
        this->name = name;
        // important: relative to gripper_link, that way the inverse of the
        // current pose is the desired pose of the gripper_link to the object!
        int_marker.header.frame_id = "gripper_link";
        
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
        
        for (int i = 0; i < cadfiles.size(); i++) {
            mesh_marker.mesh_resource = cadfiles[i];
            mesh_marker.pose = poses[i];
            mesh_control.markers.push_back( mesh_marker );
        }
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
        
        tf::Pose pre, grasp;
        tf::poseMsgToTF(this->pregrasp, pre);
        tf::poseMsgToTF(this->grasp, grasp);
        
        InteractiveMarker int_grasp = 
            GraspMarker::create( this->pregrasp, this->grasp, this->countGrasps++ );
            
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
        // TODO remove saved grasp?
        // or have a service export them on demand? --> yaml?! :)
        this->knownGrasps.erase(feedback->marker_name);
        this->server->applyChanges();
    }
    
    
    void exportGrasps( const InteractiveMarkerFeedbackConstPtr &feedback )
    {
        std::cout << "------------------------------------------------" << '\n';
        std::cout << "------------------------------------------------" << '\n';
        for (auto it = this->knownGrasps.begin(); it != this->knownGrasps.end(); it++)
        {
            geometry_msgs::Pose pre = it->second.controls[0].markers[0].pose;
            geometry_msgs::Pose grasp = it->second.controls[0].markers[1].pose;
            std::cout << "grasp_setting:" << '\n';
            std::cout << "    name: " << it->first << '\n';
            std::cout << "    grasp: " << '\n';
            std::cout << "        position: " << '\n';
            std::cout << "            x: " << grasp.position.x << '\n';
            std::cout << "            y: " << grasp.position.y << '\n';
            std::cout << "            z: " << grasp.position.z << '\n';
            std::cout << "        orientation: " << '\n';
            std::cout << "            x: " << grasp.orientation.x << '\n';
            std::cout << "            y: " << grasp.orientation.y << '\n';
            std::cout << "            z: " << grasp.orientation.z << '\n';
            std::cout << "            w: " << grasp.orientation.w << '\n';
            std::cout << "    pregrasp: " << '\n';
            std::cout << "        position: " << '\n';
            std::cout << "            x: " << pre.position.x << '\n';
            std::cout << "            y: " << pre.position.y << '\n';
            std::cout << "            z: " << pre.position.z << '\n';
            std::cout << "        orientation: " << '\n';
            std::cout << "            x: " << pre.orientation.x << '\n';
            std::cout << "            y: " << pre.orientation.y << '\n';
            std::cout << "            z: " << pre.orientation.z << '\n';
            std::cout << "            w: " << pre.orientation.w << '\n';
            std::cout << "" << '\n';
        }
        std::cout << "------------------------------------------------" << '\n';
        std::cout << "------------------------------------------------" << '\n';
    }
    
};

int main(int argc, char** args) {
    ros::init(argc, args, "GraspSpec");
    
    InteractiveMarkerServer server("markers");
    
    ObjectGraspManager gm;
    gm.init("package://pbr_resources/meshes/Axe.dae");
    gm.addToServer(server);
    
    server.applyChanges();
        
    ros::spin();
    return 0;
}
