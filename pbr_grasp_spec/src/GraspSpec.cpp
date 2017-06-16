#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_listener.h>

#include <vector>
#include <functional>

using namespace visualization_msgs;
using namespace interactive_markers;


class ObjectMarker {
private:
    InteractiveMarkerServer* server = NULL;
    InteractiveMarker int_marker;
    Marker mesh_marker;
    InteractiveMarkerControl mesh_control; // just to put the mesh into the interactive marker
    InteractiveMarkerControl mx, my, mz; // move controls
    InteractiveMarkerControl rx, ry, rz; // rotate controls
    
public:
    
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
            std::bind(&ObjectMarker::processFeedback, this, std::placeholders::_1)
        );
        
        this->server = &server;
    }
    
    void processFeedback( const InteractiveMarkerFeedbackConstPtr &feedback )
    {
        // if (feedback->pose.position.x > 0) {
        //     this->int_marker.controls[0].markers[0].color.g = 1.;
        // } else {
        //     this->int_marker.controls[0].markers[0].color.g = 0.;
        // }
        // this->server->insert(this->int_marker);
        // this->server->applyChanges();
    }
    
};


// class TiagoGripperMarker : public ObjectMarker{
// public:
//     TiagoGripperMarker() {
//         std::vector<std::string> files;
//         std::vector<geometry_msgs::Pose> poses;
//         
//         geometry_msgs::Pose pose;
//         pose.orientation.w = 1;
//         
//         std::string pre = "package://tiago_description/meshes/gripper/";
//         files.push_back( pre + "gripper_link.stl" ); poses.push_back( pose );
//         files.push_back( pre + "gripper_finger_link.stl" ); poses.push_back( pose);
//         files.push_back( pre + "gripper_finger_link.stl" ); poses.push_back( pose );
//         
//         init( files, poses, "Tiago-Gripper" );
//     }
// };



// debug test
int main(int argc, char** args) {
    ros::init(argc, args, "GraspSpec");
    
    InteractiveMarkerServer server("markers");
    
    ObjectMarker axe;
    axe.init("package://pbr_resources/meshes/Axe.dae");
    
    axe.addToServer(server);
    
    // TiagoGripperMarker tiago;
    // server.insert( tiago.getIntMarker() );
    // just load the whole robot and use tf to gripper_link.
    
    server.applyChanges();
        
    ros::spin();
    return 0;
}
