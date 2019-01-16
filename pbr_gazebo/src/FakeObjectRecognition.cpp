#include <ros/ros.h>
#include <object_msgs/ObjectInfo.h>
#include <gazebo_msgs/GetWorldProperties.h>

#include <vision_msgs/Detection3DArray.h>

#include <unordered_map>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char** args)
{
    ros::init(argc, args, "FakeObjectRecognition");
    ros::NodeHandle node;

    ros::service::waitForService("/gazebo_state_plugins/world/request_object", -1);
    ros::service::waitForService("/gazebo/get_world_properties", -1);

    // create clients
    ros::ServiceClient srvList =
        node.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");

    ros::ServiceClient srvInfo =
        node.serviceClient<object_msgs::ObjectInfo>("/gazebo_state_plugins/world/request_object");

    // publisher for detections
    ros::Publisher pubDetections =
        node.advertise<vision_msgs::Detection3DArray>("detected_objects", 100);

    // to broadcast a pose between world frame and the robot
    tf::TransformBroadcaster br;


    // a map to translate a string-type to an int-type.
    // should be loaded from the ontology, I guess.
    std::map<std::string, int> nameToId;
    // TODO: these are just placeholders.
    nameToId["Gearbox"] = 1;
    nameToId["Powerdrill"] = 2;
    nameToId["SupportSurface"] = 3;


    ros::Rate rate(2);
    while(ros::ok())
    {
        vision_msgs::Detection3DArray detections;

        // request the names of all objects
        gazebo_msgs::GetWorldProperties g;
        // call
        srvList.call(g);
        auto stamp = ros::Time::now();
        detections.header.stamp = stamp;
        detections.header.frame_id = "world";

        // --- compute objects
        std::vector<std::string>& models = g.response.model_names;
        for (int i = 0; i < models.size(); i++) {
            // exclude ico_v1 and ground_plane from "detected objects"
            if (models[i].find("ico_v1") == std::string::npos &&
                models[i].find("ground_plane") == std::string::npos) {

                vision_msgs::Detection3D detection;
                detection.header.stamp = stamp;
                detection.header.frame_id = "world";

                vision_msgs::ObjectHypothesisWithPose hypothesis;

                // create service request to gazebo
                object_msgs::ObjectInfo info;
                info.request.name = models[i];
                info.request.get_geometry = true;
                srvInfo.call(info);

                ROS_INFO( "Gazebo Object: %s", models[i].c_str() );

                // set type confidence based on name
                size_t digitId = models[i].find_first_of("0123456789");
                std::string type;
                if (digitId != std::string::npos && digitId > 0) {
                    type = models[i].substr(0, digitId-1);
                } else {
                    type = models[i];
                }

                hypothesis.pose.pose = info.response.object.primitive_poses[0];

                float dx = info.response.object.primitives[0].dimensions[0];
                float dy = info.response.object.primitives[0].dimensions[1];
                float dz = info.response.object.primitives[0].dimensions[2];

                hypothesis.score = 10.0;
                detection.bbox.center = hypothesis.pose.pose;
                detection.bbox.size.x = dx;
                detection.bbox.size.y = dy;
                detection.bbox.size.z = dz;
                detection.results.push_back(hypothesis);

                detections.detections.push_back(detection);
            }
        }

        pubDetections.publish(detections);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
