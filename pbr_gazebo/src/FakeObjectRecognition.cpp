#include <ros/ros.h>
#include <object_msgs/ObjectInfo.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <pbr_envire_msgs/ObjectCreate.h>
#include <pbr_envire_msgs/ObjectUpdateTransform.h>
#include <pbr_envire_msgs/ObjectUpdateBoundingBox.h>
#include <pbr_envire_msgs/ObjectUpdateTypeConfidences.h>
#include <pbr_envire_msgs/TypeConfidence.h>
using namespace pbr_envire_msgs;
#include <unordered_map>

#include <tf/transform_broadcaster.h>

const std::string type_prefix = "http://envire.semantic/";

int main(int argc, char** args)
{
    ros::init(argc, args, "FakeObjectRecognition");
    ros::NodeHandle node;

    ros::service::waitForService("/gazebo_state_plugins/world/request_object", -1);
    ros::service::waitForService("/gazebo/get_world_properties", -1);
    ros::service::waitForService("ObjectCreate", -1);

    // create clients
    ros::ServiceClient srvList =
        node.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");

    ros::ServiceClient srvInfo =
        node.serviceClient<object_msgs::ObjectInfo>("/gazebo_state_plugins/world/request_object");

    ros::ServiceClient srvCreate =
        node.serviceClient<pbr_envire_msgs::ObjectCreate>("ObjectCreate");

    // pub sockets for object updates
    ros::Publisher pubBox =
        node.advertise<ObjectUpdateBoundingBox>("/ObjectUpdate/BoundingBox/in", 100);

    ros::Publisher pubTransform =
        node.advertise<ObjectUpdateTransform>("/ObjectUpdate/Transform/in", 100);

    ros::Publisher pubTypeConfidences =
        node.advertise<ObjectUpdateTypeConfidences>("/ObjectUpdate/TypeConfidences/in", 100);

    // map of gazebo-obj-name --> semantic object uri
    std::unordered_map<std::string, std::string> gazeboToEnvire;

    // to broadcast a pose between world frame and the robot
    tf::TransformBroadcaster br;

    // looooooop
    ros::Rate rate(1);
    while(ros::ok())
    {// request the names of all objects
        gazebo_msgs::GetWorldProperties g;
        // call
        srvList.call(g);
        // --- compute objects
        std::vector<std::string>& models = g.response.model_names;
        for (int i = 0; i < models.size(); i++) {
            if (models[i].find("ico_v1") == std::string::npos &&
                models[i].find("ground_plane") == std::string::npos) {

                // create service request to gazebo
                object_msgs::ObjectInfo info;
                info.request.name = models[i];
                info.request.get_geometry = true;
                srvInfo.call(info);

                if (models[i].find("tiago") != std::string::npos) {
                    // publish tf!
                    // in this case, objects are located relative to the gazebo
                    // world, which is unknown to the map/robot-stuff.
                    // hence, publish tiago <--> world to create a connection
                    // between world and base_link and display the marker
                    // in the correct position

                    tf::Transform transform;

                    tf::Vector3 origin(
                        info.response.object.origin.position.x,
                        info.response.object.origin.position.y,
                        info.response.object.origin.position.z
                    );
                    tf::Quaternion orientation(
                        info.response.object.origin.orientation.x,
                        info.response.object.origin.orientation.y,
                        info.response.object.origin.orientation.z,
                        info.response.object.origin.orientation.w
                    );
                    transform.setOrigin(origin);
                    transform.setRotation(orientation);

                    if (info.response.object.primitive_origin
                        == object_msgs::Object::ORIGIN_CUSTOM)
                    {
                        // custoM --> transform in object.origin
                    } else if (info.response.object.primitive_origin
                        == object_msgs::Object::ORIGIN_AVERAGE)
                    {
                        // average --> the wheels pose influence the origin
                        // and lead to the map rotating...
                        // --> get the base_footprint as a reference part!
                        int n;
                        node.param<int>("refnum", n, 2 );
                        // which part of gazebos tiago_steel-model is the base-link?
                        // chosen by experimentation: number 2.
                        // adjust if neccessary:
                        // rosparam set /refnum <n>

                        tf::Transform ref;
                        ref.setOrigin(tf::Vector3(
                            info.response.object.primitive_poses[n].position.x,
                            info.response.object.primitive_poses[n].position.y,
                            info.response.object.primitive_poses[n].position.z
                        ));
                        ref.setRotation(tf::Quaternion(
                            info.response.object.primitive_poses[n].orientation.x,
                            info.response.object.primitive_poses[n].orientation.y,
                            info.response.object.primitive_poses[n].orientation.z,
                            info.response.object.primitive_poses[n].orientation.w
                        ));

                        transform = ref;
                    }

                    br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(),
                                        "base_link", "world"));

                } else {
                    ROS_INFO( "Gazebo Object: %s", models[i].c_str() );
                    // not tiago, ground_plane or ico_v1
                    // --> get information and publish stuff

                    float dx = info.response.object.primitives[0].dimensions[0];
                    float dy = info.response.object.primitives[0].dimensions[1];
                    float dz = info.response.object.primitives[0].dimensions[2];

                    // check if semantic object has already been created for this object
                    if (gazeboToEnvire.find(models[i]) == gazeboToEnvire.end())
                    {
                        // no? then create one!
                        ObjectCreate create;
                        create.request.transformation.rotation.x = 1;

                        create.request.box.min_corner.x = -dx/2.;
                        create.request.box.min_corner.y = -dy/2.;
                        create.request.box.min_corner.z = -dz/2.;
                        create.request.box.max_corner.x = dx/2.;
                        create.request.box.max_corner.y = dy/2.;
                        create.request.box.max_corner.z = dz/2.;
                        srvCreate.call(create);

                        gazeboToEnvire[models[i]] = create.response.uri;
                    }

                    // set type confidence based on name
                    size_t digitId = models[i].find_first_of("0123456789");
                    ObjectUpdateTypeConfidences typeUpdate;
                    typeUpdate.uri = gazeboToEnvire[models[i]];
                    TypeConfidence typeConf;
                    typeConf.confidence = 1.0;
                    if (digitId != std::string::npos && digitId > 0) {
                        typeConf.uri = type_prefix + models[i].substr(0, digitId);
                    } else {
                        typeConf.uri = type_prefix + models[i];
                    }
                    typeUpdate.confidences.push_back(typeConf);
                    pubTypeConfidences.publish(typeUpdate);

                    // publish update of transformation
                    ObjectUpdateTransform tfUpdate;
                    tfUpdate.uri = gazeboToEnvire[models[i]];
                    tfUpdate.transformation.translation.x = info.response.object.primitive_poses[0].position.x;
                    tfUpdate.transformation.translation.y = info.response.object.primitive_poses[0].position.y;
                    tfUpdate.transformation.translation.z = info.response.object.primitive_poses[0].position.z;
                    tfUpdate.transformation.rotation = info.response.object.primitive_poses[0].orientation;

                    pubTransform.publish(tfUpdate);

                    // also, publish update of bounding box (debug: rescaling in gazebo)
                    ObjectUpdateBoundingBox boxUpdate;
                    boxUpdate.uri = gazeboToEnvire[models[i]];
                    boxUpdate.box.min_corner.x = -dx/2.;
                    boxUpdate.box.min_corner.y = -dy/2.;
                    boxUpdate.box.min_corner.z = -dz/2.;
                    boxUpdate.box.max_corner.x = dx/2.;
                    boxUpdate.box.max_corner.y = dy/2.;
                    boxUpdate.box.max_corner.z = dz/2.;

                    pubBox.publish(boxUpdate);

                }
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
