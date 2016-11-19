#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>
#include <lwr_controllers/CartesianImpedancePoint.h>
#include <tf/transform_listener.h>

#include <rviz_visual_tools/rviz_visual_tools.h>
rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
ros::Publisher pub_reference;
ros::Publisher pub_pose;
lwr_controllers::CartesianImpedancePoint wp;
tf::TransformListener *tf_l;

void publishStuff(const geometry_msgs::PoseConstPtr& pose_msgs)
{    
    if(!visual_tools_)
    {
        visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("vito_anchor","/rviz_visual_markers"));
    }
    
    visual_tools_->publishAxis(*pose_msgs);
    
    wp.x_FRI = *pose_msgs;
    pub_reference.publish(wp);
    ros::spinOnce();

}

// bool get_initial_transformation(wg_planning::wall_grasp_initialize::Request &req, wg_planning::wall_grasp_initialize::Response &res)
bool get_initial_transformation(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    if(!tf_l->waitForTransform("right_arm_7_link","vito_anchor",ros::Time(0), ros::Duration(1.0)))
        return false;
    
    tf::StampedTransform transform;
    tf_l->lookupTransform("vito_anchor", "right_arm_7_link", ros::Time(0), transform);
    std::cout << "Init transform \nPosition x,y,z - Quaternion w,x,y,z:\nee_position = [" <<
    transform.getOrigin().x() << " "<< 
    transform.getOrigin().y() << " "<<
    transform.getOrigin().z() << "].'; \nee_quaternion = ["<<
    transform.getRotation().w() << " "<<
    transform.getRotation().x() << " "<<
    transform.getRotation().y() << " "<<
    transform.getRotation().z() << "]; "<<
    std::endl;
    
    geometry_msgs::Pose World_ee;
    World_ee.position.x = transform.getOrigin().x();
    World_ee.position.y = transform.getOrigin().y();
    World_ee.position.z = transform.getOrigin().z();
    World_ee.orientation.w = transform.getRotation().w();
    World_ee.orientation.x = transform.getRotation().x();
    World_ee.orientation.y = transform.getRotation().y();
    World_ee.orientation.z = transform.getRotation().z();
    
    pub_pose.publish(World_ee);
    
    return true;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "testPublisher");
    ros::NodeHandle nh;
    ros::Subscriber sub_waypoints = nh.subscribe("wg_pose_reference", 10, &publishStuff);
    ros::ServiceServer service = nh.advertiseService("get_init_transformation", get_initial_transformation);
    pub_reference = nh.advertise<lwr_controllers::CartesianImpedancePoint>("/right_arm/cartesian_impedance_controller/command",1, false);
    pub_pose = nh.advertise<geometry_msgs::Pose>("wall_grasp_init_ee_pose",1, false);
    tf_l =  new tf::TransformListener;
    
    wp.k_FRI.x = 800.0;
    wp.k_FRI.y = 800.0;
    wp.k_FRI.z = 800.0;
    wp.k_FRI.rx = 50.0;
    wp.k_FRI.ry = 50.0;
    wp.k_FRI.rz = 50.0;
    
    // Initial Cartesian damping
    wp.d_FRI.x = 0.8;
    wp.d_FRI.y = 0.8;
    wp.d_FRI.z = 0.8;
    wp.d_FRI.rx = 0.8;
    wp.d_FRI.ry = 0.8;
    wp.d_FRI.rz = 0.8;
    
    // Initial force/torque measure
    wp.f_FRI.force.x = 0.0;
    wp.f_FRI.force.y = 0.0;
    wp.f_FRI.force.z = 0.0;
    wp.f_FRI.torque.x = 0.0;
    wp.f_FRI.torque.y = 0.0;
    wp.f_FRI.torque.z = 0.0;
    
    while(ros::ok())
    {
        ros::spinOnce();
        usleep(2000);
    }
    
return 0;
}

