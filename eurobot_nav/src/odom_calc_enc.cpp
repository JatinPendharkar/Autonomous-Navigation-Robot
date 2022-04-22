#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "std_msgs/Int32.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"



#define DIST_DIFF_EPSILON 1.0e-128

struct OdomCalculate{
    using OdomMsg_t = nav_msgs::Odometry;
    
    OdomCalculate(ros::NodeHandle& _nh) : nh(_nh), loop_rate(30){
        odom_msg.child_frame_id = "base_link";
        odom_msg.header.frame_id = "odom";
        odom_msg.pose.pose.orientation.x = 0.0f;
        odom_msg.pose.pose.orientation.y = 0.0f;
        odom_msg.pose.pose.position.z = 0.0f;
	odom_msg.pose.pose.orientation.w = 1.0f;
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.z = 0.0f;
        transformStamped.transform.rotation.x = 0.0f;
        transformStamped.transform.rotation.y = 0.0f;
	transformStamped.transform.rotation.w = 1.0f;
    previous_pos[0]=0;
    previous_pos[1]=0;
    current_pos[0]=0;
    current_pos[1]=0;
    yaw_current = 0.0f;
        ticks_per_meter = 195;
        track_width = 0.23f;

        enc_left_sub = nh.subscribe("/FL_sub", 10, &OdomCalculate::encoderLeftCb, this);
        enc_right_sub = nh.subscribe("/FR_sub", 10, &OdomCalculate::encoderRightCb, this);
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 100, false);
    };
    ~OdomCalculate(){
        odom_pub.shutdown();
        enc_left_sub.shutdown();
        enc_right_sub.shutdown();
    }
    void run(void) {
        while(ros::ok()) {
            update();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    
    private:
    

    void encoderLeftCb(const std_msgs::Int32::ConstPtr& msg) {
        current_pos[0] = msg->data;
        ROS_INFO("left %d", current_pos[0]);
    }

    void encoderRightCb(const std_msgs::Int32::ConstPtr& msg) {
        current_pos[1] = msg->data;
        ROS_INFO("right %d", current_pos[1]);
    }

    void update(void){
        float pos_left = current_pos[0] - previous_pos[0];
        float pos_right = current_pos[1] - previous_pos[1];
        previous_pos[0] = current_pos[0];
        previous_pos[1] = current_pos[1];
        float mean_dist = (pos_right+pos_left)*0.5f;
        odom_msg.header.stamp = ros::Time::now();
        transformStamped.header.stamp=ros::Time::now();
        double yaw_cos = std::cos(yaw_current);
        double yaw_sin = std::sin(yaw_current);
        ROS_INFO("yaw current before %f", yaw_current);

        if(abs(pos_left)<DIST_DIFF_EPSILON && abs(pos_right)<DIST_DIFF_EPSILON){
            odom_pub.publish(odom_msg);
            tfBroadcaster.sendTransform(transformStamped);
            return;
        }
        if(abs(pos_left - pos_right) < DIST_DIFF_EPSILON){
            odom_msg.pose.pose.position.x += (mean_dist * yaw_cos);
            odom_msg.pose.pose.position.y += (mean_dist * yaw_sin);
        }
        else{
            float theta = (pos_right - pos_left)/track_width;
            ROS_INFO("yaw current after  %f", theta);
            
            float centerToICR = mean_dist / theta;
            float local_y = centerToICR * (1-std::cos(theta));
            float local_x = centerToICR * std::sin(theta);
            
            odom_msg.pose.pose.position.y += local_y * yaw_cos + local_x * yaw_sin;
            odom_msg.pose.pose.position.x += local_x * yaw_cos - local_y * yaw_sin;
            yaw_current+=theta;
            ROS_INFO("yaw current after  %f", yaw_current);
            odom_msg.pose.pose.orientation.z = std::sin(yaw_current*0.5f);
            odom_msg.pose.pose.orientation.w = std::cos(yaw_current*0.5f);
        }
        transformStamped.transform.translation.x = odom_msg.pose.pose.position.x;
        transformStamped.transform.translation.y = odom_msg.pose.pose.position.y;
        transformStamped.transform.rotation.z = odom_msg.pose.pose.orientation.z;
        transformStamped.transform.rotation.w = odom_msg.pose.pose.orientation.w;
        odom_pub.publish(odom_msg);
        tfBroadcaster.sendTransform(transformStamped);
        
    };

    

    ros::NodeHandle& nh;
    OdomMsg_t odom_msg;
    ros::Publisher odom_pub;
    ros::Subscriber enc_left_sub;
    ros::Subscriber enc_right_sub;
    int32_t ticks_per_meter;
    int32_t previous_pos[2];
    int32_t current_pos[2];
    float track_width;
    float yaw_current;
    ros::Rate loop_rate;
    tf2_ros::TransformBroadcaster tfBroadcaster;
    geometry_msgs::TransformStamped transformStamped;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom_calc_enc_node");
    ros::NodeHandle nh;
    OdomCalculate odomCalculate(nh);
    odomCalculate.run();
    ros::spin();
    nh.shutdown();
    return 0;
}
