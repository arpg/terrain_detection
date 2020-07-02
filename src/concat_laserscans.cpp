#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Concat_Laserscan {
    public:
        Concat_Laserscan();
        void leftScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void rightScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void concatScans(const sensor_msgs::LaserScan::ConstPtr& left_scan, const sensor_msgs::LaserScan::ConstPtr& right_scan);
        void publishScanOut();

    private:
        ros::NodeHandle node_;
        ros::Publisher laserscan_publisher_;

        ros::Subscriber left_scan_sub_;
        ros::Subscriber right_scan_sub_;

        sensor_msgs::LaserScan last_left_scan_;
        sensor_msgs::LaserScan last_right_scan_;
        sensor_msgs::LaserScan scan_out_;

        ros::Time last_left_scan_time_;
        ros::Time last_right_scan_time_;

        std::string vehicle_name_;
        int publish_rate_;
        double right_scan_limit_;
        double left_scan_limit_;

        int num_scan_points_;
        double scan_timeout_s_;
};

Concat_Laserscan::Concat_Laserscan(){
        left_scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("terrain_detection/scan_out_left", 100, &Concat_Laserscan::leftScanCallback, this);
        right_scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("terrain_detection/scan_out_right", 100, &Concat_Laserscan::rightScanCallback, this);
        laserscan_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("terrain_detection/concat_scan_out", 100, false);
        ros::NodeHandle nh("~");
        nh.param("publish_rate", publish_rate_, 0);
        nh.param("num_scan_points", num_scan_points_, 0);
        nh.param("right_scan_limit", right_scan_limit_, -0.5);
        nh.param("left_scan_limit", left_scan_limit_, 0.5);
        nh.param("scan_timoue_s", scan_timeout_s_, 0.5);

        scan_out_.angle_min = left_scan_limit_;
        scan_out_.angle_min = right_scan_limit_;

        last_left_scan_time_ = ros::Time::now();
        last_right_scan_time_ = ros::Time::now();
}

void Concat_Laserscan::leftScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    last_left_scan_ = *scan;
    last_left_scan_time_ = ros::Time::now();
}

void Concat_Laserscan::rightScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    last_left_scan_ = *scan;
    last_right_scan_time_ = ros::Time::now();
}

void Concat_Laserscan::concatScans(const sensor_msgs::LaserScan::ConstPtr& left_scan, const sensor_msgs::LaserScan::ConstPtr& right_scan){
    scan_out_.ranges.clear();
    if((ros::Time::now() - last_left_scan_time_).toSec() > scan_timeout_s_){
        // Haven't recieved a left scan in a while
        for(int i = 0; i < num_scan_points_; i++){
            scan_out_.ranges.push_back(10.0);
        }
    } else {
        for(int i = 0; i < num_scan_points_; i++){
            scan_out_.ranges.push_back(left_scan->ranges[i]);
        }
    }
    if((ros::Time::now() - last_right_scan_time_).toSec() > scan_timeout_s_){
        // Haven't recieved a rightt scan in a while
        for(int i = 0; i < num_scan_points_; i++){
            scan_out_.ranges.push_back(10.0);
        }
    }  else {
        for(int i = 0; i < num_scan_points_; i++){
            scan_out_.ranges.push_back(right_scan->ranges[i]);
        }
    }
}

void Concat_Laserscan::publishScanOut(){
    scan_out_.header.stamp = ros::Time::now();
    laserscan_publisher_.publish(scan_out_);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "concat_terrain_laserscan");
    Concat_Laserscan concat_laserscan;

    ros::Rate loop_rate(10);

    while(ros::ok()){
      ros::spinOnce();
      concat_laserscan.publishScanOut();
      loop_rate.sleep();
    }

    return 0;
}
