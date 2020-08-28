#include <ros/ros.h>

#include <own_local_planner/own_local_planner.h>

#include <pluginlib/class_list_macros.h>

#include <tf2/utils.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>


PLUGINLIB_EXPORT_CLASS(own_local_planner::OWNPlanner, nav_core::BaseLocalPlanner)

using namespace std;
ofstream MyExcelFile2 ("/home/robotic/catkin_ws/src/navigation/own_local_planner/src/local_path.txt", ios::trunc);

namespace own_local_planner
{
    OWNPlanner::OWNPlanner()
    {
    }

    OWNPlanner::OWNPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
         : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {
        initialize(name, tf, costmap_ros);
    }


    void OWNPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if(!initialized_) // Sensor verisine ve su anki konumuna burada sahip ol.
            {
            ros::NodeHandle gn;
            g_plan_pub_ = gn.advertise<nav_msgs::Path>("global_plan", 1);
            k_plan_pub_ = gn.advertise<nav_msgs::Path>("konum_bilgileri", 1);
            costmap_ros_ = costmap_ros;
            costmap_ros_->getRobotPose(current_pose_); //Robotun su anki konumu elde edildi.
            poseIndex=0;
            sens_sub = gn.subscribe("/scan", 100, &OWNPlanner::sensCallback, this); //Sensor verisine sahip olundu. Engeller buradan tespit edilip. Potansiyel kuvvet hesaplanacak.
            //costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap(); //Guncellendiginden emin olmak adina daha sonra bakman lazim
            initialized_ = true;
            max_angular = 5.0;
            goal_tolerance_angular = 0.2;
            goal_tolerance_linear = 0.2;
            min_accept_goal = 0.3;
						min_obs_dist = 0.2;
            // std::vector<geometry_msgs::PoseStamped> odom;
            //local_goal_reached_ = true;

            }
        else
            {
            ROS_WARN("This planner has already been initialized, doing nothing.");
            return;
        }
    }

    void OWNPlanner::publish_plan(){
      nav_msgs::Path  path;
      geometry_msgs::PoseStamped this_pose_stamped;
      for(int i=0;i<length;i++){
        this_pose_stamped.pose.position.x = plan[i].pose.position.x;
        this_pose_stamped.pose.position.y = plan[i].pose.position.y;
        this_pose_stamped.pose.position.z = plan[i].pose.position.z;
        this_pose_stamped.header.stamp = ros::Time::now();
        this_pose_stamped.header.frame_id = "map";
        path.poses.push_back(this_pose_stamped);

        this_pose_stamped.pose.orientation = plan[i].pose.orientation;
        path.header.stamp = ros::Time::now();
        path.header.frame_id="map";
      }
      g_plan_pub_.publish(path);
    }

    // void OWNPlanner::publish_location(){
      
    //   g_plan_pub_.publish(odom);
    // }

    bool OWNPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
    {
        if(!initialized_)
            {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
            }
        //reset next counter
        planIndex = 1; 
        count = 1;
        int i;
        //set plan, length and next goal
        plan = orig_global_plan; 
        length = (plan).size(); 
    
        setNext(); // next bilgileri attractive kuvveti hesaplamada kullanilacak.
        publish_plan(); //Gelen plani publish etmek icin.
        goal_reached_ = false;
        //local_goal_reached_ = true;
        return true;
    }

    void OWNPlanner::clear(){ //Vectorel kuvvetleri tutan diziyi sifirlayacak olan fonksiyon.
      int i=0;
      for(i=0;i<1440;i++) //sensor verisinin uzunlugu.
      {
        vectorArray[i].x = (0);
        vectorArray[i].y = (0);
        vectorArray[i].z = (0);
      }
    }

    double distPoint(geometry_msgs::Point p1 ,geometry_msgs::Point p2 ){
        printf("\n ULAA : %f", (double)sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2)));
        return (double)sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }

    bool OWNPlanner::select_goal(geometry_msgs::PoseStamped *goal){
      int i=0;
      geometry_msgs::PoseStamped pose;
      costmap_ros_->getRobotPose(pose);
      //ROS_INFO("\nPlanIndex: %d, length : %d", planIndex, length );
      //ROS_INFO("A");
      if(length == 0){
        //  ROS_INFO("C");
        return false;
      }
      for(i=planIndex;i<length;i++){
        double dist = sqrt( pow(plan[i].pose.position.x - pose.pose.position.x ,2) + pow(plan[i].pose.position.y - pose.pose.position.y ,2));
        //ROS_INFO("local icin dist = %f", dist);
        if(dist > min_accept_goal)
        {
            *goal = plan[i];
            current_goal = *goal;
            //ROS_INFO("\nSELECTgOAL ICINDE :goal x : %f, goal y : %f", plan[i].pose.position.x,  plan[i].pose.position.y);
            planIndex=i;
            return true;
        }
        else if(dist > min_accept_goal/2.0){
          *goal = plan[i];
            current_goal = *goal;
            //ROS_INFO("\nSELECTgOAL ICINDE :goal x : %f, goal y : %f", plan[i].pose.position.x,  plan[i].pose.position.y);
            planIndex=i;
            return true;
        }
      }
			return false;
    }


    double OWNPlanner::dist(float x,  float y){ //Euclian distance
      ///ROS_INFO("\n Distance degeri : %f", (x-y));
      return (double)(x-y);
    }

    double OWNPlanner::calcRepGains(double cm_x){ //
      
      return  pow(cm_x,3) / (cm_x - min_obs_dist);
    }


    bool OWNPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
      //!!!!!!!!!! Verbose function yaz. Hesaplamalari daha duzgun yap.
      int i=0;
      if(!initialized_)
        {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
        }
      // Bilesik kuvveti hesapla. Bu kuvvete gore hareket komutu uret. Engel itici hedef cekici.
      costmap_ros_->getRobotPose(current_pose_); // current_pose_ da robotun su anki pozu var.

      double rep_X = 0;
      double rep_Y = 0;
      double att_X = 0;
      double att_Y = 0;
      double avg_x = 0;
      double avg_y = 0; 
      double att_Gain = 1/min_accept_goal + 0.1; // 0.1 -> refer epsulon 
      double rep_Gain ;
      double distanceX;
      double distanceY;
      double robot_to_goal;
      float range;
      geometry_msgs::PoseStamped goal;

      MyExcelFile2 << current_pose_.pose.position <<endl;


      // odom.poses.push_back(current_pose_);

      geometry_msgs::PoseStamped this_pose_stamped;
      this_pose_stamped.pose.position.x = current_pose_.pose.position.x;
      this_pose_stamped.pose.position.y = current_pose_.pose.position.y;
      this_pose_stamped.pose.position.z = current_pose_.pose.position.z;
      this_pose_stamped.pose.orientation = current_pose_.pose.orientation;
      this_pose_stamped.header.stamp = ros::Time::now();
      this_pose_stamped.header.frame_id = "map";
      odom.poses.push_back(this_pose_stamped);

      odom.header.stamp = ros::Time::now();
      odom.header.frame_id="map";
      
      if(! select_goal(&goal))
      {
        ROS_INFO("Hedefe ulasildi local hedef yok");
        goal_reached_ = true;
        return true;
        // publish_location();
       
      }
      
      //*** Robot hareket komutunu aldiginda. ilk olarak donusu yap. Sonrada hareket edersin.
      angle_diff = tf2::getYaw(goal.pose.orientation) - tf2::getYaw(current_pose_.pose.orientation);
      // ROS_INFO(" Angle Diff : %f ", angle_diff);
      // ROS_INFO(" GoalYaw : %f ", tf2::getYaw(goal.pose.orientation));

      // if(angle_diff > goal_tolerance_angular){
      //   cmd_vel.linear.x = 0;
      //   cmd_vel.linear.y = 0;
      //   ROS_INFO("\n\n\n\n %f > %f Need to rotate",angle_diff, goal_tolerance_angular);
      //   if(angle_diff > max_angular){
      //     angle_diff = (fabs(angle_diff)/angle_diff) * max_angular;
      //   }
      //   cmd_vel.angular.z = angle_diff;
      //   //ROS_INFO("vel_X : %f , vel_Y : %f , ang_z : %f, count : %d, length :%d",cmd_vel.linear.x,cmd_vel.linear.y, cmd_vel.angular.z,count, length);
      //   return true;
      // } 
			
      //ROS_INFO("Goal.x  = %f , Goal.y = %f", goal.pose.position.x, goal.pose.position.y );
      //ROS_INFO("next.x  = %f , next.y = %f", next.x, next.y );

      //att_X = att_Gain * (next.x - current_pose_.pose.position.x);
      //att_Y = att_Gain * (next.y - current_pose_.pose.position.y);
			
      // ******* Bu noktada cekivi kuvvetleri 0-1 araligina cekmek icin mevcut parametrelre gore dinamik bir att_Gain hesapla .. ******
			att_X = att_Gain * (goal.pose.position.x - current_pose_.pose.position.x);
      att_Y = att_Gain * (goal.pose.position.y - current_pose_.pose.position.y);
      rep_Gain = calcRepGains(0.12); // 0.12 -> refers to min_sensor_range ;
      // rep_Gain = 4;
      // ROS_INFO("\natt.X  = %f , att.Y = %f", att_X , att_Y );
      // *********** Bu noktada repulsive kuvveti 0-1 araligina cekmek icin. mevcut parametrelere gore dinamik bir rep_Gain hesapla.. ******
			double cm_x = vectorArray[minSensIndex].x * 100;
			double cm_y = vectorArray[minSensIndex].y * 100;
			range = sqrt(pow(vectorArray[minSensIndex].x,2) + pow(vectorArray[minSensIndex].y,2));
			if(range<min_obs_dist){
				rep_X = rep_Gain * (1/min_obs_dist - 1/ cm_x) * (1/(cm_x * cm_x));
				rep_Y = rep_Gain * (1/min_obs_dist - 1/ cm_y) * (1/(cm_y * cm_y));
			}
			
      ROS_INFO("\nrep.X  = %f , rep.Y = %f", rep_X , rep_Y );


      cmd_vel.linear.x = att_X + rep_X;
      cmd_vel.linear.y = att_Y + rep_Y;

			yaw = atan2(cmd_vel.linear.y, cmd_vel.linear.x) - tf2::getYaw(current_pose_.pose.orientation);
        // ROS_INFO("GUNCEL YAW %f", tf2::getYaw(current_pose_.pose.orientation));
      if(fabs(yaw) > max_angular){
         yaw = (fabs(yaw)/yaw) * max_angular;
      }
      if (yaw < -M_PI) {
        yaw += 2 * M_PI;
      } else if (yaw > M_PI) {
        yaw -= 2 * M_PI;
      }

			cmd_vel.linear.x = sqrt(pow(cmd_vel.linear.x,2) + pow(cmd_vel.linear.y,2));
      if(cmd_vel.linear.x > 0.2){
        cmd_vel.linear.x = 0.2;
      }
      cmd_vel.angular.z = yaw;

      setNext();
      count = count + 5;
      // ROS_INFO("vel_X : %f , vel_Y : %f , ang_z : %f, count : %d, length :%d",cmd_vel.linear.x,cmd_vel.linear.y, cmd_vel.angular.z,count, length);
      k_plan_pub_.publish(odom);
      return true;
    }

    bool OWNPlanner::isGoalReached()
    {
          // check if plugin initialized
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        // Ulasip ulasmadigini computeVelocityde hesapla.
        return goal_reached_;
                
    }

    OWNPlanner::~OWNPlanner()
    {
    }

    void OWNPlanner::setNext()
  {
    next.x = plan[count].pose.position.x;
    next.y = plan[count].pose.position.y;
    next.az = tf2::getYaw(plan[count].pose.orientation);
  }

  void OWNPlanner::sensCallback(const sensor_msgs::LaserScan::Ptr& msg){
    int i = 0;
    minSensIndex = 0;
    clear();                        //Her adimda etki eden vectorleri temizlemeli.
    double angle=0;                 //aciyi tutmak icin kullanilan degsiken.
    for(i=0; i<1440; i++){
        angle=msg->angle_min + (i*msg->angle_increment);
        vectorArray[i].x = (msg->ranges[i] * cos(angle));
        vectorArray[i].y = (msg->ranges[i] * sin(angle));
        vectorArray[i].z = (0);
       
    }

      for(i=0;i<1440;i++)
      {
        if ((msg->ranges[i] <= msg->ranges[minSensIndex]) && (msg->ranges[i] >= msg->range_min) && (msg->ranges[i] <= msg->range_max))
        {
            minSensIndex = i;
        }
      }
  }
}
