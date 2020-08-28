
#ifndef OWN_LOCAL_PLANNER_OWN_LOCAL_PLANNER_H_
#define OWN_LOCAL_PLANNER_OWN_LOCAL_PLANNER_H_

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <tf2_ros/buffer.h>

#include <Eigen/Core>

#include <dynamic_reconfigure/server.h>

#include <own_local_planner/OWNPlannerConfig.h>


#include <nav_core/base_local_planner.h>
#include <base_local_planner/oscillation_cost_function.h>
#include <own_local_planner/transform_global_plan.h>

#include <own_local_planner/join_costmap.h>
#include  <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

#include <geometry_msgs/Vector3.h>
//#include <tf/Vector3.h>

namespace own_local_planner
{
    
 struct pos {
	double x, y, az;	
   };

    class OWNPlanner : public nav_core::BaseLocalPlanner
    {

    public:
        OWNPlanner();
        /**
         * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
         * @return True if a valid velocity command was found, false otherwise
         */
        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

        /**
         * @brief  Check if the goal pose has been achieved by the local planner
         * @return True if achieved, false otherwise
         */
        bool isGoalReached();

        /**
         * @brief  Set the plan that the local planner is following
         * @param plan The plan to pass to the local planner
         * @return True if the plan was updated successfully, false otherwise
         */
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief Constructs the local planner
         * @param name The name to give this instance of the local planner
         * @param tf A pointer to a transform buffer
         * @param costmap_ros The cost map to use for assigning costs to local plans
         */
        void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

        void clear();

        //void fill();

        double dist(float x, float y);

        void publish_plan();

        // void publish_location();

        bool select_goal(geometry_msgs::PoseStamped *);

        double distPoint(geometry_msgs::Point p1, geometry_msgs::Point p2);

        double calcRepGains(double cm_x);

        ~OWNPlanner();

       // void setOrient();

	
      	OWNPlanner(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

    protected:
        void setNext();                           //Bir sonraki hedefe yoneltecek function;

        void sensCallback(const sensor_msgs::LaserScan::Ptr& msg);

        ros::Publisher g_plan_pub_, l_plan_pub_, k_plan_pub_; //Global ve local planin publish edilmesi -> global plandaki her nokta siradaki hedefimiz.
        geometry_msgs::PoseStamped current_pose_;//Su anki poz var;
                                                //Goal var.
        ros::Subscriber sens_sub;               //Sensor verisi var;
        bool initialized_ ;
        costmap_2d::Costmap2DROS* costmap_ros_;
        std::vector<geometry_msgs::PoseStamped> plan; //Globaldan gelen plani tutmak icin kullanilacak.
        int count;                                    //Bir sonraki hedefe gidilmesi icin kullanilcak.
        int length;                                   //Hedef dizisini gezinim icin kullanilcak.
        bool goal_reached_ ;                          //Hedefe ulasip ulasilmadigini tutacak degisken.
        bool local_goal_reached_ ;
        pos next;                                     //Siradaki hedef bilgisini tutacak olan degisken.
        geometry_msgs::PoseStamped goal;
        geometry_msgs::PoseStamped current_goal;
        geometry_msgs::Twist cmd;                     //Velocity komutlarini vermek icin kullanilcak.
        tf2_ros::Buffer* tf_;
        geometry_msgs::Vector3 vectorArray[1440];         //Engellerden gelen itici kuvvetleri tutmak icin kullanilacak.
        sensor_msgs::LaserScan laser_data;
        // std::vector<geometry_msgs::PoseStamped> odom;
        nav_msgs::Path odom;
        double att_Gain;
        double yaw;
        double max_angular;
        double angle_diff ;
        double goal_tolerance_angular;
        double goal_tolerance_linear;
        int poseIndex;
        int minSensIndex;
        int planIndex;
        double min_accept_goal;
        double min_obs_dist;
        geometry_msgs::PoseStamped last_poses_[100] ; //Son 10 noktanin ortalamasini alip belli bir esikten geicirecegim osci durumuna bakmak icin.
        //tf::Vector3 vectorArray[1440];
    };
};
#endif
