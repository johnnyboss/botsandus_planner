#ifndef BOTS_AND_US_PLANNER_H_
#define BOTS_AND_US_PLANNER_H_

#include <ros/ros.h>

#include <nav_core/base_local_planner.h>

#include <base_local_planner/goal_functions.h>

// time
#include <time.h>

//files
#include <fstream>
#include <iostream>
using namespace std;

// msgs
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

// transforms
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>



#include <tf2_ros/buffer.h>


// other
#include <array>
#include <vector>

// definitions
#define PI 3.14159265
#define D2R 0.0174532925      // = 3.14159265/180


namespace botsandus_planner
{
    struct Pos {

    double x, y, w;    

   };

    class BotsAndUsPlannerROS : public nav_core::BaseLocalPlanner
    {

    public:
        BotsAndUsPlannerROS();

        /**
       * @brief  Destructor for the wrapper
       */
        ~BotsAndUsPlannerROS();
        /**
         * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
         * @return True if a valid velocity command was found, false otherwise
         */
        void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Set the plan that the controller is following; also reset Simple-planner
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

      /**
      * @brief getYaw: function calculates the Yaw angle from a given orientation
      * @param msg: passes the quaternion for the orientation
      */
      double getYaw(geometry_msgs::Quaternion orientation);
      // Velocity methods
      /**
      * @brief setLinearVel: function that sets linear speed 
      */
      void setLinearVel();

      /**
      * @brief setAngVel: function that sets angular speed 
      */
      void setAngVel();

      /**
      * @brief Stops the robot
      */
      bool stopRobot();

      geometry_msgs::Twist cmd; // contains the velocity
    private:

      //Pointer to external objects (do NOT delete object)
      costmap_2d::Costmap2DROS* costmap_ros_; ///<@brief pointer to costmap  
      costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)

      // Topics & Services
      ros::Subscriber amcl_sub; ///<@brief subscribes to the amcl topic
      ros::Publisher path_pub; ///<@brief publishes to the bubble shape to visualize on rviz 

      tf2_ros::Buffer* tf_; //!< pointer to tf buffer

      // Data
      Pos currPt; // present frame
      Pos next; // next frame
      Pos nError; // error between present and next frames
      double distance;
      int length; // number of frames in the global plan 
      int count; // keeps track of the number for the next frame in the global plan
      std::vector<geometry_msgs::PoseStamped> plan; // contains the global plan
      
      visualization_msgs::Marker points;
      double average;
      int num;
      double minus;
      double beforee;
      ofstream file;

     //measuring
      double stopTime, startTime;
      bool firstTime, hasStarted;
      double pathLength;

      double finalOrientation;
      // Flags
      bool goal_reached_;
      bool initialized_;
      double lastAng;
      bool lastPt;



      // Methods
      /**
       * @brief Amcl Callback: function is called whenever a new amcl msg is published on that topic 
       * @param Pointer to the received message
       */
      void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg);

      
      /**
      * @brief setcurrError: calculates the error between the next path point and current Position
      */
      void setcurrError();

      /**
      * @brief getNext: get next path point 
      */
      void getNext();

    /**
      * @brief visualizePoint: publishes a marker for visualization 
      * @param prune: visualize whole path or just next target
      */
      void visualizePoint(Pos p, bool prune);

    };
};
#endif