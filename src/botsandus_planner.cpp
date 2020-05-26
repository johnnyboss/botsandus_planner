
#include <ros/ros.h>

#include <botsandus_planner/botsandus_planner.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(botsandus_planner::BotsAndUsPlannerROS, nav_core::BaseLocalPlanner)

namespace botsandus_planner
{

    BotsAndUsPlannerROS::BotsAndUsPlannerROS(): costmap_ros_(NULL), tf_(NULL), initialized_(false), lastPt(false) {}

    void BotsAndUsPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if(!initialized_)
        {   
            // create Node Handle with name of plugin (as used in move_base for loading)
            ros::NodeHandle ph("~/" + name);
            ros::NodeHandle nh;
            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.   
            amcl_sub = nh.subscribe("amcl_pose", 100, &BotsAndUsPlannerROS::amclCallback, this);
            path_pub = ph.advertise<visualization_msgs::Marker>("visualization_marker", 10);

            //initializing the visualization markers
            points.header.frame_id = "/map";
         
            points.header.stamp = ros::Time::now();
         
            points.ns = "simple_path";
         
            points.action = visualization_msgs::Marker::ADD;
         
            points.id = 0;
         
            points.type = visualization_msgs::Marker::POINTS;
         
            points.scale.x = 0.1;
            points.scale.y = 0.1;
         
            points.color.g = 1.0f;
            points.color.a = 1.0;

            average = 0;
            num = 0;
            firstTime = 1;
            hasStarted = 0;
            pathLength = 0;
   
            // set initialized flag
            initialized_ = true;
        }
        else
        {
            ROS_WARN("This planner has already been initialized, doing nothing.");
        }   
    }

    bool BotsAndUsPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
    	// check if plugin initialized
        if(!initialized_)
        {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
        }


        if(length != 0)
        { 

            if(firstTime)
            {
                startTime = ros::Time::now().toSec();
                firstTime = 0;
            }

            double beginning = ros::Time::now().toSec();
            
            setcurrError();
            
            if(distance < 0.1) //distance to curr Target
            { 
                //INSTEAD OF FOLLOWING ALL POINT, LET'S JUMP X TARGET POINTS AHEAD
                int jump = 20;
                if(count<(length-1))//UNTIL REACHES LAST POINT
                {
                    if((length - 1 - count) < jump+1)
                    { 
                        count = length - 1;
                    }
                    else
                    {
                        count += jump; 
                    }
                    getNext();
                }
                else 
                {
                    lastPt = true;
                    //ROTATE TO MATCH FINAL ORIENTATION
                    stopTime = ros::Time::now().toSec();

                    if(fabs(nError.w) > 5*D2R) //10 deg accuracy
                    {
                        setAngVel(); 
                    }
                    else
                    {
                        stopRobot();
                        ROS_INFO("journey duration: %f", (stopTime-startTime));

                        //path length
                        ROS_INFO("path length: %f", pathLength);

                        //average executation time (for computational cost)
                        ROS_INFO("avrg exec time: %f", average/num);
                        goal_reached_ = true;
                        lastPt = false;
                    }
                }
            }
            else
            {
                if(fabs(nError.w) > 15*D2R)//corrects robot's orientation by keeping an eye on the target
                {
                    setAngVel();
                }
                else
                {
                    setLinearVel();
                }
            }

            double ending = ros::Time::now().toSec();
            average += (ending-beginning);
            num++;
            //ROS_INFO("avrg exec time: %f", average/num);
        }

        cmd_vel = cmd;  

        return true;
    }

    bool BotsAndUsPlannerROS::isGoalReached()
    {
    	// check if plugin initialized
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        // this info comes from compute velocity commands:
        return goal_reached_;
    }

    bool BotsAndUsPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
    {
        // check if plugin initialized
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        //reset next counter
        count = 1; 

        //set plan, length and next goal
        plan = orig_global_plan; 
        length = (plan).size();  
        lastAng = getYaw(plan[length-1].pose.orientation);  //remeber desired target orientation

        getNext(); 

    	goal_reached_ = false;

        return true;

    }

    void BotsAndUsPlannerROS::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr& msg)
    {
        Pos before;
        if(hasStarted)
        {
            before.x = currPt.x;
            before.y = currPt.y;
        }
        currPt.x = msg->pose.pose.position.x;
        currPt.y = msg->pose.pose.position.y;
        currPt.w = getYaw(msg->pose.pose.orientation); 

        setcurrError();

        if(hasStarted)
        {
            pathLength += std::sqrt((currPt.x-before.x)*(currPt.x-before.x) + (currPt.y-before.y)*(currPt.y-before.y));
            //ROS_INFO("%f, %f, %f", stopTime, startTime, pathLength);
        }
        
        hasStarted = 1;
    }

    void BotsAndUsPlannerROS::setcurrError()
    {

        double d;

        nError.x = (next.x - currPt.x);
        nError.y = (next.y - currPt.y);

        if (nError.y == 0 & nError.x == 0)
        {  
            d = next.w;
        }
        else
        {  
            d = std::atan2(nError.y, nError.x);
        }

        if(lastPt)//align with desired orientation
            d = lastAng;
        
        distance = std::sqrt(nError.x*nError.x +nError.y*nError.y);
        nError.w = d - currPt.w;

        // make sure that we chose the smallest angle, so that the robot takes the shortest turn
        if ( nError.w > 180*D2R ) 
        { 
            nError.w -= 360*D2R; 
        }
        if ( nError.w < -180*D2R ) 
        { 
            nError.w += 360*D2R;  
        }

    }

    double BotsAndUsPlannerROS::getYaw(geometry_msgs::Quaternion orientation)
    {

        double q[4];
        q[0]= orientation.x;
        q[1]= orientation.y;
        q[2]= orientation.z;
        q[3]= orientation.w;

        double t3 = +2.0 * (q[3] * q[2] + q[0] * q[1]);
        double t4 = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);  

        return std::atan2(t3, t4);

    }

    void BotsAndUsPlannerROS::setLinearVel()
    {

        // the output speed depends on the distance to the next point. saturate to 0.2 m/s to avoid slippage 
        //this should be done using an acc ramp
        cmd.linear.x= distance;
        if(cmd.linear.x > 0.2)
            cmd.linear.x = 0.2;

        // keeping a small angular speed so that the movement is smooth //note that nError.w is small here 
        cmd.angular.z= 0.5*(nError.w);

        //once again saturate the ang velocity to 0.1 
        if(cmd.angular.z > 0.1)
            cmd.angular.z = 0.1;
        else if(cmd.angular.z < -0.1)
            cmd.angular.z = -0.1;

    }

    void BotsAndUsPlannerROS::setAngVel()
    {
        // the angular velocity depends on the dif between the current heading and the desired
        if (fabs(nError.w) > 30*D2R)
        {
            cmd.angular.z=(nError.w)*0.6;
        }
        else //as approaches the target heading, decrease velocity
        {
            cmd.angular.z=nError.w*0.5;
        }
        cmd.linear.x= 0.0;
        cmd.linear.y= 0.0;
    }

    bool BotsAndUsPlannerROS::stopRobot()
    {
        cmd.linear.x= 0;
        cmd.linear.y= 0;
        cmd.angular.z=0;
        return true;
    }

    void BotsAndUsPlannerROS::getNext()
    { 
        next.x = plan[count].pose.position.x;
        next.y = plan[count].pose.position.y;
        next.w = plan[count].pose.orientation.w;
        visualizePoint(next, true);
    }

    
    void BotsAndUsPlannerROS::visualizePoint(Pos p, bool prune)
    {
        geometry_msgs::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.w;
        if(prune)
            points.points.clear();

        points.points.push_back(pt);         
        path_pub.publish(points);  
    }

    BotsAndUsPlannerROS::~BotsAndUsPlannerROS()
    {
    }
}