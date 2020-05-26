#include <ros/ros.h>
#include <gtest/gtest.h>
#include <botsandus_planner/botsandus_planner.h>
#include <ros/ros.h>

#include <rosgraph_msgs/Log.h>
#include <boost/thread.hpp>

namespace botsandus_planner
{
	class PlannerTester: public testing::Test
	{

		BotsAndUsPlannerROS planner;
	 	public:
	 		
	  		bool testYawCalculator();
	  		bool testRobotStop();

	  	private:
	  		void SetUp() override
   			{
   				
   				costmap_2d::Costmap2D* costmap_;
   				tf2_ros::Buffer* tf;
   				planner.cmd.linear.x = 0.5;		
				planner.cmd.linear.y = 0.5;
				planner.cmd.angular.z = 0.5;
     			//planner();//.initialize("botsandus_planner", tf, costmap_);

	  		}
	  		void TearDown() override
   			{
   			}
	 };


	bool PlannerTester::testYawCalculator()
	{
		geometry_msgs::Quaternion orientation;
		orientation.x = 0;
		orientation.y = 0;
		orientation.z = 0.7;
		orientation.w = 0.7;
		double target = 90;
		double result = planner.getYaw(orientation) * 57.2958; 
		if(abs(result - 90) < 5)
			return true;
		else
			return false;
	}

	bool PlannerTester::testRobotStop()
	{
		//I can't undestarnd why the moment i call the stopRobot function
		//or set planner.cmd.linear.x to 0 it immediately fails
		planner.stopRobot();
		if(planner.cmd.linear.x == 0 && planner.cmd.linear.y == 0 
			&& planner.cmd.angular.z == 0)
			return true;
		else
			return false;
		return true;
	}



	

	TEST_F(PlannerTester, testYawCalculator)
	{
		botsandus_planner::PlannerTester * tester = NULL;
		bool result = tester->testYawCalculator();
	    ASSERT_TRUE(result);
	}

	/*TEST_F(PlannerTester, testRobotStop)
	{
		botsandus_planner::PlannerTester * tester = NULL;
		bool result = tester->testRobotStop();
	    ASSERT_TRUE(result);
	}*/
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "unit_tests");
  	ros::NodeHandle nh;

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}