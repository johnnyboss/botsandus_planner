# BotsAndUs_Planner
About the botsandus_planner package:
-the planner receives a goal
-segments the path
-per segment, orientates itself to the next target and drives in a straight line
-along the way it's constantly monitoring its orientation to the target and correct it if needed
-when it reaches the last point it orientates itself with the desired orientation
# Launching
roslaunch botsandus_planner turtlebot3_world.launch - launches the gazebo world with no gui
roslaunch botsandus_planner turtlebot3_navigation.launch - launches localization and navigation with the botsandus_planner as local planner

# Compile and unit test
catkin_make && catkin_make run_tests_botsandus_planner_rostest_test_unittest_planner.launch 

# Planner test
rostest botsandus_planner test.launch 
Sends three goals and check that it was able to get to all points with the desired orientation

