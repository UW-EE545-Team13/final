# Welcome to EE P 545 Final Project Setup

## Pre-Setup (this assumes you already have the catkin_ws installed/setup)

1. Clone the final project code into the catkin_ws `src` folder:
```
$ cd /home/robot/catkin_ws/src
$ git clone https://github.com/UW-EE545-Team13/final.git
```
2. run catkin make
```
$ cd /home/robot/catkin_ws
$ catkin_make
```

## Now you are ready to try it on either the simulator or the real robot.

### Simulator
1. Launch mushr_sim
    ```
    $ roslaunch mushr_sim teleop.launch
    ```

2. Launch rviz in a new terminal window
    ```
    $ rviz
    ```	

3. Launch particle filter in a new terminal window
    ```
    $ roslaunch final ParticleFilter.launch
    ```
    Note: it takes some time to launch, wait a couple mins till you see ‘Initialization complete’ on the terminal before you move to the next step.

4. Launch line follower in a new terminal window
    ```
    $ roslaunch final line_follower.launch
    ```

5. Launch planner node in a new terminal window

	For final race purposes run:
    ```
	$ roslaunch final PlannerNode.launch final_race:=True
    ```
	For general purposes run:
    ```
	$ roslaunch final PlannerNode.launch final_race:=False
    ```
6.  Add topics on RVIZ

    Click on the Add button at the bottom left corner and click `By topic`, add the following topics:
    ```
	/graph (MarkerArray)
    /planner_node/car_plan (PoseArray)
    /pf/viz/inferred_pose (Pose)
    ```
    feel free to add other topics if you wish

7. Get the planned path

    For final race purposes the planner node will only return the plan for the final race, for general usage please see step 5 above.

    Set the start of the plan by using the `2D Pose Estimate` button on the nav bar, and set the end of the plan by using the `2D Nav Goal` button next to the `2D Pose Estimate` button. Then wait till the plan shows up on the screen.

8.  Make the car follow the path

    After step 7 the car should already be at the starting position of the plan, now go to the terminal window where you launched the line_follower node. 
    
    On that terminal window you should see ‘Press Enter to when plan available…’, verify the plan is ready then hit `Enter` on your keyboard. Now just watch the car do the magic :) it should start moving along the plan.

### Real Robot (Mushr Car)

1. Set up the Mushr Car by following car_setup.pdf
2. Tune the car by following this: https://mushr.io/tutorials/tuning/

    Note:This is really important, otherwise the particle filter will give wrong locations

3. If the car has any problems please read this [guid]|(https://mushr.io/hardware/build_instructions/) or reach out to TA.
4. Make sure the car is all setup, now connect to the car’s wifi and ssh into the car.
    ```
	$ ssh robot@10.42.0.1
    ```

    The password is `prl_robot`
5. Launch mushr_base

    Open a new terminal window and ssh into the robot and then run:
    ```
	$ roslaunch mushr_base teleop.launch
    ```
6. Launch rviz

    For rviz you don’t need to ssh into the robot, just open a new terminal window and run:
    ```
	$rosrun rviz rviz
    ```
7. Launch particle filter

	Open a new terminal window and ssh into the robot and then run:
    ```
	$ roslaunch final ParticleFilter.launch
    ```
	Note: This could take a while to finish, wait till you see ‘Initialization complete’ on the terminal before moving to the next step.
8. Launch line follower

	Open a new terminal window and ssh into the robot and then run:
    ```
	$ roslaunch final line_follower.launch
    ```
9. Launch planner node

	For planner node you don’t need to ssh into the robot, this can run on your machine.Open a new terminal window.

	For final race purposes, run:
    ```
	$ roslaunch final PlannerNode.launch final_race:=True
    ```
	For general purposes, run:
    ```
	$ roslaunch final PlannerNode.launch final_race:=False
    ```
	For final race purposes the planner node will only return the plan for the final race

10. Make the plan

    Wait till you see `[Planner Node] Ready to plan` on the PlannerNode ternimal window then open RVIZ and click the `2D Pose Estimate` button on the nav bar, and click and drag on the map according to where the robot actually is in the map, and then use the `2D Nav Goal` button on the nav bar to set the goal position. Wait till the plan shows up on the screen.

11. Run the robot

	Go back to the terminal window where the line_follower node is running, on that terminal window you should see ‘Press Enter to when plan available…’, verify the plan is ready then hold down the ‘R1’ button on the joystick and then hit ‘Enter’ on your keyboard. Now watch the car do the magic :) 


