# README #
This repo contains the solution of the Assigment 2, for the Group 05.

## Introduction
In order to **solve** the assigment we managed to create 6 **packages** :

* `detection` : is the package used for detection tasks.

* `detection_msgs` : contains the messages, services and actions used by `detection` package.

* `manipulation` : is the package used for manipulation purposes i.e. controls the movement of the arm of the robot .

* `manipulation_msgs` contains the messages, services and actions used by `manipulation` package.

* `navigation_automatic_ros` : is the package used for navigation purposes i.e. controls the movement of the robot.

* `solution` : is the package that manage the execution of the overall nodes for solving the assigment.

## Group Members
| Surname       | Name          | ID            |
| ------------- | ------------- | ------------- |
| Barusco       | Manuel        | 2053083       |
| Caldivezzi    | Francesco	    | 2037893       |
| Rampon        | Riccardo      | 2052416       |

## Execution Instructions
In order to execute the code you need to :

* open a terminal and build the workspace : `catkin build`.

* open a terminal and execute : `roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full_tables`.

* open a terminal and execute : `roslaunch tiago_iaslab_simulation apriltag.launch`.

* open a terminal and execute : `roslaunch tiago_iaslab_simulation navigation.launch`.

For executing solution no-extra point :

* open a terminal and execute : `roslaunch solution no_extra.launch`.

For executing solution extra point :

* open a terminal and execute : `roslaunch solution extra.launch`.