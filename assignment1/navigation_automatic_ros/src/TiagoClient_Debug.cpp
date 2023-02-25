// Node that reads-in command line arguments for a x,y goal position and sends it to move_base via topic message

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class TiagoClient{
    private: 
        MoveBaseClient client;
            
        /** callback for action done
         * @param &state final state 
         * @param &result_ptr Boost Pointer to the final result of the move action
        */
        void doneCb(const actionlib::SimpleClientGoalState &state,const move_base_msgs::MoveBaseResultConstPtr &result_ptr)
        {
            if(state==actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("ROBOT HAS FINISHED: The Robot has arrived to the final pose "); //successed
            }
            else
            {
                ROS_WARN("ROBOT HAS FAILED: The robot has failed to approch the final pose.");//failed
                if(state==actionlib::SimpleClientGoalState::ABORTED)//aborted
                    ROS_INFO("The Robot aborted the navigation, maybe is stucked");
                else if(state==actionlib::SimpleClientGoalState::PREEMPTED)//preempted
                    ROS_INFO("The Navigation has been preempted.");
                else if(state==actionlib::SimpleClientGoalState::REJECTED)//rejected
                    ROS_INFO("The Robot aborted the navigation, maybe the final pose is unreachable");
            }      
        }

        //callback for when robot starts the navigation
        void activeCb()
        {
            ROS_INFO("ROBOT IS STARTING: The Robot starts the navigation to the defined pose");
        }

        /** callback for feedback messages
         * @param &feedback_ptr pointer to the MoveBaseFeedback message
        */
        void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback_ptr)
        {
            ROS_INFO("ROBOT IS MOVING: Coordinates (%.2f,%.2f)",feedback_ptr->base_position.pose.position.x,feedback_ptr->base_position.pose.position.y);

        }  

    public: 

        /* default constructor */
        TiagoClient(): client("move_base", true){
            ROS_INFO("Tiago Client started"); 
        }
    
        
        /* 
        @param x coordinate x of the final pose (w.r.t. map ref. frame)
        @param y coordinate y of the final pose (w.r.t. map ref. frame)
        @param orZ orientation in the z axis of the final pose (w.r.t. map ref. frame)
        @return -1 if the goal posed is not valid, 0 if is valid
        */
       int sendGoal(float x, float y, float orZ){
            // Wait for the action server to come up so that we can begin processing goals.
            ROS_INFO("Waiting for the move_base action server to come up");
            client.waitForServer(); 
            
            move_base_msgs::MoveBaseGoal goal;

            //set the goal position
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = x;
            goal.target_pose.pose.position.y = y;
            goal.target_pose.pose.orientation.z = orZ;

            //all is wrt. world (map) ref frame
            goal.target_pose.pose.orientation.w = 1.0;

            ROS_INFO("Sending goal");
            //client.sendGoal(goal, , nullptr, nullptr);
            client.sendGoal(goal, boost::bind(&TiagoClient::doneCb , this, _1, _2), 
                                  boost::bind(&TiagoClient::activeCb , this ), 
                                  boost::bind(&TiagoClient::feedbackCb , this, _1)); 
            
       }


}; 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Ciao");

    //read the command lines argument 
    if (argc != 4) {
        ROS_INFO("Please provide x, y and orientazione wrt. z axis of the goal position!");
        return 1;  
    } 

    float x,y, orZ; 

    char* endPtr; 
    x = strtof(argv[1], &endPtr);
    y = strtof(argv[2], &endPtr);
    orZ = strtof(argv[3], &endPtr);

    //construct the client
    TiagoClient client; 

    //send the goal
    client.sendGoal(x,y,orZ); 

    ros::spin();
    return 0;
}