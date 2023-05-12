#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include "std_msgs/Bool.h"

#define waiting_for_a_person 0
#define observing_the_person 1
#define rotating_to_the_person 2
#define moving_to_the_person 3
#define interacting_with_the_person 4
#define rotating_to_the_base 5
#define moving_to_the_base 6
#define resetting_orientation 7

#define frequency_expected 25
#define max_base_distance 6
#define close_threshold 0.5 // if less than 50cm, the distance is close
#define person_move_threshold 0.1 // if less than 10cm, a person is not moving
#define rotating_threshold 0.1
#define cycles_no_person_threshold 30

class decision_node
{
private:

    ros::NodeHandle n;

    // communication with datmo_node
    ros::Subscriber sub_person_position;
    bool new_person_position, is_person_moving;
    geometry_msgs::Point person_position;
    geometry_msgs::Point previous_person_position;

    // communication with robot_moving_node
    ros::Subscriber sub_robot_moving;
    bool robot_moving;

    // communication with rotation_action
    ros::Publisher pub_rotation_to_do;
    float rotation_to_person;

    // communication with action_node
    ros::Publisher pub_goal_to_reach;
    float translation_to_person;

    // communication with localization
    ros::Subscriber sub_localization;
    bool new_localization;
    bool init_localization;
    geometry_msgs::Point current_position;
    float current_orientation;

    // distance to base position
    float translation_to_base;
    float prev_translation_to_base;
    // rotation to do toward base
    float rotation_to_base;
    float prev_rotation_to_base;
    //rotation to reset orientation
    float rotation_reset_orientation;
    float prev_rotation_reset_orientation;

    // point message for rotation to base (10001, 10001, rotation_to_base)
    geometry_msgs::Point dir_base_orientation;
    // point message for translation to base (translation_to_base, 0)
    geometry_msgs::Point dir_base_translation;
     // point message for reset orientation (10001, 10001, rotation_reset_orientation)
    geometry_msgs::Point dir_reset_orientation;

    int current_state, previous_state;

    // for how many cycles person does not move
    int frequency;
    // for how many cycles person is not detected
    int cycles_no_person;

    geometry_msgs::Point base_position;
    float base_orientation;
    geometry_msgs::Point origin_position;
    bool state_has_changed;
    // if base position is saved
    bool base_pos_get;

public:

decision_node()
{

    // communication with datmo_node <- in
    sub_person_position = n.subscribe("person_position", 1, &decision_node::person_positionCallback, this);

    // communication with rotation_node -> out
    pub_rotation_to_do = n.advertise<geometry_msgs::Point>("goal_to_rotate", 1); // Use point it will face to

    // communication with action_node -> out
    pub_goal_to_reach = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);     // Preparing a topic to publish the position of the person

    // communication with robot_moving_node <- in
    sub_robot_moving = n.subscribe("robot_moving", 1, &decision_node::robot_movingCallback, this);

    // communication with localization node <- in
    sub_localization = n.subscribe("localization", 1, &decision_node::localizationCallback, this);

    current_state = waiting_for_a_person;
    previous_state = -1;

    new_person_position = false;
    state_has_changed = false;

    // TO DEFINE according to the position of the base/initial position in the map
    base_position.x = 0;
    base_position.y = 0;
    base_orientation = 0;
    base_pos_get = false;

    origin_position.x = 0;
    origin_position.y = 0;
    previous_person_position.x = 0;
    previous_person_position.y = 0;

    is_person_moving = false;
    cycles_no_person = 10000; //infinity for init status

    prev_rotation_to_base = 10000; // infinity
    prev_translation_to_base = 10000; // infinity
    prev_rotation_reset_orientation = 10000; //inf

    //INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will work at 10hz
    while (ros::ok())
    {
        ros::spinOnce();//each callback is called once
        update();
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

//UPDATE: main processing of laser data
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update()
{

    if ( init_localization )
    {

        update_variables();

        // DO NOT FORGET that if robair is too far from its base (ie, its distance to the base is higher than max_base_distance),
        // robair should stop to interact with the moving person and go back to its base
        switch ( current_state )
        {
            case waiting_for_a_person:
                process_waiting_for_a_person();
                break;

            case observing_the_person:
                process_observing_the_person();
                break;

            case rotating_to_the_person:
                process_rotating_to_the_person();
                break;

            case moving_to_the_person:
                process_moving_to_the_person();
                break;

            case interacting_with_the_person:
                process_interacting_with_the_person();
                break;

            case rotating_to_the_base:
                process_rotating_to_the_base();
                break;

            case moving_to_the_base:
                process_moving_to_the_base();
                break;

            case resetting_orientation:
                process_resetting_orientation();
                break;
        }

    new_localization = false;
    new_person_position = false;

    state_has_changed = current_state != previous_state;
    previous_state = current_state;

    }
    else
        ROS_WARN("Initialize localization");

}// update

void update_variables()
{

    if ( new_person_position )
    {
        cycles_no_person = 0;
        translation_to_person = distancePoints(origin_position, person_position);
        
        if ( translation_to_person > 0 )
        {
            rotation_to_person = acos( person_position.x / translation_to_person );
            if ( person_position.y < 0 )
                rotation_to_person *=-1;
        }
        else
            rotation_to_person = 0;

        // if tracked person is moving
        float diff_position = distancePoints(previous_person_position, person_position);
        if (diff_position > person_move_threshold)
            is_person_moving = true;
        else
            is_person_moving = false;

        //save as the previous position
        previous_person_position = person_position;
    }
    else
    {
        cycles_no_person++;
    }

    if ( new_localization )
    {
        if(!base_pos_get)
        {
            base_position = current_position;
            base_orientation = current_orientation;
            base_pos_get = true;
            ROS_INFO("====== Base initialized: (%f, %f, %f)", base_position.x, base_position.y, base_orientation);
        }
        //TODO
        // when we receive a new position(x, y, o) of robair in the map, we update:
        // translation_to_base: the translation that robair has to do to reach its base
        // rotation_to_base: the rotation that robair has to do to reach its base
        // dir_base_orientation: the direction to the base when orientation on unit circle

        //For reset orientation
        rotation_reset_orientation = base_orientation - current_orientation;
        while(rotation_reset_orientation > M_PI)
	        rotation_reset_orientation -= 2*M_PI;
        while(rotation_reset_orientation < -M_PI)
	        rotation_reset_orientation += 2*M_PI;

        //For moving to the base
        translation_to_base = distancePoints(current_position, base_position);

        //For rotating to the base
        float xx = base_position.x - current_position.x;
        float yy = base_position.y - current_position.y;
        float angle_toward_base = acos( xx / translation_to_base );
        if ( yy < 0 )
            angle_toward_base *=-1;

        rotation_to_base = angle_toward_base - current_orientation;
        while(rotation_to_base > M_PI)
	        rotation_to_base -= 2*M_PI;
        while(rotation_to_base < -M_PI)
	        rotation_to_base += 2*M_PI;
    }

}

void reset_initial_state()
{

}

void process_waiting_for_a_person()
{

    if ( state_has_changed )
    {
        ROS_INFO("current_state: waiting_for_a_person");
        // ROS_INFO("press enter to continue");
        // getchar();
    }

    // Processing of the state
    // as soon as we detect a moving person, we switch to the state "observing_the_person"
    if ( new_person_position ) {
        ROS_INFO("[process_waiting_for_a_person]: PERSON DETECTED!");
        current_state = observing_the_person;
    }

}

void process_observing_the_person()
{

    if ( state_has_changed )
    {
        ROS_INFO("current_state: observing_the_person");
        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
        // ROS_INFO("press enter to continue");
        // getchar();
        frequency = 0;
    }

    // Processing of the state
    // Robair only observes and tracks the moving person
    // if the moving person does not move during a while (use frequency), we switch to the state "rotating_to_the_person"
    if ( new_person_position )
    {
        ROS_INFO("person_position: (%f, %f), frequency = %d", person_position.x, person_position.y, frequency);
        if(is_person_moving)
            frequency = 0;
    }
    frequency ++;

    if(frequency >= frequency_expected) 
    {
        ROS_INFO("[process_observing_the_person]: PERSON NOT MOVING!");
        current_state = rotating_to_the_person;
        return;
    }

    // what should robair do if it loses the moving person ?
    if(cycles_no_person >= cycles_no_person_threshold) 
    {
        ROS_INFO("[process_observing_the_person]: PERSON LOST FOR LONG TIME!");
        current_state = waiting_for_a_person;
    }

}

void process_rotating_to_the_person()
{

    if ( state_has_changed )
    {
        ROS_INFO("current_state: rotating_to_the_person");
        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
        //ROS_INFO("press enter to continue");
        //getchar();
        frequency = 0;
    }

    // Processing of the state
    // Robair rotates to be face to the moving person
    // if robair is face to the moving person and the moving person does not move during a while (use frequency), we switch to the state "moving_to_the_person"
    if ( new_person_position )
    {
        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
        pub_rotation_to_do.publish(person_position);
        if(is_person_moving)
            frequency = 0;
    }
    frequency ++;

    if(frequency >= frequency_expected) 
    {
        ROS_INFO("[process_rotating_to_the_person]: PERSON NOT MOVING!");
        current_state = moving_to_the_person;
        return;
    }

    // what should robair do if it loses the moving person ?
    if(cycles_no_person >= cycles_no_person_threshold) 
    {
        ROS_INFO("[process_rotating_to_the_person]: PERSON LOST FOR LONG TIME!");
        current_state = resetting_orientation; //NEED TO ROTATE BACK!
    }
}

void process_moving_to_the_person()
{

    if ( state_has_changed )
    {
        ROS_INFO("current_state: moving_to_the_person");
        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
        //ROS_INFO("press enter to continue");
        //getchar();
        frequency = 0;
    }

    // Processing of the state
    // Robair moves to be close to the moving person
    // if robair is close to the moving person and the moving person does not move during a while (use frequency), we switch to the state "interacting_with_the_person"
    if ( new_person_position )
    {
        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
        if( (!robot_moving) && (translation_to_person > close_threshold) )
            pub_goal_to_reach.publish(person_position);
    }

    if( (!robot_moving) && (translation_to_person <= close_threshold) )
    {
        ROS_INFO("[process_moving_to_the_person]: REACH PERSON TARGET!");
        if( new_person_position && is_person_moving )
            frequency = 0;
        else
            frequency ++;

        if(frequency >= frequency_expected) 
        {
            ROS_INFO("[process_moving_to_the_person]: PERSON NOT MOVING!");
            current_state = interacting_with_the_person;
            return;
        }
    }

    // what should robair do if it loses the moving person ?
    if(cycles_no_person >= cycles_no_person_threshold) 
    {
        ROS_INFO("[process_moving_to_the_person]: PERSON LOST FOR LONG TIME!");
        current_state = rotating_to_the_base; //NEED TO MOVE BACK!
    }
}

void process_interacting_with_the_person()
{

    if ( state_has_changed )
    {
        ROS_INFO("current_state: interacting_with_the_person");
        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
        //ROS_INFO("press enter to continue");
        //getchar();
        frequency = 0;
    }

    // Processing of the state
    // Robair does not move and interacts with the moving person until the moving person goes away from robair
    // if the person goes away from robair, after a while (use frequency), we switch to the state "rotating_to_the_base"
    if ( new_person_position )
    {
        ROS_INFO("person_position: (%f, %f)", person_position.x, person_position.y);
        if(translation_to_person <= close_threshold)
            frequency = 0;
    }
    frequency ++;

    if(frequency >= frequency_expected) 
    {
        ROS_INFO("[process_interacting_with_the_person]: PERSON LEAVE!");
        current_state = rotating_to_the_base;
        return;
    }

    // what should robair do if it loses the moving person ?
    if(cycles_no_person >= cycles_no_person_threshold) 
    {
        ROS_INFO("[process_interacting_with_the_person]: PERSON LOST FOR LONG TIME!");
        current_state = rotating_to_the_base; //NEED TO MOVE BACK!
    }
}

void process_rotating_to_the_base()
{

    if ( state_has_changed )
    {
        ROS_INFO("current_state: rotating_to_the_base");
        ROS_INFO("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation*180/M_PI);
        //ROS_INFO("press enter to continue");
        //getchar();
        frequency = 0;
        prev_rotation_to_base = 10000;//inf
    }

    // Processing of the state
    // Robair rotates to be face to its base
    // if robair is face to its base and does not move, after a while (use frequency), we switch to the state "moving_to_the_base"
    //!!!!! detect new localization is FALSE. frequency of new localization is low!
    if ( !robot_moving )
    {
        ROS_INFO("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation*180/M_PI);
        ROS_INFO("local base position: %f (%f, %f)", rotation_to_base, dir_base_orientation.x, dir_base_orientation.y);
        // if remaining rotation < previous remaining rotation, means rotation is in progress(not too much), continue
        if(abs(rotation_to_base) < abs(prev_rotation_to_base))
        {
            dir_base_orientation.x = dir_base_orientation.y = 10001;
            dir_base_orientation.z = rotation_to_base;
            pub_rotation_to_do.publish(dir_base_orientation);

            prev_rotation_to_base = rotation_to_base;
        }
        
        frequency ++;
    } else {
        frequency = 0;
    }
    
    //robot stop for a long time
    if(frequency >= frequency_expected) 
    {
        ROS_INFO("[process_rotating_to_the_base]: ROBOT NOT MOVING!");
        current_state = moving_to_the_base;
        return;
    }

}

void process_moving_to_the_base()
{

    if ( state_has_changed )
    {
        ROS_INFO("current_state: moving_to_the_base");
        ROS_INFO("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation*180/M_PI);
        //ROS_INFO("press enter to continue");
        //getchar();
        frequency = 0;
        prev_translation_to_base = 10000;// inf
    }

    // Processing of the state
    // Robair moves to its base
    // if robair is close to its base and does not move, after a while (use frequency), we switch to the state "resetting_orientation"
    if ( !robot_moving )
    {
        ROS_INFO("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation*180/M_PI);
        // if remaining translation < previous remaining translation, and remaining translation > threshold, 
        // means translation is in progress(not too much), continue
        if((abs(translation_to_base) < abs(prev_translation_to_base)) && (translation_to_base > close_threshold)) {
            dir_base_translation.x = translation_to_base;
            dir_base_translation.y = 0;
            pub_goal_to_reach.publish(dir_base_translation);

            prev_translation_to_base = translation_to_base;
        }
        // if remaining translation is not decrease, but distance is still big, means rotation has error, redo it.
        else if(translation_to_base > close_threshold)
        {
            ROS_INFO("[process_moving_to_the_base]: ROBOT NOT MOVING!");
            current_state = rotating_to_the_base;
            return;
        }
        // if distance is already small, not move again

        frequency ++;
    } 
    else
    {
        frequency = 0;
    }

    //robot stop for long time
    if(frequency >= frequency_expected) 
    {
        ROS_INFO("[process_moving_to_the_base]: ROBOT NOT MOVING!");
        current_state = resetting_orientation;
        return;
    }
}

void process_resetting_orientation()
{

    if ( state_has_changed )
    {
        ROS_INFO("current_state: resetting_orientation");
        ROS_INFO("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation*180/M_PI);
        //ROS_INFO("press enter to continue");
        //getchar();
        frequency = 0;
        prev_rotation_reset_orientation = 10000;//inf
    }

    // Processing of the state
    // Robair rotates to its initial orientation
    // if robair is close to its initial orientation and does not move, after a while (use frequency), we switch to the state "waiting_for_a_person"

    if ( !robot_moving )
    {
        ROS_INFO("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation*180/M_PI);
        //need a threshold?
        if(abs(rotation_reset_orientation) < abs(prev_rotation_reset_orientation)) {
            dir_reset_orientation.x = dir_reset_orientation.y = 10001;
            dir_reset_orientation.z = rotation_reset_orientation;
            pub_rotation_to_do.publish(dir_reset_orientation);

            prev_rotation_reset_orientation = rotation_reset_orientation;
        }
        frequency ++;
    } 
    else 
    {
        frequency = 0;
    }

    if(frequency >= frequency_expected) 
    {
        ROS_INFO("[process_resetting_orientation]: ROBOT NOT MOVING!");
        current_state = waiting_for_a_person;
        return;
    }

}

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void person_positionCallback(const geometry_msgs::Point::ConstPtr& g)
{
// process the goal received from moving_persons detector

    new_person_position = true;
    person_position.x = g->x;
    person_position.y = g->y;

}

void robot_movingCallback(const std_msgs::Bool::ConstPtr& state)
{

    robot_moving = state->data;
    if(robot_moving)
        ROS_INFO("Robot Moving!");
}//robot_movingCallback

void localizationCallback(const geometry_msgs::Point::ConstPtr& l)
{
// process the localization received from my localization
    new_localization = true;
    init_localization = true;
    current_position = *l;
    current_orientation = l->z;
    ROS_INFO("New localization!");
}

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

};

int main(int argc, char **argv)
{

    ROS_INFO("(decision_node) waiting for a /person_position");
    ros::init(argc, argv, "decision_node");

    decision_node bsObject;

    ros::spin();

    return 0;

}
