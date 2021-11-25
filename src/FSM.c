/****
 This file contains logic for the FSMs
****/

#ifndef _FSM_C
#define _FSM_C

#include "roboAI.h"
#include "PID.c"


int get_new_state_Chase(struct RoboAI *ai, int old_state);
int get_new_state_Penalty (struct RoboAI *ai, int old_state);


// TODO: move to header
#define STOP_BOT 106
#define GOOD_BALL_DIST 100
#define ANGLE_RESET 0.5 //this is for Chase FSM


#define KICK_SPEED 80



int get_new_state_Chase(struct RoboAI *ai, int old_state){
    // todo: redundant state
    // Find event based on the info in RoboAI ai struct (maybe also old_state)
    // call when in penalty mode
    double ballPos[2] = {ai->st.old_bcx, ai->st.old_bcy};
    double botHeading[2] = {ai->st.sdx, ai->st.sdy};
    double botPos[2] = {ai->st.old_scx, ai->st.old_scy};
    double goalPos[2];
    // side=0 implies the robot's own side is the left side
    // side=1 implies the robot's own side is the right side
    goalPos[0] = (ai->st.side == 0) ? IM_SIZE_X : 0; //gets the enemy goal
    goalPos[1] = IM_SIZE_Y / 2;

    static double targetP[2] = {-1, -1};

    
      double goal_to_ball[2] = {ballPos[0]- goalPos[0], ballPos[1] - goalPos[1] };
      double magnitude = magnitude_vector(goal_to_ball);
      // make it vector of magnitude GOOD_BALL_DIST in the balls direction
      goal_to_ball[0] = GOOD_BALL_DIST * goal_to_ball[0] / magnitude;
      goal_to_ball[1] = GOOD_BALL_DIST * goal_to_ball[1] / magnitude;
      
      // target = ball + offset in ball's direction
      targetP[0] = ballPos[0] + goal_to_ball[0];
      targetP[1] = ballPos[1] + goal_to_ball[1];


    double bot_to_ball_vector[2] = {ballPos[0] - botPos[0], ballPos[1] - botPos[1]};
    double bot_to_ball_dist = magnitude_vector(bot_to_ball_vector);
    
    double bot_to_targetP_vector[2] = {targetP[0] - ai->st.old_scx, targetP[1] - ai->st.old_scy};
    double bot_to_targetP_dist = magnitude_vector(bot_to_targetP_vector);


    double bot_to_targetP_angle = getAngle_vector(bot_to_targetP_vector, botHeading);
    // acos(dottie_vector(bot_to_targetP_vector, botHeading) / sqrt(dottie_vector(bot_to_targetP_vector, bot_to_targetP_vector)* dottie_vector(botHeading, botHeading)));

    double bot_to_ball_angle = getAngle_vector(bot_to_ball_vector, botHeading); 
    // acos(dottie_vector(bot_to_ball_vector, botHeading) / sqrt(dottie_vector(bot_to_ball_vector, bot_to_ball_vector)* dottie_vector(botHeading, botHeading)));

        // printf("theta: %f\n", theta);

    switch (old_state){
        case 201: {
            
            //action

            if( fabs(bot_to_targetP_angle) > 0.1){ //more than 60 deg
                return 202;
            }else{
                return 203;
            }
        }
        case 202: {

            //rotation PID
            int done_turning = turn_to_target(botHeading[0], botHeading[1], bot_to_targetP_vector[0], bot_to_targetP_vector[1]);

            // if( fabs(bot_to_targetP_angle) < 0.3){ // less than 17 deg
            if(done_turning){
                BT_all_stop(1);
                return 203;
            }else{
                return 202;
            } 
        }
        case 203: {

            //Run PID to targetP - should be able to turn a bit as well
            

        double distance_err, lateral_err;
        finding_errors(ballPos, targetP, goalPos, &distance_err, &lateral_err); // distance error is always positive
        double drive_straight_output = drive_straight_to_target_PID(distance_err); 
        BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, drive_straight_output);

            if(bot_to_targetP_dist < 100){ //next state
                BT_all_stop(0);
                return 204;
            }else if ( fabs(bot_to_targetP_angle) > ANGLE_RESET){ // more than 60 deg
                BT_all_stop(0);
                return 202;
            }else{
                return 203;
            }
            
        }
        case 204: {

            //rotation PID
            int done_turning = turn_to_target(botHeading[0], botHeading[1], bot_to_ball_vector[0], bot_to_ball_vector[1]);

            // if( fabs(bot_to_targetP_angle) < 0.3){ // less than 17 deg
            if(done_turning){
                BT_all_stop(1);
                return 205;
            }else{
                return 204;
            } 

            
        }
        case 205: {
            //Run PID to targetP - should be able to turn a bit as well
            BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, 60);
            BT_motor_port_start(MOTOR_C, -100);
            printf("bot_to_ball_dist %f \n",bot_to_ball_dist);

            if(bot_to_ball_dist > 200){ //next state
                BT_all_stop(0);
                return 206;
            }else if ( fabs(bot_to_ball_angle) > ANGLE_RESET){ // more than 60 deg
                BT_all_stop(0);
                return 201; // reset entire thing;
            }else{
                return 205;
            }

        }
        case 206: {
            BT_all_stop(0);

            if(bot_to_ball_dist > 200){ 
                BT_all_stop(0);
                return 205; //start pid again
            }else{
                return 206;
            }

            return 206; //stay in 206
        }

        return 201;
    }

    return 201;
}


int get_new_state_Penalty (struct RoboAI *ai, int old_state){
    // call when in penalty mode
    double ballPos[2] = {ai->st.old_bcx, ai->st.old_bcy};
    double botHeading[2] = {ai->st.sdx, ai->st.sdy};
    double botPos[2] = {ai->st.old_scx, ai->st.old_scy};
    double goalPos[2];
    // side=0 implies the robot's own side is the left side
    // side=1 implies the robot's own side is the right side
    goalPos[0] = (ai->st.side == 0) ? IM_SIZE_X : 0; //gets the enemy goal
    goalPos[1] = IM_SIZE_Y / 2;

    static double targetP[2] = {-1, -1};
    double goal_to_ball[2] = {ballPos[0]- goalPos[0], ballPos[1] - goalPos[1] };
    double magnitude = magnitude_vector(goal_to_ball);

    if (targetP[0] == -1 && targetP[1] == -1 ) { // happens once only Q: what if first ball reading is wrong?
    // lock position only once robot close to ball?
      // make it vector of magnitude GOOD_BALL_DIST in the balls direction
      goal_to_ball[0] = GOOD_BALL_DIST * goal_to_ball[0] / magnitude;
      goal_to_ball[1] = GOOD_BALL_DIST * goal_to_ball[1] / magnitude;
      
      // target = ball + offset in ball's direction
      targetP[0] = ballPos[0] + goal_to_ball[0];
      targetP[1] = ballPos[1] + goal_to_ball[1];
    }


    double bot_to_ball_vector[2] = {ballPos[0] - botPos[0], ballPos[1] - botPos[1]};
    double bot_to_ball_dist = magnitude_vector(bot_to_ball_vector);
    
    double bot_to_targetP_vector[2] = {targetP[0] - ai->st.old_scx, targetP[1] - ai->st.old_scy};
    double bot_to_targetP_dist = magnitude_vector(bot_to_targetP_vector);


    double bot_to_targetP_angle = getAngle_vector(bot_to_targetP_vector, botHeading);
    double bot_to_ball_angle = getAngle_vector(bot_to_ball_vector, botHeading); 

    switch (old_state)
    {
    case 101: { // Penalty
        return 102;

    }
    case 102: {// is it facing the target_p?
        //rotation PID
        int done_turning = turn_to_target(botHeading[0], botHeading[1], bot_to_targetP_vector[0], bot_to_targetP_vector[1]);

         // if( fabs(bot_to_targetP_angle) < 0.3){ // less than 17 deg
        if(done_turning){
            BT_all_stop(1);
            return 103;
        }else{
            return 102;
        } 
    }
    case 103: {
        // printf("bot_to_targetP_dist %f\n", bot_to_targetP_dist);
            //Run PID to targetP - should be able to turn a bit as well
            // BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, 25);

        double distance_err, lateral_err;
        finding_errors(ballPos, targetP, goalPos, &distance_err, &lateral_err); // distance error is always positive
        double drive_straight_output = drive_straight_to_target_PID(distance_err); 
        BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, drive_straight_output);

            if(bot_to_targetP_dist < 100){ //next state
                BT_all_stop(0);
                return 104;
            }else{
                return 103;
            }
        break;
    }
    case 104: {
        //rotation PID
        int done_turning = turn_to_target(botHeading[0], botHeading[1], bot_to_ball_vector[0], bot_to_ball_vector[1]);

        // if( fabs(bot_to_targetP_angle) < 0.3){ // less than 17 deg
        if(done_turning){
            BT_all_stop(1);
            return 105;
        }else{
            return 104;
        } 
        return 104;
        break;
    }
    case 105: {
        BT_motor_port_start(LEFT_MOTOR | RIGHT_MOTOR, KICK_SPEED);
        BT_motor_port_start(MOTOR_C, -100);
        if (bot_to_ball_dist > 200){
            BT_all_stop(1);
            return 106;
        }else{
            return 105;
        }

        return 105;
        break;
    }
    case 106: { //done
        BT_all_stop(0);
        break;
    }
    case 107: {
        BT_all_stop(0);
        return 107; //stay in 107
        break;
    }
     default: {
         return 101;
     }
    }

    return 101;
}


#endif