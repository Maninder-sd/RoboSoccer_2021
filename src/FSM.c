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
#define GOOD_BALL_DIST 150
#define ANGLE_RESET 0.5 //this is for Chase FSM

#define FORWARD_MAX_SPEED 30
#define KICK_SPEED 80

#define MAX_SPEED 100


int get_new_state_soccer(struct RoboAI *ai, int old_state) {
    double angle_bound=0.70; 

    static double target_pos[2];
    static double opponent[2];

    if (ai->st.opp) {
        opponent[0] = ai->st.old_ocx;
        opponent[1] = ai->st.old_ocy;
    }


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


    int is_bot_off_side = (ballPos[0] - botPos[0] <= 0);
    int is_opponent_offside = (ballPos[0] - opponent[0] > 0);

    //flip offside booleans based on home goal position
    if (ai->st.side) {
        is_bot_off_side = !is_bot_off_side;
        is_opponent_offside = !is_opponent_offside;
    }

    double opponent_to_ball[2] = {ballPos[0] - opponent[0], ballPos[1] - opponent[1]};

    double bot_to_ball_vector[2] = {ballPos[0] - botPos[0], ballPos[1] - botPos[1]};
    double bot_to_ball_dist = magnitude_vector(bot_to_ball_vector);
    
    // acos(dottie_vector(bot_to_targetP_vector, botHeading) / sqrt(dottie_vector(bot_to_targetP_vector, bot_to_targetP_vector)* dottie_vector(botHeading, botHeading)));

    double bot_to_ball_angle = getAngle_vector(bot_to_ball_vector, botHeading); 

    int is_opponent_closer = (magnitude_vector(opponent_to_ball) <= bot_to_ball_dist);
    int ball_in_middle = (ballPos[1] > IM_SIZE_Y * 0.25 && ballPos[1] < IM_SIZE_Y * 0.75);

    double angle_thresh = 0.1;
    double own_goal_x = fabs(IM_SIZE_X - goalPos[0]);
    int bot_above_ball = botPos[1] - ballPos[1] > 0;

    printf("bot offside : %d, opponent_closer : %d, opp offside : %d\n", is_bot_off_side, is_opponent_closer, is_opponent_offside);

    //attack mode
    if (!is_bot_off_side && (!is_opponent_closer || is_opponent_offside)) {
        printf("attack mode\n");
        target_pos[0] = targetP[0]; target_pos[1] = targetP[1];
    } 

    //defense mode
    else if (!is_bot_off_side && is_opponent_closer) {
        printf("defense mode\n");
        target_pos[0] = ballPos[0] - own_goal_x; target_pos[1] = ballPos[1] - goalPos[1];

        double mg = magnitude_vector(target_pos);
        target_pos[0] = ballPos[0] + 5*GOOD_BALL_DIST*target_pos[0] / mg;
        target_pos[1] = ballPos[1] + 5*GOOD_BALL_DIST*target_pos[1] / mg;
        
    }

    //offside deflect down
    else if (is_bot_off_side && ball_in_middle && bot_above_ball) {
        printf("offside down\n");
        double corner_target[2] = {own_goal_x, 10};
        target_pos[0] = corner_target[0] - ballPos[0]; target_pos[1] = corner_target[1] - ballPos[1];

        double mg = magnitude_vector(target_pos);
        target_pos[0] = ballPos[0] + GOOD_BALL_DIST*target_pos[0] / mg;
        target_pos[1] = ballPos[1] + GOOD_BALL_DIST*target_pos[1] / mg;

    }

    //offside deflect up
    else if (is_bot_off_side && ball_in_middle && !bot_above_ball) {
        printf("offside up\n");
        double corner_target[2] = {own_goal_x, 10};
        target_pos[0] = corner_target[0] - ballPos[0]; target_pos[1] = corner_target[1] - ballPos[1];

        double mg = magnitude_vector(target_pos);
        target_pos[0] = ballPos[0] + GOOD_BALL_DIST*target_pos[0] / mg;
        target_pos[1] = ballPos[1] + GOOD_BALL_DIST*target_pos[1] / mg;

    }

    //offside flee
    else {
        printf("offside retreat");
        target_pos[0] = own_goal_x - 20; target_pos[1] = goalPos[1];
    };

    double bot_to_targetP_vector[2] = {target_pos[0] - ai->st.old_scx, target_pos[1] - ai->st.old_scy};
    double bot_to_targetP_dist = magnitude_vector(bot_to_targetP_vector);


    double bot_to_targetP_angle = getAngle_vector(bot_to_targetP_vector, botHeading);

    switch (old_state) {
              case 1: {
            
            //action

            if( fabs(bot_to_targetP_angle) > angle_bound){ //more than 60 deg
                return 2;
            }else{
                return 3;
            }
        }
        case 2: {

            //rotation PID
            int done_turning = turn_to_target(botHeading[0], botHeading[1], bot_to_targetP_vector[0], bot_to_targetP_vector[1]);

            if( fabs(bot_to_targetP_angle) < 0.3){ // less than 17 deg
            // if(done_turning){
                BT_all_stop(1);
                return 3;
            }else{
                return 2;
            } 
        }
        case 3: {

            //Run PID to targetP - should be able to turn a bit as well
            BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, FORWARD_MAX_SPEED);

            if(bot_to_targetP_dist < 100){ //next state
                BT_all_stop(0);
                return 4;
            }else if ( fabs(bot_to_targetP_angle) > angle_bound){ // more than 60 deg
                BT_all_stop(0);
                return 2;
            }else{
                return 3;
            }
        }
        case 4: {
            //rotation PID
            int done_turning = turn_to_target(botHeading[0], botHeading[1], bot_to_ball_vector[0], bot_to_ball_vector[1]);

            // if( fabs(bot_to_targetP_angle) < 0.3){ // less than 17 deg
            if(done_turning){
                BT_all_stop(1);
                return 5;
            }else{
                return 4;
            } 

            
        }
        case 5: {
            //Run PID to targetP - should be able to turn a bit as well
            BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, 60);
            BT_motor_port_start(MOTOR_C, -100);

            if(bot_to_ball_dist > 200){ //next state
                BT_all_stop(0);
                return 6;
            }else if ( fabs(bot_to_ball_angle) > 1){ // more than 60 deg
                BT_all_stop(0);
                return 1; // reset entire thing;
            }else{
                return 5;
            }

        }
        case 6: {
            BT_all_stop(0);

            if(bot_to_ball_dist > 200){ 
                BT_all_stop(0);
                return 5; //start pid again
            }else{
                return 6;
            }

            return 6; //stay in 206
        }


    }

    return 1;
}



int get_new_state_Chase(struct RoboAI *ai, int old_state){
    // todo: redundant state
    // Find event based on the info in RoboAI ai struct (maybe also old_state)
    // call when in penalty mode
    double angle_bound=0.70; 

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

            if( fabs(bot_to_targetP_angle) > angle_bound){ //more than 60 deg
                return 202;
            }else{
                return 203;
            }
        }
        case 202: {

            //rotation PID
            int done_turning = turn_to_target(botHeading[0], botHeading[1], bot_to_targetP_vector[0], bot_to_targetP_vector[1]);

            if( fabs(bot_to_targetP_angle) < 0.3){ // less than 17 deg
            // if(done_turning){
                BT_all_stop(1);
                return 203;
            }else{
                return 202;
            } 
        }
        case 203: {

            //Run PID to targetP - should be able to turn a bit as well
            

        double distance_err, lateral_err;

        // finding_errors(ballPos, targetP, goalPos, &distance_err, &lateral_err); // distance error is always positive
        // double drive_straight_output = drive_straight_to_target_PID(distance_err); 
        // BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, 50*drive_straight_output);

        //  BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, FORWARD_MAX_SPEED);


         simple_straight_to_target_PID(bot_to_targetP_dist,  bot_to_targetP_angle);  // Maninder- I will test this later

            if(bot_to_targetP_dist < 100){ //next state
                BT_all_stop(0);
                return 204;
            }else if ( fabs(bot_to_targetP_angle) > angle_bound){ // more than 40 deg
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
                BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, 20);
                sleep(1);
                return 205;
            }else if ( bot_to_targetP_dist >100 ){ // more than 60 deg
                BT_all_stop(0);
                return 201; // reset entire thing;
            }else{
                return 204;
            } 

            
        }
        case 205: {
            // just go straight and score
            BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, 60);
            BT_motor_port_start(MOTOR_C, -100);
            printf("bot_to_ball_dist %f \n",bot_to_ball_dist);

            if(bot_to_ball_dist > 200){ //next state
                BT_all_stop(0);
                return 206;
            }else if ( fabs(bot_to_ball_angle) > angle_bound){ // more than 60 deg
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
                return 201; //start pid again
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

    static double targetPos[2] = {-1, -1};

    // printf("ballPos[0]: %f, ballPos[1]: %f\n", ballPos[0], ballPos[1]);
    // printf("targetPos[0]: %f, targetPos[1]: %f\n", targetPos[0], targetPos[1]);


    double bot_to_ball_vector[2] = {ballPos[0] - botPos[0], ballPos[1] - botPos[1]};
    double bot_to_ball_dist = magnitude_vector(bot_to_ball_vector);
    
    double bot_to_targetP_vector[2] = {targetPos[0] - botPos[0], targetPos[1] - botPos[1]};
    double target_to_goal_vector[2] = {goalPos[0] - targetPos[0], goalPos[1] - targetPos[1]};

    double beta = getAngle_vector(bot_to_targetP_vector, target_to_goal_vector);
    //printf("beta angle : %f\n", beta);

    double bot_to_targetP_dist = magnitude_vector(bot_to_targetP_vector);


    double bot_to_targetP_angle = getAngle_vector(bot_to_targetP_vector, botHeading);
    double bot_to_ball_angle = getAngle_vector(bot_to_ball_vector, botHeading); 

    BT_all_stop(0);

    switch (old_state)
    {
    case 101: { // Penalty
        // TODO: set initialization stuff here
        // ex: MUST move target computation here
        double goal_to_ball[2] = {ballPos[0]- goalPos[0], ballPos[1] - goalPos[1] };
        double magnitude = magnitude_vector(goal_to_ball);

        goal_to_ball[0] = GOOD_BALL_DIST * goal_to_ball[0] / magnitude;
        goal_to_ball[1] = GOOD_BALL_DIST * goal_to_ball[1] / magnitude;
        
        // target = ball + offset in ball's direction
        targetPos[0] = ballPos[0] + goal_to_ball[0];
        targetPos[1] = ballPos[1] + goal_to_ball[1];

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
            //Run PID to targetP
            // BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, 25);

        double distance_err, lateral_err;
        finding_errors(targetPos, botPos, goalPos, &distance_err, &lateral_err); // distance error is always positive
        double drive_straight_output = drive_straight_to_target_PID(distance_err); 
        // printf("drive_straight_output: %f\n", drive_straight_output);
        BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, MAX_SPEED * drive_straight_output);

            if(fabs(beta) < .1){ //next state
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
        return 106;
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