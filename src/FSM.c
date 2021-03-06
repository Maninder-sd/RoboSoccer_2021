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

#define BOUNDARY_X_PADDING 10

#define Y_UP_PADDING 150
#define Y_DOWN_PADDING 150
#define X_DEFENSE_PADDING 200
#define x_OFFENSE_PADDING 50


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

    double ball_to_goal[2] = {goalPos[0] - ballPos[0], goalPos[1] - ballPos[1]};
    double enemy_to_ball[2] = {ballPos[0] - opponent[0], ballPos[1] - opponent[1]};

    static double targetP[2] = {-1, -1};

    int opponent_defending_net = 0;
    int target_on_up_boundary = (targetP[1] <= Y_UP_PADDING);
    int target_on_down_boundary = ( IM_SIZE_Y - Y_DOWN_PADDING <= targetP[1] );
    int do_up_trick_shot = 0;
    int do_down_trick_shot = 0;

    int ball_above_middle = ballPos[1] < IM_SIZE_Y / 2;
    int enemy_defending_shot = fabs(getAngle_vector(ball_to_goal, enemy_to_ball)) > M_PI*.9;

    if (enemy_defending_shot) printf("enemy defending\n");

    if(target_on_up_boundary || (enemy_defending_shot && ball_above_middle)){ // for trickshot - boundary case
        do_up_trick_shot = 1;
        printf("Trick shot up\n");
        ballPos[1] *=-1; 
    }else if (target_on_down_boundary || (enemy_defending_shot && !ball_above_middle)){
        do_down_trick_shot = 1;
         printf("Trick shot down\n");
        ballPos[1] = 2*IM_SIZE_Y -ballPos[1]; 
    }

    
    double goal_to_ball[2] = {ballPos[0]- goalPos[0], ballPos[1] - goalPos[1]};
    

    double magnitude = magnitude_vector(goal_to_ball);
    // make it vector of magnitude GOOD_BALL_DIST in the balls direction
    goal_to_ball[0] = GOOD_BALL_DIST * goal_to_ball[0] / magnitude;
    goal_to_ball[1] = GOOD_BALL_DIST * goal_to_ball[1] / magnitude;

    
    // target = ball + offset in ball's direction
    targetP[0] = ballPos[0] + goal_to_ball[0];
    targetP[1] = ballPos[1] + goal_to_ball[1];

    if(do_up_trick_shot){ // for trickshot - boundary case
        printf("Trick shot \n");
        ballPos[1] *=-1; 
        targetP[1] *= -1;
    }else if (do_down_trick_shot){
        printf("Trick shot \n");
        ballPos[1] = 2*IM_SIZE_Y -ballPos[1]; 
        targetP[1] = 2*IM_SIZE_Y -targetP[1];
    }


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

    int is_opponent_close = (magnitude_vector(opponent_to_ball) <= 200 && magnitude_vector(opponent_to_ball) < bot_to_ball_dist);
    int ball_in_middle = (ballPos[1] > (IM_SIZE_Y * 0.25) && ballPos[1] < IM_SIZE_Y * 0.75);

    double angle_thresh = 0.1;
    double own_goal_x = fabs(IM_SIZE_X - goalPos[0]);
    int bot_above_ball = botPos[1] - ballPos[1] > 0;
    int ball_in_defense_zone = fabs(ai->st.side * IM_SIZE_X - ballPos[0]) < X_DEFENSE_PADDING && !ball_in_middle;
    
    

    printf("bot offside : %d, opponent_closer : %d, opp offside : %d\n", is_bot_off_side, is_opponent_close, is_opponent_offside);

    int attack_mode = 0;
    int defense_mode = 0;
    //attack mode
    if (!is_bot_off_side && (!is_opponent_close || is_opponent_offside) && !ball_in_defense_zone) {
        printf("attack mode\n");
        target_pos[0] = targetP[0]; target_pos[1] = targetP[1];
        attack_mode = 1;
    } 

    //defense mode
    else if (is_opponent_close || ball_in_defense_zone) {
        printf("defense mode\n");
        target_pos[0] = ballPos[0] - own_goal_x; target_pos[1] = ballPos[1] - goalPos[1];

        double mg = magnitude_vector(target_pos);
        target_pos[0] = ballPos[0] - GOOD_BALL_DIST*target_pos[0] / mg;
        target_pos[1] = ballPos[1] - GOOD_BALL_DIST*target_pos[1] / mg;
        defense_mode = 1;
        
    }

    //offside deflect down
    else if (is_bot_off_side && ball_in_middle && bot_above_ball) {
        printf("offside down\n");
        double corner_target[2] = {own_goal_x, 10};
        target_pos[0] = corner_target[0] - ballPos[0]; target_pos[1] = corner_target[1] - ballPos[1];

        double mg = magnitude_vector(target_pos);
        target_pos[0] = ballPos[0] - GOOD_BALL_DIST*target_pos[0] / mg;
        target_pos[1] = ballPos[1] - GOOD_BALL_DIST*target_pos[1] / mg;

    }

    //offside deflect up
    else if (is_bot_off_side && ball_in_middle && !bot_above_ball) {
        printf("offside up\n");
        double corner_target[2] = {own_goal_x, IM_SIZE_Y - 10};
        target_pos[0] = corner_target[0] - ballPos[0]; target_pos[1] = corner_target[1] - ballPos[1];

        double mg = magnitude_vector(target_pos);
        target_pos[0] = ballPos[0] - GOOD_BALL_DIST*target_pos[0] / mg;
        target_pos[1] = ballPos[1] - GOOD_BALL_DIST*target_pos[1] / mg;

    }

    //offside flee
    else {
        printf("offside retreat");
        target_pos[0] = own_goal_x + (ai->st.side ? -20 : 20); target_pos[1] = goalPos[1];
    }



    double enemy_to_bot_vector[2] = { botPos[0] - opponent[0], botPos[1] - opponent[1]};
    double enemy_to_targetP_vector[2] = { target_pos[0] - opponent[0], targetP[1] - opponent[1]};

    double ball_to_bot_vector[2] = { botPos[0] - ballPos[0], botPos[1] - ballPos[1]};
    double ball_to_targetP_vector[2] = { target_pos[0] - ballPos[0], target_pos[1] - ballPos[1]};

    //TODO: ensure target is within bounds

     // Updates targetP based on Obstacle
    if ((attack_mode || is_bot_off_side) && fabs(getAngle_vector(enemy_to_bot_vector, enemy_to_targetP_vector)) > M_PI*2/3 ){
        // means enemy is an obstacle
        printf("enemy is obstacle \n");
        target_pos[0] = opponent[0];
        target_pos[1] = opponent[1];
        target_pos[1] =  (opponent[1] > IM_SIZE_Y/2) ? target_pos[1] - IM_SIZE_Y/3:  target_pos[1] + IM_SIZE_Y/3;
    }else if (fabs(getAngle_vector(ball_to_bot_vector, ball_to_targetP_vector)) > M_PI*3/4 ){
        // means ball is an obstacle
        printf("Ball is obstacle \n");
        targetP[0] = ballPos[0];
        targetP[1] = ballPos[1];
        targetP[1] =  (ballPos[1] > IM_SIZE_Y/2) ? target_pos[1] - IM_SIZE_Y/3:  ballPos[1] + IM_SIZE_Y/3;
    }

    double bot_to_targetP_vector[2] = {target_pos[0] - ai->st.old_scx, target_pos[1] - ai->st.old_scy};
    double bot_to_targetP_dist = magnitude_vector(bot_to_targetP_vector);


    double bot_to_targetP_angle = getAngle_vector(bot_to_targetP_vector, botHeading);

 switch (old_state){
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
            int done_turning = turn_to_target(botHeading[0], botHeading[1], bot_to_targetP_vector[0], bot_to_targetP_vector[1], 0);

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
            

        // double distance_err, lateral_err;

        // finding_errors(ballPos, targetP, goalPos, &distance_err, &lateral_err); // distance error is always positive
        // double drive_straight_output = drive_straight_to_target_PID(distance_err); 
        // BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, 50*drive_straight_output);

        //  BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, FORWARD_MAX_SPEED);


         simple_straight_to_target_PID(bot_to_targetP_dist,  bot_to_targetP_angle, 0);  // Maninder- I will test this later

            if(bot_to_targetP_dist < 50){ //next state
                BT_all_stop(0);
                return 4;
            }else if ( fabs(bot_to_targetP_angle) > angle_bound){ // more than 40 deg
                BT_all_stop(0);
                return 2;
            }else{
                return 3;
            }
            
        }
        case 4: {

            //rotation PID
            int done_turning = turn_to_target(botHeading[0], botHeading[1], bot_to_ball_vector[0], bot_to_ball_vector[1], 0);

            // if( fabs(bot_to_targetP_angle) < 0.3){ // less than 17 deg
            if(done_turning){
                BT_all_stop(1);
                BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, 20);
                sleep(1);
                return 5;
            }else if ( bot_to_targetP_dist >100 ){ // targetP changed - Hopefully this prevents going to kicking states state 5/6 
                BT_all_stop(0);
                return 1; // reset entire thing;
            }else{
                return 4;
            } 

            
        }
        case 5: {
            // just go straight and score
            BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, 60);
            BT_motor_port_start(MOTOR_C, -100);
            printf("bot_to_ball_dist %f \n",bot_to_ball_dist);

            if(bot_to_ball_dist > 200){ //next state
                BT_all_stop(0);
                return 1;
            }else if ( fabs(bot_to_ball_angle) > angle_bound){ // more than 60 deg
                BT_all_stop(0);
                return 1; // reset entire thing;
            }else{
                return 5;
            }

        }
        case 6: {
            BT_all_stop(0);

            if(bot_to_targetP_dist > 200){ //idk if should be bot_to_targetP_dist or bot_to_ball_dist
                BT_all_stop(0);
                return 1; //start pid again
            }else{
                return 1;
            }

            return 6; //stay in 206
        }

        return 1;
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
    double enemyPos[2] = {ai->st.old_ocx, ai->st.old_ocy};
    double goalPos[2];
    // side=0 implies the robot's own side is the left side
    // side=1 implies the robot's own side is the right side
    goalPos[0] = (ai->st.side == 0) ? IM_SIZE_X : 0; //gets the enemy goal
    goalPos[1] = IM_SIZE_Y / 2;

    static double targetP[2] = {-1, -1};

    int ball_on_up_boundary = (ballPos[1] <= Y_UP_PADDING);
    int ball_on_down_boundary = ( IM_SIZE_Y - Y_DOWN_PADDING <= ballPos[1] );
    
    if(ball_on_up_boundary){ // for trickshot - boundary case
    printf("Trick shot \n");
        ballPos[1] *=-1; 
    }else if (ball_on_down_boundary){
         printf("Trick shot \n");
        ballPos[1] = 2*IM_SIZE_Y -ballPos[1]; 
    }

      double goal_to_ball[2] = {ballPos[0]- goalPos[0], ballPos[1] - goalPos[1] };
      double magnitude = magnitude_vector(goal_to_ball);
      // make it vector of magnitude GOOD_BALL_DIST in the balls direction
      goal_to_ball[0] = GOOD_BALL_DIST * goal_to_ball[0] / magnitude;
      goal_to_ball[1] = GOOD_BALL_DIST * goal_to_ball[1] / magnitude;

      // target = ball + offset in ball's direction
      targetP[0] = ballPos[0] + goal_to_ball[0];
      targetP[1] = ballPos[1] + goal_to_ball[1];

    if(ball_on_up_boundary){// for trickshot - boundary case
        targetP[1] *=-1; 
        ballPos[1] *=-1; // flip back to correct ball pos
    }else if (ball_on_down_boundary){
        targetP[1] = 2*IM_SIZE_Y -targetP[1]; 
        ballPos[1] = 2*IM_SIZE_Y -ballPos[1]; // flip back to correct ball pos
    }

    double enemy_to_bot_vector[2] = { botPos[0] - enemyPos[0], botPos[1] - enemyPos[1]};
    double enemy_to_targetP_vector[2] = { targetP[0] - enemyPos[0], targetP[1] - enemyPos[1]};

    double ball_to_bot_vector[2] = { botPos[0] - ballPos[0], botPos[1] - ballPos[1]};
    double ball_to_targetP_vector[2] = { targetP[0] - ballPos[0], targetP[1] - ballPos[1]};

    // Updates targetP based on Obstacle
    if (fabs(getAngle_vector(enemy_to_bot_vector, enemy_to_targetP_vector)) > M_PI*3/4 ){
        // means enemy is an obstacle
        printf("enemy is obstacle \n");
        targetP[0] = enemyPos[0];
        targetP[1] = enemyPos[1];
        targetP[1] =  (targetP[1] > IM_SIZE_Y/2) ? targetP[1] - IM_SIZE_Y/3:  targetP[1] + IM_SIZE_Y/3;
    }else if (fabs(getAngle_vector(ball_to_bot_vector, ball_to_targetP_vector)) > M_PI*3/4 ){
        // means ball is an obstacle
        printf("Ball is obstacle \n");
        targetP[0] = ballPos[0];
        targetP[1] = ballPos[1];
        targetP[1] =  (targetP[1] > IM_SIZE_Y/2) ? targetP[1] - IM_SIZE_Y/3:  targetP[1] + IM_SIZE_Y/3;
    }
    
    printf("TargetP(x,y): %f, %f \n",targetP[0],targetP[1]);
    printf("TargetP(x,y): %f, %f \n",targetP[0],targetP[1]);
    // recalculates incase needed late on
    enemy_to_targetP_vector[0] = targetP[0] - enemyPos[0];
    enemy_to_targetP_vector[1] = targetP[1] - enemyPos[1];

    ball_to_targetP_vector[0] = targetP[0] - ballPos[0];
    ball_to_targetP_vector[1] = targetP[1] - ballPos[1];

    // calculates needed measurements 
    double bot_to_ball_vector[2] = {ballPos[0] - botPos[0], ballPos[1] - botPos[1]};
    double bot_to_ball_dist = magnitude_vector(bot_to_ball_vector);
    double bot_to_ball_angle = getAngle_vector(bot_to_ball_vector, botHeading); 
    
    double bot_to_targetP_vector[2] = {targetP[0] - ai->st.old_scx, targetP[1] - ai->st.old_scy};
    double bot_to_targetP_dist = magnitude_vector(bot_to_targetP_vector);
    double bot_to_targetP_angle = getAngle_vector(bot_to_targetP_vector, botHeading);


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
            int done_turning = turn_to_target(botHeading[0], botHeading[1], bot_to_targetP_vector[0], bot_to_targetP_vector[1], 0);
            // bootleg_turn_on_spot_PID(bot_to_targetP_angle);

            if( fabs(bot_to_targetP_angle) < 0.3){ // less than 17 deg
            // if(done_turning){
                printf("condition met\n");
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


         simple_straight_to_target_PID(bot_to_targetP_dist,  bot_to_targetP_angle, 0);  // Maninder- I will test this later

            if(bot_to_targetP_dist < 50){ //next state
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
            int done_turning = turn_to_target(botHeading[0], botHeading[1], bot_to_ball_vector[0], bot_to_ball_vector[1], 0);
            // bootleg_turn_on_spot_PID(bot_to_targetP_angle);

            // if( fabs(bot_to_targetP_angle) < 0.1){ // less than 17 deg
            if(done_turning){
                printf("condition met - in 204\n");
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
    double angle_bound=0.70; //penalty must be more precise

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
    double target_to_ball_vector[2] = {ballPos[0] - targetPos[0], ballPos[1] - targetPos[1]};
    // double target_to_goal_vector[2] = {goalPos[0] - targetPos[0], goalPos[1] - targetPos[1]};

    // double beta = getAngle_vector(bot_to_targetP_vector, target_to_goal_vector);
    //printf("beta angle : %f\n", beta);

    double bot_to_targetP_dist = magnitude_vector(bot_to_targetP_vector);
    double bot_to_target_line_projection[2];
    project_u_on_v_vec(bot_to_targetP_vector, target_to_ball_vector, bot_to_target_line_projection);

    double lateral_err_vec[2] = {bot_to_targetP_vector[0] - bot_to_target_line_projection[0], bot_to_targetP_vector[1] - bot_to_target_line_projection[1]};
    double lateral_err_mg = magnitude_vector(lateral_err_vec);

    double bot_to_goal[2] = {goalPos[0] - botPos[0], goalPos[1] - botPos[1]};
    double bot_to_goal_mg = magnitude_vector(bot_to_goal);
    bot_to_goal[0] = bot_to_goal[0] / bot_to_goal_mg;
    bot_to_goal[1] = bot_to_goal[1] / bot_to_goal_mg;

    double bot_to_targetP_angle = getAngle_vector(bot_to_targetP_vector, botHeading);
    double bot_to_ball_angle = getAngle_vector(bot_to_ball_vector, botHeading); 

    printf("targetPos[0]: %f, targetPos[1]: %f\n", targetPos[0], targetPos[1]);
    printf("ballPos[0]: %f, ballPos[1]: %f\n", ballPos[0], ballPos[1]);
    printf("lateral_err_mg: %f \n",lateral_err_mg);

    switch (old_state)
    {
    case 101: { // Penalty
        double goal_to_ball[2] = {ballPos[0]- goalPos[0], ballPos[1] - goalPos[1] };
        double magnitude = magnitude_vector(goal_to_ball);

        goal_to_ball[0] = (GOOD_BALL_DIST + 50) * goal_to_ball[0] / magnitude;
        goal_to_ball[1] = (GOOD_BALL_DIST + 50)  * goal_to_ball[1] / magnitude;
        
        // target = ball + offset in ball's direction
        targetPos[0] = ballPos[0] + goal_to_ball[0];
        targetPos[1] = ballPos[1] + goal_to_ball[1];

        return 102;
        // if( fabs(bot_to_targetP_angle) > angle_bound){ //more than 60 deg
        //     return 102; //turning
        // }else{
        //     return 103; //going forward
        // }
    }
    case 102: {// is it facing the target_p?
        
        if (fabs(lateral_err_mg) < 10) {
            return 104;
        }

        // get target keeps updating 
        // double goal_to_ball[2] = {ballPos[0]- goalPos[0], ballPos[1] - goalPos[1] };
        // double magnitude = magnitude_vector(goal_to_ball);

        // goal_to_ball[0] = (GOOD_BALL_DIST + 50)  * goal_to_ball[0] / magnitude;
        // goal_to_ball[1] = (GOOD_BALL_DIST + 50)  * goal_to_ball[1] / magnitude;
        
        // // target = ball + offset in ball's direction
        // targetPos[0] = ballPos[0] + goal_to_ball[0];
        // targetPos[1] = ballPos[1] + goal_to_ball[1];

        //rotation PID
        int done_turning = turn_to_target(botHeading[0], botHeading[1], bot_to_targetP_vector[0], bot_to_targetP_vector[1], 1);

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

        //  BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, FORWARD_MAX_SPEED);

         simple_straight_to_target_PID(bot_to_targetP_dist,  bot_to_targetP_angle, 1);  // Maninder- I will test this later

            if(fabs(lateral_err_mg) < 7){ //next state
                BT_all_stop(0);
                return 104;
            }else if ( fabs(bot_to_targetP_angle) > angle_bound){ // more than 40 deg
                BT_all_stop(0);
                return 102;
            }else{
                return 103;
            }
        return 103;
    }
    case 104: {
        //rotation PID
        // int done_turning = turn_to_target(botHeading[0], botHeading[1], bot_to_ball_vector[0], bot_to_ball_vector[1], 1);
        int done_turning = turn_to_target(botHeading[0], botHeading[1], bot_to_goal[0], bot_to_goal[1], 1);
        // printf("botHeading[0]: %f, botHeading[1]: %f\nbot_to_goal[0]: %f, bot_to_goal[1]: %f\n", botHeading[0], botHeading[1], bot_to_goal[0], bot_to_goal[1]);
        // if( fabs(bot_to_targetP_angle) < 0.3){ // less than 17 deg
        if(done_turning){
            BT_all_stop(1);
            BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, 20);
            sleep(1);
            return 105;
        }else{
            return 104;
        } 
    return 104;

    }
    case 105: { //kicking 

            // just go straight and score
            BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, 90);
            BT_motor_port_start(MOTOR_C, -100);
            sleep(2);
            printf("bot_to_ball_dist %f \n",bot_to_ball_dist);

            if ( fabs(bot_to_ball_angle) > angle_bound){ // more than 60 deg
                BT_all_stop(0);
                return 101; // reset entire thing;
            }
            // if(bot_to_ball_dist > 200){ //next state
            BT_all_stop(0);
            return 106;
            // }
            // else{
            //     return 105;
            // }

    }
    case 106: { //done
        BT_all_stop(0);
        return 106;
        break;
    }
     default: {
         return 101;
     }
    }

    return 101;
}


#endif