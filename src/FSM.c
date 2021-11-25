/****
 This file contains logic to :
    1) Find event based on the info in RoboAI ai struct (maybe also old_state)
    2) Given (old_state, event) -> gets next state
    3) Given (state) -> gets action function 
****/

// #define NUM_STATES 300
// #define NUM_EVENTS 100

// int FSM[NUM_STATES][NUM_EVENTS];

// double dottie_vector(double v[2], double u[2])
// {
//     // Returns the dot product of the two vectors
//     return (v[0] * u[0]) + (v[1] * u[1]);
// }

double magnitude_vector(double v[2])
{
    // Returns the dot product of the two vectors
    return sqrt(dottie_vector(v, v));
}


double getAngle_vector(double u[2] , double v[2])
{
    // Returns the dot product of the two vectors
    return acos(dottie_vector(u, v) / sqrt(dottie_vector(u, u)* dottie_vector(v, v)));
}

#define STOP_BOT 7
#define GOOD_BALL_DIST 100




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

            if( fabs(bot_to_targetP_angle) > 1){ //more than 60 deg
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
            BT_motor_port_start(LEFT_MOTOR|RIGHT_MOTOR, 25);

            if(bot_to_targetP_dist < 100){ //next state
                BT_all_stop(0);
                return 204;
            }else if ( fabs(bot_to_targetP_angle) > 1){ // more than 60 deg
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

            if(bot_to_ball_dist < 100){ //next state
                BT_all_stop(0);
                return 206;
            }else if ( fabs(bot_to_ball_angle) > 1){ // more than 60 deg
                BT_all_stop(0);
                return 201 // reset entire thing;
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




    switch (old_state)
    {
    case 101: { // Penalty
        return 102;

     if (ai->st.self==NULL)                                         
        {
            return STOP_BOT;
        }

        // double bot_to_ball_dist ;
        double bot_to_line_dist;

        // finding_errors(ballPos, bot_pos, goalPos, &bot_to_ball_dist, &bot_to_line_dist);

        break;
    }
    case 102: {// is it facing the target_p?
     if (ai->st.self==NULL)                                         
        { //NULL check
            return STOP_BOT;
        }

        //   int ballPos[2]= {ai->st.old_bcx, ai->st.old_bcy};
        //   int botPos[2]= {ai->st.old_scx, ai->st.old_scy};
        // int bot_to_ball_vector[2] = { ai->st.old_bcx - ai->st.old_scx, ai->st.old_bcy - ai->st.old_scy };
        // // side=0 implies the robot's own side is the left side
        // // side=1 implies the robot's own side is the right side
        // bot_to_ball_vector[0] = (ai->side == 0) ? IM_SIZE_X : 0; //gets the enemy goal

        // double theta = dottie_vector(bot_to_targetP_vector, botHeading) / sqtr(dottie_vector(bot_to_targetP_vector, bot_to_targetP_vector)* dottie_vector(botHeading, headingDir_vector));
        int done_turning = turn_to_target(botHeading[0], botHeading[1], bot_to_targetP_vector[0], bot_to_targetP_vector[1]);
        
        return done_turning ? 103 : 102;
        // // maninder's simple version for turning
        // double theta = acos(dottie_vector(bot_to_targetP_vector, botHeading) / sqrt(dottie_vector(bot_to_targetP_vector, bot_to_targetP_vector)* dottie_vector(botHeading, botHeading)));
        // printf("theta: %f\n", theta);
        // // theta = acos(theta);
        // // get_angle_ ( bot_to_targetP_vector, headingDir_vector  );


        // BT_motor_port_start(LEFT_MOTOR, -10);
        // BT_motor_port_start(RIGHT_MOTOR, 10);

        // statements
        break;
    }
    case 103: {
     if (ai->st.self==NULL)                                         
        { //NULL check
            return STOP_BOT;
        }
        printf("bot_to_targetP_dist %f\n", bot_to_targetP_dist);
        if (bot_to_targetP_dist < 100)
        {
            return 104;
        }
        BT_motor_port_start(LEFT_MOTOR | RIGHT_MOTOR, 25);
        return 103; //stay in current state
        break;
    }
    case 104: {
     if (ai->st.self==NULL)                                         
        { //NULL check
            return STOP_BOT;
        }
        double theta = acos(dottie_vector(bot_to_ball_vector, botHeading) / sqrt(dottie_vector(bot_to_ball_vector, bot_to_ball_vector)* dottie_vector(botHeading, botHeading)));
        // printf("bot_to_ball_vector %f botHeading %f",bot_to_ball_vector, botHeading);
        // theta = acos(theta);
        
        // get_signed_angle_from_vectors(bot_to_ball[0], bot_to_ball[1], botHeading[0], botHeading[1]);
        printf("5- theta: %f\n", theta);
        if (fabs(theta) < 0.2)
        {             //event = facing ball
            return 105; // State turning = stop+move forward PID
        }

        BT_motor_port_start(LEFT_MOTOR, -10);
        BT_motor_port_start(RIGHT_MOTOR, 10);

        return 104;
        break;
    }
    case 105: {
        BT_motor_port_start(LEFT_MOTOR | RIGHT_MOTOR, 25);
        BT_motor_port_start(MOTOR_C, -100);
        if (bot_to_ball_dist > 300)
        {
            return 107;
        }

        return 105;
        break;
    }
    case 106: {
     if (ai->st.self==NULL)                                         
        {             //NULL check
            return 101; //reset penalty
        }
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



/*
int get_new_state_Penalty(struct RoboAI *ai, int old_state){
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
    if (targetP[0] == -1 && targetP[1] == -1 ) {
        targetP[0] = (ai->st.side == 0) ? ballPos[0] - GOOD_BALL_DIST : ballPos[0] + GOOD_BALL_DIST;
        targetP[1] = ballPos[1];
    }
    // targetP[0] = (ai->st.side == 0) ? targetP[0] - GOOD_BALL_DIST : targetP[0] + GOOD_BALL_DIST; // add a bit to x distance

    double bot_to_targetP_vector[2] = {targetP[0] - ai->st.old_scx, targetP[1] - ai->st.old_scy};
    double bot_to_targetP_dist = magnitude_vector(bot_to_targetP_vector);

    double bot_to_ball_vector[2] = {ballPos[0] - botPos[0], ballPos[1] - botPos[0]};
    double bot_to_ball_dist = magnitude_vector(bot_to_ball_vector);

    switch (old_state)
    {
    case 2: { // Penalty
        return 3;

        if (!(ai->st.old_bcx && ai->st.old_bcy && ai->st.old_scx && ai->st.old_scy))
        {
            return STOP_BOT;
        }

        // double bot_to_ball_dist ;
        double bot_to_line_dist;

        // finding_errors(ballPos, bot_pos, goalPos, &bot_to_ball_dist, &bot_to_line_dist);

        break;
    }
    case 3: {// is it facing the target_p?
        if (!(ai->st.old_bcx && ai->st.old_bcy && ai->st.old_scx && ai->st.old_scy))
        { //NULL check
            return STOP_BOT;
        }

        //   int ballPos[2]= {ai->st.old_bcx, ai->st.old_bcy};
        //   int botPos[2]= {ai->st.old_scx, ai->st.old_scy};
        // int bot_to_ball_vector[2] = { ai->st.old_bcx - ai->st.old_scx, ai->st.old_bcy - ai->st.old_scy };
        // // side=0 implies the robot's own side is the left side
        // // side=1 implies the robot's own side is the right side
        // bot_to_ball_vector[0] = (ai->side == 0) ? IM_SIZE_X : 0; //gets the enemy goal

        // double theta = dottie_vector(bot_to_targetP_vector, botHeading) / sqtr(dottie_vector(bot_to_targetP_vector, bot_to_targetP_vector)* dottie_vector(botHeading, headingDir_vector));
        double theta = acos(dottie_vector(bot_to_targetP_vector, botHeading) / sqrt(dottie_vector(bot_to_targetP_vector, bot_to_targetP_vector)* dottie_vector(botHeading, botHeading)));
        printf("theta: %f\n", theta);
        // theta = acos(theta);
        // get_angle_ ( bot_to_targetP_vector, headingDir_vector  );

        if (fabs(theta) < 0.2)
        {             //event = facing ball
            return 4; // State turning = stop+move forward PID
        }
        BT_motor_port_start(LEFT_MOTOR, -10);
        BT_motor_port_start(RIGHT_MOTOR, 10);

        // statements
        return 3; //stay in current state
        break;
    }
    case 4: {
        if (!(ai->st.old_bcx && ai->st.old_bcy && ai->st.old_scx && ai->st.old_scy))
        { //NULL check
            return STOP_BOT;
        }
        printf("bot_to_targetP_dist %f\n", bot_to_targetP_dist);
        if (bot_to_targetP_dist < 100)
        {
            return 5;
        }
        BT_motor_port_start(LEFT_MOTOR | RIGHT_MOTOR, 25);
        return 4; //stay in current state
        break;
    }
    case 5: {
        if (!(ai->st.old_bcx && ai->st.old_bcy && ai->st.old_scx && ai->st.old_scy))
        { //NULL check
            return STOP_BOT;
        }
        double theta = acos(dottie_vector(bot_to_ball_vector, botHeading) / sqrt(dottie_vector(bot_to_ball_vector, bot_to_ball_vector)* dottie_vector(botHeading, botHeading)));
        // printf("bot_to_ball_vector %f botHeading %f",bot_to_ball_vector, botHeading);
        // theta = acos(theta);
        
        // get_signed_angle_from_vectors(bot_to_ball[0], bot_to_ball[1], botHeading[0], botHeading[1]);

        if (fabs(theta) < 0.2)
        {             //event = facing ball
            return 6; // State turning = stop+move forward PID
        }

        BT_motor_port_start(LEFT_MOTOR, -10);
        BT_motor_port_start(RIGHT_MOTOR, 10);

        return 5;
        break;
    }
    case 6: {
        BT_motor_port_start(LEFT_MOTOR | RIGHT_MOTOR, 25);
        BT_motor_port_start(MOTOR_C, -100);
        if (bot_to_ball_dist > 800)
        {
            return 8;
        }

        return 6;
        break;
    }
    case 7: {
        if (ai->st.old_bcx && ai->st.old_bcy && ai->st.old_scx && ai->st.old_scy)
        {             //NULL check
            return 2; //reset penalty
        }
        BT_all_stop(0);
        break;
    }
    case 8: {
        BT_all_stop(0);
        return 8;
        break;
    }
     default: {
         return 2;
     }
    }

    return 2;
}

*/