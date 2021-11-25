/****
 This file contains logic to :
    1) Find event based on the info in RoboAI ai struct (maybe also old_state)
    2) Given (old_state, event) -> gets next state
    3) Given (state) -> gets action function 
****/

#define NUM_STATES 300
#define NUM_EVENTS 100

int FSM[NUM_STATES][NUM_EVENTS]; 

double dottie_vector(double v[2], double u[2])
{
 // Returns the dot product of the two vectors 
 return (v[0]*u[0])+(v[1]*u[1]);
}

double magnitude_vector(double v[2])
{
 // Returns the dot product of the two vectors 
 return sqrt(dottie_vector(v));
}


#define STOP_BOT 7


int get_new_state(struct RoboAI *ai, int old_state){

}

#define GOOD_BALL_DIST 10


int get_new_state_Penalty(struct RoboAI *ai, int old_state){
    // Find event based on the info in RoboAI ai struct (maybe also old_state)
    // call when in penalty mode
    double ballPos[2]= {ai->st.old_bcx, ai->st.old_bcy};
    double botHeading[2] = {ai->st.sdx, ai->st.sdy};
    double botPos[2]= {ai->st.old_scx, ai->st.old_scy};
    double goalPos[2];
    // side=0 implies the robot's own side is the left side
    // side=1 implies the robot's own side is the right side
    goalPos[0] = (ai->side == 0) ? IM_SIZE_X : 0; //gets the enemy goal
    goalPos[1] = IM_SIZE_Y/2;
    
    double targetP[2] = {ballPos[0],ballPos[1] }
    targetP[0] = (ai->side == 0) ? targetP[0] - GOOD_BALL_DIST : targetP[0] + GOOD_BALL_DIST; // add a bit to x distance


    double bot_to_targetP_vector[2] = { targetP[0] - ai->st.old_scx, targetP[1] - ai->st.old_scy };
    double bot_to_targetP_dist = magnitude_vector(bot_to_targetP_vector);

    double bot_to_ball_vector[2] = { ballPos[0] - botPos[0], ballPos[1] - botPos[0] };
    double bot_to_ball_dist = magnitude_vector(bot_to_ball_vector);


    switch (old_state)
    ​{
        case 2: // Penalty
            //
            return 3;

            if( !(ai->st.old_bcx && ai->st.old_bcy && ai->st.old_scx && ai->st.old_scy) ){
                return STOP_BOT;
            }

            // double bot_to_ball_dist ;
            double bot_to_line_dist ;

            finding_errors( ballPos, bot_pos, goalPos , &bot_to_ball_dist, &bot_to_line_dist);
            
            break;

        case 3: // is it facing the target_p?
            if( !(ai->st.old_bcx && ai->st.old_bcy && ai->st.old_scx && ai->st.old_scy) ){ //NULL check
                return STOP_BOT;
            }

            //   int ballPos[2]= {ai->st.old_bcx, ai->st.old_bcy};
            //   int botPos[2]= {ai->st.old_scx, ai->st.old_scy}; 
            // int bot_to_ball_vector[2] = { ai->st.old_bcx - ai->st.old_scx, ai->st.old_bcy - ai->st.old_scy };
            // // side=0 implies the robot's own side is the left side
            // // side=1 implies the robot's own side is the right side
            // bot_to_ball_vector[0] = (ai->side == 0) ? IM_SIZE_X : 0; //gets the enemy goal

            int headingDir_vector[2] = {ai->st.sdx, ai->st.sdy};
            
            double theta = get_angle_ ( bot_to_targetP_vector, headingDir_vector  );
            = dottie_vector(bot_to_targetP_vector, headingDir_vector) / sqtr(dottie_vector(bot_to_targetP_vector,bot_to_targetP_vector) ,dottie_vector(headingDir_vector, headingDir_vector)  );
            theta = acos(theta);

            if( fabs(theta) < 0.1 ){ //event = facing ball
                return 4; // State turning = stop+move forward PID 
            }

            // statements
            return 3; //stay in current state
            break;

        case 4:
            if( !(ai->st.old_bcx && ai->st.old_bcy && ai->st.old_scx && ai->st.old_scy) ){ //NULL check
                return STOP_BOT;
            }
            if (bot_to_targetP_dist < 100){
                return 5;
            }
            return 4;  //stay in current state
            break;

        case 5:
            if( !(ai->st.old_bcx && ai->st.old_bcy && ai->st.old_scx && ai->st.old_scy) ){ //NULL check
                return STOP_BOT;
            }
            double theta = get_angle (bot_to_ball, botHeading);

            if( fabs(theta) < 0.1 ){ //event = facing ball
                return 6; // State turning = stop+move forward PID 
            }

            return 5;
            break;

        case 6:
            
            if(bot_to_ball_dist > 300){
                return 8;
            }


            return 6;
            break;
        case 7:
            if( ai->st.old_bcx && ai->st.old_bcy && ai->st.old_scx && ai->st.old_scy ){ //NULL check
                return 2; //reset penalty
            }
            break;
        case 8:
            return 8;
            break;
}


int get_new_state(int old_state, int event){
    // Given (old_state, event) -> gets next state
    return -1;
}

void do_action(int state){
    // Given (state) -> gets action function
    return;
}


inline void state3(){
    // turn to face 
    return 2;
}