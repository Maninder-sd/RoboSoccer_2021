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


#define BALL_LOST 5


int get_event(struct RoboAI *ai, int old_state){
    // Find event based on the info in RoboAI ai struct (maybe also old_state)

switch (old_state)
​{
    case 2: // Penalty
      //
      return 3;

      if( !(ai->st.old_bcx && ai->st.old_bcy && ai->st.old_scx && ai->st.old_scy) ){
          return BALL_LOST;
      }

      int ballPos[2]= {ai->st.old_bcx, ai->st.old_bcy};
      int botPos[2]= {ai->st.old_scx, ai->st.old_scy};
      int goalPos[2];

       // side=0 implies the robot's own side is the left side
	   // side=1 implies the robot's own side is the right side
       goalPos[0] = (ai->side == 0) ? IM_SIZE_X : 0; //gets the enemy goal
       goalPos[1] = IM_SIZE_Y/2;

      double bot_to_ball_dist ;
      double bot_to_line_dist ;

      finding_errors( ballPos, bot_pos, goalPos , &bot_to_ball_dist, &bot_to_line_dist);
      
      break;

    case 3: // is it facing the ball?
      if( !(ai->st.old_bcx && ai->st.old_bcy && ai->st.old_scx && ai->st.old_scy) ){ //NULL check
          return BALL_LOST;
      }

    //   int ballPos[2]= {ai->st.old_bcx, ai->st.old_bcy};
    //   int botPos[2]= {ai->st.old_scx, ai->st.old_scy}; 
      int bot_to_ball_vector[2] = { ai->st.old_bcx - ai->st.old_scx, ai->st.old_bcy - ai->st.old_scy };
      int headingDir_vector[2] = {ai->sdx, ai->st.sdy};
      
      double theta = dottie_vector(bot_to_ball_vector, headingDir_vector) / sqtr(dottie_vector(bot_to_ball_vector,bot_to_ball_vector) ,dottie_vector(headingDir_vector, headingDir_vector)  );
      theta = acos(theta);

      if( fabs(theta) < 0.2 ){ 
          return 
      }

      // statements
      break;

    default:
        return -1;
}

    
}

int get_new_state(int old_state, int event){
    // Given (old_state, event) -> gets next state
    return -1;
}

void do_action(int state){
    // Given (state) -> gets action function
    return;
}