/****
This file will contain the logic for handling sensor data
1) denoising sensor data
2) dealing with missing data3
3) Making predictions based on previous data
******/

#define NUM_TO_STORE 10

int is_moving; // boolean
double suggested_target[2]; 

void update_AI_denoise(struct RoboAI *ai) {
/*
    Index rules:
    0 - ball info
    1 - Our bot info
    2 - enemy bot info
*/
    static int index=0; // index to update
    static double prev_pos[NUM_TO_STORE][3][2]; // [NUM_TO_STORE] [ball;bot;enemy] [x-coord;y-coord]
    static double prev_heading[NUM_TO_STORE][3]; // [NUM_TO_STORE] [ball;bot;enemy] 


    


}