
#include "roboAI.h"

#ifndef _LOST_FINDER_
#define _LOST_FINDER_

/**
 * usage: 
 * if (<.. cant find bot ...>) {
 *  pendulum_s_movement();
 * } else {
 *  reset_lost_config();
 * }
 * 
*/

static int MOTOR_1_POWER = 50, MOTOR_2_POWER = 15;
static int current_frame_count = 0, target_frame_count = 0, go_backwards = 1, undo_previous_action = 0;

void inline reset_lost_config() {
    if (!(current_frame_count == 0 && target_frame_count == 0 && go_backwards == 1 && undo_previous_action == 0)) {
      current_frame_count = 0; target_frame_count = 0; go_backwards = 1; undo_previous_action = 0;
      printf("robot found and motors stopped\n");
      BT_all_stop(1);
    }
}

void pendulum_s_movement() {
  if (go_backwards && !undo_previous_action && current_frame_count == 0) {
    target_frame_count += 10;
    current_frame_count = 0;
  }

  current_frame_count += 1;

  int left_motor_power = 0, right_motor_power = 0;
  if (go_backwards) {
    // printf("go_backwards\n");
    if (!undo_previous_action) {
      // printf("first action\n");
      left_motor_power = -MOTOR_1_POWER;
      right_motor_power = -MOTOR_2_POWER;
      if (current_frame_count == target_frame_count) {
        undo_previous_action = 1;
        current_frame_count = 0;
      }
    } else {
      // printf("undo_previous_action\n");
      left_motor_power = MOTOR_1_POWER;
      right_motor_power = MOTOR_2_POWER;
      if (current_frame_count == target_frame_count) {
        go_backwards = 0;
        undo_previous_action = 0;
        current_frame_count = 0;
      }
    }
  } else {
    // printf("go_fwd\n");
    if (!undo_previous_action) {
      // printf("first action\n");
      left_motor_power = MOTOR_2_POWER;
      right_motor_power = MOTOR_1_POWER;
      if (current_frame_count == target_frame_count) {
        undo_previous_action = 1;
        current_frame_count = 0;
      }
    } else {
      // printf("undo_previous_action\n");
      left_motor_power = -MOTOR_2_POWER;
      right_motor_power = -MOTOR_1_POWER;
      if (current_frame_count == target_frame_count) {
        go_backwards = 1;
        undo_previous_action = 0;
        current_frame_count = 0;
      }
    }
  }

  // printf("left_motor_power: %d, right_motor_power: %d, cur_frame_count: %d, total: %d\n", left_motor_power, right_motor_power, current_frame_count, target_frame_count);
  // reverse 
  BT_turn(LEFT_MOTOR, left_motor_power, RIGHT_MOTOR, right_motor_power);
  // get to starting
  BT_turn(LEFT_MOTOR, left_motor_power, RIGHT_MOTOR, right_motor_power);
}

#endif