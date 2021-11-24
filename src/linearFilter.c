#include <stdio.h>
#include <float.h>

#define MAX_SENSOR_READINGS 12
#define INVALID_FILTER_VALUE FLT_MAX
#define AVG_FILTER_SIZE 5

struct linear_filter_context {
  double values[MAX_SENSOR_READINGS];
  long unsigned int times[MAX_SENSOR_READINGS];
  int filter_size;
  int current_index;
};

static struct linear_filter_context self_bot_linear_filter_pos_x;

void print_linear_filter(struct linear_filter_context* filter_struct) {
  fprintf(stderr, "======= start linear_filter_context print\n");
  for (int i = 0; i < MAX_SENSOR_READINGS; i++) {
    fprintf(stderr, "values: %f time: %ld\n", filter_struct->values[i], filter_struct->times[i]);
  }
}

double apply_filter(struct linear_filter_context* filter_struct, double new_input, double new_time) {
  // add new input in correct position
  filter_struct->values[filter_struct->current_index] = new_input;
  filter_struct->times[filter_struct->current_index] = new_time;
  filter_struct->current_index = (filter_struct->current_index+1) % filter_struct->filter_size;
  // compute denoised value
  print_linear_filter(filter_struct);

  double mean_y = 0, mean_time = 0;
  int n=0;
  for (int i = 0; i < filter_struct->filter_size; i++) {
    if (filter_struct->values[i] != INVALID_FILTER_VALUE) {
      mean_y += filter_struct->values[i];
      mean_time += filter_struct->times[i];
      n += 1;
    }
  }

  if (n == 1) {
    return new_input;
  }

  mean_y /= n;
  mean_time /= n;
  
  double m_numer = 0, m_denom = 0;
  for (int i = 0; i < filter_struct->filter_size; i++) {
    if (filter_struct->values[i] != INVALID_FILTER_VALUE) {
      double diff_y = filter_struct->values[i] - mean_y;
      double diff_time = filter_struct->times[i] - mean_time;

      m_numer += diff_y*diff_time;
      m_denom += diff_time*diff_time;
    }
  }

  double m = m_numer / m_denom;
  double b = mean_y - m*mean_time;

  double denoised_input = m*(new_time+1) + b;

  return denoised_input;
}

void initialize_filter(struct linear_filter_context* filter_struct, int filter_size) {
  filter_struct->filter_size = filter_size;
  for (int i = 0; i < MAX_SENSOR_READINGS; i++) {
    filter_struct->values[i] = INVALID_FILTER_VALUE;
    filter_struct->times[i] = 0;
  }

  filter_struct->current_index = 0;
}
