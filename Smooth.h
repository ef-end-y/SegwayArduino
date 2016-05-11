#ifndef SMOOTH_H
#define SMOOTH_H

#include "./QueueArray.h" // очереди

class Smooth
{
  QueueArray <float> queue;
  float sum;
  int count;

  public:

  Smooth(int set_count)
  {
    sum = 0;
    count = set_count;
    for(int i=0; i<count; i++) queue.push(0);
  }

  float get(float value)
  {
    queue.push(value);
    sum += value;
    sum -= queue.pop();
    return sum / count;
  }
};

#endif

