#include <ros/ros.h>
#include "gjk.h"
#include <iostream>

int  main(int argc, char *argv[]) {
  shape ts1[] = {1.0, 1.0, 1.0,
                                -1.0, 1.0, 1.0, 
                                1.0, -1.0, 1.0,
                                -1.0, -1.0, 1.0,
                                1.0, 1.0, -1.0,
                                -1.0, 1.0, -1.0, 
                                1.0, -1.0, -1.0,
                                -1.0, -1.0, -1.0};
  shape cs1_1[] = {1.0, 1.0, 3.0,
                                -1.0, 1.0, 3.0, 
                                1.0, -1.0, 3.0,
                                -1.0, -1.0, 3.0,
                                1.0, 1.0, 1.0,
                                -1.0, 1.0, 1.0, 
                                1.0, -1.0, 1.0,
                                -1.0, -1.0, 1.0};

  int dim_ts = sizeof(ts1)/sizeof(*ts1);
  int dim_cs1_1 = sizeof(cs1_1)/sizeof(*cs1_1);

  bool check1 = gjk(ts1, cs1_1, dim_ts, dim_cs1_1);

  std::cout << "check result:"<< check1 << std::endl;
  
}