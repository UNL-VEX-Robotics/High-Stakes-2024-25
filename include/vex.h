#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include <iostream>
#include <vector>
#include <chrono>
#include <functional>


#define waitUntil(condition)                                                   \
  do {                                                                         \
    task::sleep(5);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)