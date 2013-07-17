#include <cstdlib>
#include <sys/time.h>

#include "util.hh"

double now()
{
  struct timeval now_tv;
  gettimeofday(&now_tv, NULL);
  return double(now_tv.tv_sec) + double(now_tv.tv_usec) / 1e6;
}
