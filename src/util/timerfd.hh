#ifndef TIMERFD_HH
#define TIMERFD_HH

#include <time.h>
#include <sys/timerfd.h>

#include "file_descriptor.hh"

class Timerfd : public FileDescriptor
{
public:
  Timerfd(int clockid = CLOCK_MONOTONIC, int flags = TFD_NONBLOCK);
  
  void set_timeout(const timespec & initial_expiration,
                   const timespec & interval);

  uint64_t read_expirations();
};

#endif
