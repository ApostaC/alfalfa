#include "timerfd.hh"
#include "exception.hh"

using namespace std;

Timerfd::Timerfd(int clockid, int flags)
  : FileDescriptor(SystemCall("Timerfd", timerfd_create(clockid, flags)))
{
}

void Timerfd::set_timeout(const timespec & initial_expiration,
                          const timespec & interval)
{
  itimerspec its;
  its.it_value = initial_expiration;
  its.it_interval = interval;

  SystemCall("Timerfd::set_timeout", timerfd_settime(fd_num(), 0, &its, nullptr));
}

uint64_t Timerfd::read_expirations()
{
  uint64_t num_exp = 0;
  if (SystemCall("Timerfd::read_expirations", ::read(fd_num(), &num_exp, sizeof(num_exp))) != sizeof(num_exp)) {
    throw runtime_error("Read error in Timerfd::read_expirations");
  }

  return num_exp;
}
