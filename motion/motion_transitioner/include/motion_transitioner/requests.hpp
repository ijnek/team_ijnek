#ifndef MOTION_TRANSITIONER_REQUESTS_HPP_
#define MOTION_TRANSITIONER_REQUESTS_HPP_

class GetupRequest
{
  bool frontGetup;
};

class KickRequest
{
  int power;
  bool isLeft;
};

#endif  // MOTION_TRANSITIONER_REQUESTS_HPP_