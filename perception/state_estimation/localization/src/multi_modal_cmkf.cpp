#include "multi_modal_cmkf.hpp"

#include "cmkf.hpp"

namespace localization
{

MultiModalCMKF::MultiModalCMKF()
{
}

MultiModalCMKF::~MultiModalCMKF()
{
}

void MultiModalCMKF::predict(const nav_msgs::msg::Odometry &odometry)
{
  (void)odometry;
}

void MultiModalCMKF::update(const soccer_vision_3d_msgs::msg::MarkingArray &markers)
{
  (void)markers;
}

}  // namespace localization
