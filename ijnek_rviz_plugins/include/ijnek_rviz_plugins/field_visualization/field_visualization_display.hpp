#pragma once

#include <memory>

#include "ijnek_interfaces/msg/field_visualization.hpp"
#include "rviz_common/ros_topic_display.hpp"

namespace Ogre
{
class ManualObject;
}

namespace rviz_rendering
{
class BillboardLine;
class MeshShape;
}



namespace ijnek_rviz_plugins
{

class FieldVisualizationDisplay
  : public rviz_common::RosTopicDisplay<ijnek_interfaces::msg::FieldVisualization>
{
public:
  FieldVisualizationDisplay();

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;

  void update(float wall_dt, float ros_dt) override;

  void reset() override;

protected:
  void processMessage(ijnek_interfaces::msg::FieldVisualization::ConstSharedPtr field_visualization) override;

private:
  Ogre::ManualObject * manual_object_;
  std::shared_ptr<rviz_rendering::MeshShape> grass_;
  std::shared_ptr<rviz_rendering::BillboardLine> lines_;
};

}  // namespace ijnek_rviz_plugins
