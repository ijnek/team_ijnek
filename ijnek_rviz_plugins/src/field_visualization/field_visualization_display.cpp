#include "ijnek_rviz_plugins/field_visualization/field_visualization_display.hpp"

#include <OgreVector.h>

#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"

#include "rviz_rendering/objects/billboard_line.hpp"
#include "rviz_rendering/objects/mesh_shape.hpp"

namespace ijnek_rviz_plugins
{

FieldVisualizationDisplay::FieldVisualizationDisplay()
: rviz_common::RosTopicDisplay<ijnek_interfaces::msg::FieldVisualization>()
{

}

void FieldVisualizationDisplay::onInitialize()
{
  RosTopicDisplay::onInitialize();
  topic_property_->setValue("field_visualization");
  topic_property_->setDescription("ijnek_interfaces::msg::FieldVisualization topic to subscribe to.");
}


void FieldVisualizationDisplay::load(const rviz_common::Config & config)
{
  Display::load(config);
}

void FieldVisualizationDisplay::processMessage(ijnek_interfaces::msg::FieldVisualization::ConstSharedPtr msg)
{
  scene_node_->setVisible(true);

  if (!grass_) {
    grass_ = std::make_shared<rviz_rendering::MeshShape>(
      scene_manager_, scene_node_);
    grass_->estimateVertexCount(4);

    grass_->beginTriangles();
    Ogre::ColourValue c;
    c.r = 0.0;
    c.g = 1.0;
    c.b = 0.0;
    c.a = 0.4;
    grass_->setColor(c);

    auto x = msg->field_length / 2.0;
    auto y = msg->field_width / 2.0;
    grass_->addVertex(Ogre::Vector3(x, y, 0));
    grass_->addVertex(Ogre::Vector3(x, -y, 0));
    grass_->addVertex(Ogre::Vector3(-x, y, 0));
    grass_->addVertex(Ogre::Vector3(-x, -y, 0));

    grass_->addTriangle(0, 2, 3);
    grass_->addTriangle(0, 1, 3);
    grass_->endTriangles();
  }

  if (!lines_) {
    lines_ = std::make_shared<rviz_rendering::BillboardLine>(
      scene_manager_, scene_node_);
    lines_->setColor(1, 0, 0, 1);
    lines_->setLineWidth(0.1);
    lines_->setMaxPointsPerLine(2);
    lines_->setNumLines(static_cast<uint32_t>(msg->field_lines.size()));

    Ogre::ColourValue c;
    c.r = 1.0;
    c.g = 1.0;
    c.b = 1.0;
    c.a = 1.0;

    for (const auto & field_line : msg->field_lines) {
      const auto& p1 = field_line.start;
      const auto& p2 = field_line.end;
      Ogre::Vector3 v1(p1.x, p1.y, p1.z);
      Ogre::Vector3 v2(p2.x, p2.y, p2.z);
      lines_->addPoint(v1, c);
      lines_->addPoint(v2, c);
      lines_->finishLine();
    }
  }
}

void FieldVisualizationDisplay::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;
}

void FieldVisualizationDisplay::reset()
{
  RosTopicDisplay::reset();
}

}  // namespace ijnek_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ijnek_rviz_plugins::FieldVisualizationDisplay, rviz_common::Display)
