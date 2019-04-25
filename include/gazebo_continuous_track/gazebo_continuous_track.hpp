#ifndef GAZEBO_CONTINUOUS_TRACK
#define GAZEBO_CONTINUOUS_TRACK

#include <iostream>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

class ContinuousTrack : public ModelPlugin {
public:
  void Load(physics::ModelPtr parent_model, sdf::ElementPtr plugin_sdf) {
    const std::string plugin_name(plugin_sdf->GetAttribute("name")->GetAsString());

    std::cout << "[" << plugin_name << "]:"
              << " Start loading plugin" << std::endl;

    // TODO: create nice continuous track model

    // done!!
    std::cout << "[" << plugin_name << "]:"
              << " Loaded plugin" << std::endl;
  }
};

} // namespace gazebo

#endif