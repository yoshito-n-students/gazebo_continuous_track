#ifndef GAZEBO_CONTINUOUS_TRACK
#define GAZEBO_CONTINUOUS_TRACK

#include <iostream>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <boost/lexical_cast.hpp>

namespace gazebo {

class ContinuousTrack : public ModelPlugin {
public:
  void Load(physics::ModelPtr parent_model, sdf::ElementPtr plugin_sdf) {
    const std::string plugin_name(plugin_sdf->GetAttribute("name")->GetAsString());

    std::cout << "[" << plugin_name << "]:"
              << " Start loading plugin" << std::endl;

    // find the seed wheel link with the value of [seed_wheel] element
    GZ_ASSERT(plugin_sdf->HasElement("seed_wheel"), "No [seed_wheel] element");
    const physics::LinkPtr seed_wheel(
        parent_model->GetLink(plugin_sdf->Get< std::string >("seed_wheel")));
    GZ_ASSERT(seed_wheel, "Cannot find a link with the value of [seed_wheel] element");
    const std::string seed_wheel_name(seed_wheel->GetName());
    const sdf::ElementPtr seed_wheel_sdf(seed_wheel->GetSDF());

    // find the seed axle joint in parent joints of the seed wheel
    std::string seed_axle_name;
    sdf::ElementPtr seed_axle_sdf;
    for (const physics::JointPtr &joint : seed_wheel->GetParentJoints()) {
      // continuous joint is categorized to HINGE_JOINT in gazebo9 api
      if (joint->GetType() & physics::Joint::HINGE_JOINT) {
        seed_axle_name = joint->GetName();
        seed_axle_sdf = joint->GetSDF();
        break;
      }
    }
    GZ_ASSERT(seed_axle_sdf,
              "Cannot find a rotational joint in parent joints of the [seed_wheel] link");

    // find the seed belt joint in parent joints of the seed wheel
    std::string seed_belt_name;
    sdf::ElementPtr seed_belt_sdf;
    for (const physics::JointPtr &joint : seed_wheel->GetParentJoints()) {
      if (joint->GetType() & physics::Joint::GEARBOX_JOINT) {
        seed_belt_name = joint->GetName();
        seed_belt_sdf = joint->GetSDF();
        break;
      }
    }
    GZ_ASSERT(seed_belt_sdf,
              "Cannot find a gearbox joint in parent joints of the [seed_wheel] link");

    // read other properties
    GZ_ASSERT(plugin_sdf->HasElement("dpose"), "No [dpose] element");
    const ignition::math::Pose3d dpose(plugin_sdf->Get< ignition::math::Pose3d >("dpose"));
    GZ_ASSERT(plugin_sdf->HasElement("count"), "No [count] element");
    const int count(plugin_sdf->Get< int >("count"));

    // populate wheels, axles, and belts using the seed entities
    ignition::math::Pose3d new_wheel_pose(
        seed_wheel_sdf->GetElement("pose")->Get< ignition::math::Pose3d >());
    for (int i = 1; i <= count; ++i) {
      // create a new wheel
      const std::string new_wheel_name(seed_wheel_name + boost::lexical_cast< std::string >(i));
      {
        const sdf::ElementPtr sdf(seed_wheel_sdf->Clone());
        sdf->GetAttribute("name")->Set(new_wheel_name);
        new_wheel_pose += dpose;
        sdf->GetElement("pose")->Set(new_wheel_pose);
        const physics::LinkPtr wheel(parent_model->CreateLink(new_wheel_name));
        wheel->Load(sdf);
        wheel->Init();
        std::cout << "[" << plugin_name << "]:"
                  << " Created " << new_wheel_name << std::endl;
      }

      // create a new axle
      {
        const std::string name(seed_axle_name + boost::lexical_cast< std::string >(i));
        const sdf::ElementPtr sdf(seed_axle_sdf->Clone());
        sdf->GetAttribute("name")->Set(name);
        sdf->GetElement("child")->Set(new_wheel_name);
        const physics::JointPtr axle(parent_model->CreateJoint(sdf));
        axle->Init();
        std::cout << "[" << plugin_name << "]:"
                  << " Created " << name << std::endl;
      }

      // create a new belt
      {
        const std::string name(seed_belt_name + boost::lexical_cast< std::string >(i));
        const sdf::ElementPtr sdf(seed_belt_sdf->Clone());
        sdf->GetAttribute("name")->Set(name);
        sdf->GetElement("child")->Set(new_wheel_name);
        const physics::JointPtr belt(parent_model->CreateJoint(sdf));
        belt->Init();
        std::cout << "[" << plugin_name << "]:"
                  << " Created " << name << std::endl;
      }
    }

    // TODO: reduce the mass and inertia of a corresponding link

    // done!!
    std::cout << "[" << plugin_name << "]:"
              << " Loaded plugin" << std::endl;
  }
};

} // namespace gazebo

#endif