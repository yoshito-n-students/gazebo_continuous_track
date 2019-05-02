#ifndef GAZEBO_WRAP
#define GAZEBO_WRAP

#include <gazebo/physics/physics.hh>

namespace gazebo {
namespace wrap {

// *****
// World
// *****

static inline physics::PhysicsEnginePtr Physics(const physics::WorldPtr &_world) {
#if GAZEBO_MAJOR_VERSION >= 8
  return _world->Physics();
#else
  return _world->GetPhysicsEngine();
#endif
}

// ****
// Link
// ****

static inline ignition::math::Pose3d WorldPose(const physics::LinkPtr &_link) {
#if GAZEBO_MAJOR_VERSION >= 8
  return _link->WorldPose();
#else
  return _link->GetWorldPose().Ign();
#endif
}

// *****
// Joint
// *****

static inline ignition::math::Vector3d GlobalAxis(const physics::JointPtr &_joint,
                                                  const unsigned int _index) {
#if GAZEBO_MAJOR_VERSION >= 8
  return _joint->GlobalAxis(_index);
#else
  return _joint->GetGlobalAxis(_index).Ign();
#endif
}

static inline double Position(const physics::JointPtr &_joint, const unsigned int _index) {
#if GAZEBO_MAJOR_VERSION >= 8
  return _joint->Position(_index);
#else
  return *(_joint->GetAngle(_index));
#endif
}

static inline bool SetPosition(const physics::JointPtr &_joint, const unsigned int _index,
                               const double _position, const bool _preserveWorldVelocity = false) {
#if GAZEBO_MAJOR_VERSION >= 8
  return _joint->SetPosition(_index, _position, _preserveWorldVelocity);
#else
  const bool result(_joint->SetPosition(_index, _position));
  if (_preserveWorldVelocity) {
    // TODO: do something to preserve world velocity of child links
  }
  return result;
#endif
}

static inline ignition::math::Pose3d WorldPose(const physics::JointPtr &_joint) {
#if GAZEBO_MAJOR_VERSION >= 8
  return _joint->WorldPose();
#else
  return _joint->GetWorldPose().Ign();
#endif
}

} // namespace wrap
} // namespace gazebo

#endif