#ifndef GAZEBO_WRAP
#define GAZEBO_WRAP

#include <map>
#include <string>

#include <gazebo/physics/physics.hh>

namespace gazebo {
namespace wrap {

#if GAZEBO_MAJOR_VERSION < 8
// utility to append a static member variable for a non-templated class
// without defining the static member in .cpp file
template < typename T > struct StaticVar { static T value_; };
template < typename T > T StaticVar< T >::value_;
#endif

// *****
// World
// *****

static inline std::string Name(const physics::WorldPtr &_world) {
#if GAZEBO_MAJOR_VERSION >= 8
  return _world->Name();
#else
  return _world->GetName();
#endif
}

static inline physics::PhysicsEnginePtr Physics(const physics::WorldPtr &_world) {
#if GAZEBO_MAJOR_VERSION >= 8
  return _world->Physics();
#else
  return _world->GetPhysicsEngine();
#endif
}

// *****
// Model
// *****

#if GAZEBO_MAJOR_VERSION < 8
// magic to access physics::Model::links which is a private member variable.

// pointer type of the private member we want to access
typedef physics::Link_V physics::Model::*LinksPtrT;

// actual implementation of CreateLink().
// The member variable StaticVar<>::value_ is initialized to &Model::links
// by CreateLinkImplInitializer.
class CreateLinkImpl : private StaticVar< LinksPtrT > {
  template < LinksPtrT LinksPtr > friend class CreateLinkImplInitializer;

public:
  static physics::LinkPtr Call(const physics::ModelPtr &_model, const std::string &_name) {
    // create a named link
    const physics::LinkPtr link(_model->GetWorld()->GetPhysicsEngine()->CreateLink(_model));
    link->SetName(_name);

    // add the new link to the private cache of the model.
    // this cannot be performed without accessing the private variable in gazebo 7.0 .
    ((*_model).*StaticVar< LinksPtrT >::value_).push_back(link);

    return link;
  }
};

// the constructor initializes CreateLinkImpl::StaticVar<>::value_ according to the template
// variable.
template < LinksPtrT LinksPtr > class CreateLinkImplInitializer {
public:
  CreateLinkImplInitializer() { CreateLinkImpl::StaticVar< LinksPtrT >::value_ = LinksPtr; }

private:
  static CreateLinkImplInitializer instance_;
};
template < LinksPtrT LinksPtr >
CreateLinkImplInitializer< LinksPtr > CreateLinkImplInitializer< LinksPtr >::instance_;

// instantiate Initializer with &Model::links.
// this calls the constructor of Initializer,
// and it initializes CreateLinkImpl::StaticVar<>::value_ to &Model::links
template class CreateLinkImplInitializer< &physics::Model::links >;
#endif

static inline physics::LinkPtr CreateLink(const physics::ModelPtr &_model,
                                          const std::string &_name) {
#if GAZEBO_MAJOR_VERSION >= 8
  return _model->CreateLink(_name);
#else
  return CreateLinkImpl::Call(_model, _name);
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

#if GAZEBO_MAJOR_VERSION < 8
typedef math::Pose (physics::Joint::*ComputeChildLinkPosePtrT)(unsigned int, double);
typedef bool (physics::Joint ::*FindAllConnectedLinksPtrT)(const physics::LinkPtr &,
                                                           physics::Link_V &);

class SetPositionImpl : private StaticVar< ComputeChildLinkPosePtrT >,
                        private StaticVar< FindAllConnectedLinksPtrT > {
  template < ComputeChildLinkPosePtrT ComputeChildLinkPosePtr,
             FindAllConnectedLinksPtrT FindAllConnectedLinksPtr >
  friend class SetPositionImplInitializer;

public:
  static bool Call(const physics::JointPtr &_joint, const unsigned int _index, double _position,
                   const bool _preserveWorldVelocity) {
    {
      // limit desired position
      const double lower_limit(*(_joint->GetLowerLimit(_index)));
      const double upper_limit(*(_joint->GetUpperLimit(_index)));
      _position = lower_limit < upper_limit
                      ? ignition::math::clamp(_position, lower_limit, upper_limit)
                      : ignition::math::clamp(_position, upper_limit, lower_limit);
    }

    if (_preserveWorldVelocity) {
      // child link's current pose & new pose based on position change
      const math::Pose child_pose(_joint->GetChild()->GetWorldPose());
      const math::Pose new_child_pose(
          ((*_joint).*StaticVar< ComputeChildLinkPosePtrT >::value_)(_index, _position));

      // populate child links recursively
      physics::Link_V links;
      ((*_joint).*StaticVar< FindAllConnectedLinksPtrT >::value_)(_joint->GetParent(), links);

      // update pose of each child link on the basis of joint position change
      for (const physics::LinkPtr &link : links) {
        // NEVER EVER call Link::MoveFrame(child_pose, new_child_pose)
        // because there is a critical bug which zeros link world velocity
        link->SetWorldPose((link->GetWorldPose() - child_pose) + new_child_pose);
      }
      return true;
    } else {
      return _joint->SetPosition(_index, _position);
    }
  }
};

template < ComputeChildLinkPosePtrT ComputeChildLinkPosePtr,
           FindAllConnectedLinksPtrT FindAllConnectedLinksPtr >
class SetPositionImplInitializer {
public:
  SetPositionImplInitializer() {
    SetPositionImpl::StaticVar< ComputeChildLinkPosePtrT >::value_ = ComputeChildLinkPosePtr;
    SetPositionImpl::StaticVar< FindAllConnectedLinksPtrT >::value_ = FindAllConnectedLinksPtr;
  }

private:
  static SetPositionImplInitializer instance_;
};
template < ComputeChildLinkPosePtrT ComputeChildLinkPosePtr,
           FindAllConnectedLinksPtrT FindAllConnectedLinksPtr >
SetPositionImplInitializer< ComputeChildLinkPosePtr, FindAllConnectedLinksPtr >
    SetPositionImplInitializer< ComputeChildLinkPosePtr, FindAllConnectedLinksPtr >::instance_;

template class SetPositionImplInitializer< &physics::Joint::ComputeChildLinkPose,
                                           &physics::Joint::FindAllConnectedLinks >;
#endif

static inline bool SetPosition(const physics::JointPtr &_joint, const unsigned int _index,
                               const double _position, const bool _preserveWorldVelocity = false) {
#if GAZEBO_MAJOR_VERSION >= 8
  return _joint->SetPosition(_index, _position, _preserveWorldVelocity);
#else
  return SetPositionImpl::Call(_joint, _index, _position, _preserveWorldVelocity);
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