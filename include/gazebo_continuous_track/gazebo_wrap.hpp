#ifndef GAZEBO_WRAP
#define GAZEBO_WRAP

#include <map>
#include <queue>
#include <string>

#include <gazebo/physics/physics.hh>

namespace gazebo {
namespace wrap {

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

// actual implementation of CreateLink().
// The member variable links_ptr_ is initialized to &Model::links
// by CreateLinkImplInitializer.
class CreateLinkImpl {
  template < physics::Link_V physics::Model::*LinksPtr > friend class CreateLinkImplInitializer;

public:
  static physics::LinkPtr Call(const physics::ModelPtr &_model, const std::string &_name) {
    const physics::LinkPtr link(_model->GetWorld()->GetPhysicsEngine()->CreateLink(_model));
    link->SetName(_name);
    ((*_model).*links_ptr_).push_back(link);
    return link;
  }

private:
  static physics::Link_V physics::Model::*links_ptr_;
};
physics::Link_V physics::Model::*CreateLinkImpl::links_ptr_;

// the constructor initializes CreateLinkImpl::links_ptr_ according to the template variable.
template < physics::Link_V physics::Model::*LinksPtr > class CreateLinkImplInitializer {
public:
  CreateLinkImplInitializer() { CreateLinkImpl::links_ptr_ = LinksPtr; }

private:
  static CreateLinkImplInitializer instance_;
};
template < physics::Link_V physics::Model::*LinksPtr >
CreateLinkImplInitializer< LinksPtr > CreateLinkImplInitializer< LinksPtr >::instance_;

// instantiate Initializer with &Model::links.
// this calls the constructor of Initializer,
// and it initializes CreateLinkImpl::link_ptr_ to &Model::links
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

class SetPositionImpl {
  template < ComputeChildLinkPosePtrT ComputeChildLinkPosePtr,
             FindAllConnectedLinksPtrT FindAllConnectedLinksPtr >
  friend class SetPositionImplInitializer;

public:
  static bool Call(const physics::JointPtr &_joint, const unsigned int _index,
                   const double _position, const bool _preserveWorldVelocity) {
    if (_preserveWorldVelocity) {
      // child link's current pose & new pose based on position change
      const math::Pose child_pose(_joint->GetChild()->GetWorldPose());
      const math::Pose new_child_pose(((*_joint).*computeChildLinkPosePtr_)(_index, _position));

      // update all connected links
      physics::Link_V links;
      ((*_joint).*findAllConnectedLinksPtr_)(_joint->GetParent(), links);
      for (const physics::LinkPtr &link : links) {
        // NEVER EVER call Link::MoveFrame(child_pose, new_child_pose)
        // because a critical bug which zeros link world velocity exists
        link->SetWorldPose((link->GetWorldPose() - child_pose) + new_child_pose);
      }
      return true;
    } else {
      return _joint->SetPosition(_index, _position);
    }
  }

private:
  static ComputeChildLinkPosePtrT computeChildLinkPosePtr_;
  static FindAllConnectedLinksPtrT findAllConnectedLinksPtr_;
};
ComputeChildLinkPosePtrT SetPositionImpl::computeChildLinkPosePtr_;
FindAllConnectedLinksPtrT SetPositionImpl::findAllConnectedLinksPtr_;

template < ComputeChildLinkPosePtrT ComputeChildLinkPosePtr,
           FindAllConnectedLinksPtrT FindAllConnectedLinksPtr >
class SetPositionImplInitializer {
public:
  SetPositionImplInitializer() {
    SetPositionImpl::computeChildLinkPosePtr_ = ComputeChildLinkPosePtr;
    SetPositionImpl::findAllConnectedLinksPtr_ = FindAllConnectedLinksPtr;
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