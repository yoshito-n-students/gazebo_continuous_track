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

#if GAZEBO_MAJOR_VERSION < 8
typedef std::map< uint32_t, msgs::Visual > Visuals_M;

class QueueVisibleMsgsImpl {
  template < Visuals_M physics::Link::*VisualsPtr > friend class QueueVisibleMsgsImplInitializer;

public:
  static void Call(std::queue< msgs::VisualPtr > &_visible_msgs, const physics::LinkPtr &_link,
                   const bool _visible) {
    for (const Visuals_M::value_type &visual : (*_link).*visuals_ptr_) {
      msgs::VisualPtr msg(new msgs::Visual());
      msg->set_name(visual.second.name());
      msg->set_parent_name(visual.second.parent_name());
      msg->set_visible(_visible);
      _visible_msgs.push(msg);
    }
  }

private:
  static Visuals_M physics::Link::*visuals_ptr_;
};
Visuals_M physics::Link::*QueueVisibleMsgsImpl::visuals_ptr_;

template < Visuals_M physics::Link::*VisualsPtr > class QueueVisibleMsgsImplInitializer {
public:
  QueueVisibleMsgsImplInitializer() { QueueVisibleMsgsImpl::visuals_ptr_ = VisualsPtr; }

private:
  static QueueVisibleMsgsImplInitializer instance_;
};
template < Visuals_M physics::Link::*VisualsPtr >
QueueVisibleMsgsImplInitializer< VisualsPtr >
    QueueVisibleMsgsImplInitializer< VisualsPtr >::instance_;

template class QueueVisibleMsgsImplInitializer< &physics::Link::visuals >;
#endif

static inline void QueueVisibleMsgs(std::queue< msgs::VisualPtr > &_visible_msgs,
                                    const physics::LinkPtr &_link, const bool _visible) {
#if GAZEBO_MAJOR_VERSION >= 8
  msgs::VisualPtr msg(new msgs::Visual());
  msg->set_name(_link->GetScopedName());
  msg->set_parent_name(_link->GetModel()->GetScopedName());
  msg->set_visible(_visible);
  _visible_msgs.push(msg);
#else
  QueueVisibleMsgsImpl::Call(_visible_msgs, _link, _visible);
#endif
}

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
  if (_preserveWorldVelocity) {
    const physics::LinkPtr child(_joint->GetChild());
    const math::Vector3 child_lin_vel(child->GetWorldLinearVel());
    const math::Vector3 child_ang_vel(child->GetWorldAngularVel());
    const bool result(_joint->SetPosition(_index, _position));
    child->SetWorldTwist(child_lin_vel, child_ang_vel);
    return result;
  } else {
    return _joint->SetPosition(_index, _position);
  }
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