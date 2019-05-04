#ifndef GAZEBO_CONTINUOUS_TRACK
#define GAZEBO_CONTINUOUS_TRACK

#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <gazebo_continuous_track/gazebo_continuous_track_properties.hpp>
#include <gazebo_continuous_track/gazebo_wrap.hpp>
#include <ros/package.h>

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/pointer_cast.hpp>

namespace gazebo {

class ContinuousTrack : public ModelPlugin {

private:
  typedef ContinuousTrackProperties Properties;

public:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    // store plugin name just for console message
    plugin_name_ = _sdf->GetAttribute("name")->GetAsString();

    std::cout << "[" << plugin_name_ << "]:"
              << " Start loading plugin" << std::endl;

    GZ_ASSERT(wrap::Physics(_model->GetWorld())->GetType() == "ode",
              "ContinuousTrack only supports ODE.");

    // advertise the visual topic to toggle track visuals
    InitVisualPublisher(_model);

    // load properties from sdf
    const Properties prop(_model, _sdf);

    // compose joints/links of the track by duplicating segment joints/links specified in properties
    track_ = ComposeTrack(prop);

    // schedule enabling the initial variant & disabling other variants
    // (the queued msgs are published at the begging of the first world update
    //  because gzclient does not update visuals before world starts)
    for (const Track::Belt::Segment &segment : track_.belt.segments) {
      for (std::size_t variant_id = 0; variant_id < segment.variants.size(); ++variant_id) {
        QueueVisibleMsgs(segment.variants[variant_id].link, variant_id == track_.belt.variant_id);
      }
    }

    // remove segment joints/links in properties no longer required
    RemoveExtraEntities(prop.trajectory);

    // enable callback on beggining of every world step to keep updating the track
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ContinuousTrack::UpdateTrack, this, _1));

    std::cout << "[" << plugin_name_ << "]:"
              << " Loaded plugin" << std::endl;
  }

private:
  // ***************
  // building tracks
  // ***************

  struct Track {
    //
    struct Sprocket {
      physics::JointPtr joint;
      // scale from joint position to length along track
      // (equals to the pitch radius of sprocket)
      double joint_to_track;
    };
    Sprocket sprocket;
    //
    struct Belt {
      struct Segment {
        struct Variant {
          physics::JointPtr joint;
          physics::LinkPtr link;
        };
        std::vector< Variant > variants;
        // scale from joint position to length along track
        // (1.0 if joint is translational, rotational_radius if rotational)
        double joint_to_track;
        // length of segment
        double length;
      };
      std::vector< Segment > segments;
      // circumferential length of track
      double perimeter;
      std::size_t elements_per_round;
      // index of variant to be enabled
      std::size_t variant_id;
    };
    Belt belt;
  };

  Track ComposeTrack(const Properties &_prop) const {
    Track track;
    track.sprocket = ComposeSprocket(_prop.sprocket);
    track.belt = ComposeBelt(_prop.trajectory, _prop.pattern);
    return track;
  }

  Track::Sprocket ComposeSprocket(const Properties::Sprocket &_prop) const {
    Track::Sprocket sprocket;
    sprocket.joint = _prop.joint;
    sprocket.joint_to_track = _prop.pitch_diameter / 2.;
    return sprocket;
  }

  Track::Belt ComposeBelt(const Properties::Trajectory &_traj_prop,
                          const Properties::Pattern &_pattern_prop) const {
    Track::Belt belt;
    ComposeSegments(_traj_prop, _pattern_prop, belt.segments, belt.perimeter);
    belt.elements_per_round = _pattern_prop.elements_per_round;
    belt.variant_id = 0;
    return belt;
  }

  void ComposeSegments(const Properties::Trajectory &_traj_prop,
                       const Properties::Pattern &_pattern_prop,
                       std::vector< Track::Belt::Segment > &_segments, double &_perimeter) const {
    namespace im = ignition::math;

    // fill fields related to length (_segment.joint_to_track, _segment.length, and _perimeter).
    // this resizes segments.
    FillSegmentLength(_traj_prop, _segments, _perimeter);

    // populate base sdfs which segment links/joints will inherit
    const std::vector< sdf::ElementPtr > base_link_sdfs(PopulateBaseSegmentLinkSDFs(_traj_prop));
    const std::vector< sdf::ElementPtr > base_joint_sdfs(PopulateBaseSegmentJointSDFs(_traj_prop));

    // compose segments for each variant
    for (std::size_t variant_id = 0; variant_id < _pattern_prop.elements.size(); ++variant_id) {

      // separation between adjacent elements
      const double len_step(_perimeter / _pattern_prop.elements_per_round);
      // length left on the current segment to place elements
      double len_left(-len_step / 2.);
      // length traveled on the current segment
      // (to place exactly <elements_per_round> elements, we do not initialize this value to 0)
      double len_traveled(len_step / 2.);
      // index of the element to be placed next
      // (variant[0] places element[n-1] first, ... variant[n-1] does element[0])
      std::size_t elem_id(_pattern_prop.elements.size() - 1 - variant_id);

      //
      for (std::size_t segm_id = 0; segm_id < _traj_prop.segments.size(); ++segm_id) {

        // update length left for the current segment
        len_left += _segments[segm_id].length;

        // base pose of pattern elements on the current segment
        im::Pose3d base_pose(GetChildPoseOffset(_traj_prop.segments[segm_id].joint, 0,
                                                len_traveled / _segments[segm_id].joint_to_track));
        const im::Pose3d base_pose_step(GetChildPoseOffset(
            _traj_prop.segments[segm_id].joint, 0, len_step / _segments[segm_id].joint_to_track));

        // link sdf for the current segment
        const sdf::ElementPtr link_sdf(base_link_sdfs[segm_id]->Clone());
        link_sdf->GetAttribute("name")->Set(link_sdf->GetAttribute("name")->GetAsString() +
                                            "_variant" +
                                            boost::lexical_cast< std::string >(variant_id));

        // joint sdf for the current segment
        const sdf::ElementPtr joint_sdf(base_joint_sdfs[segm_id]->Clone());
        joint_sdf->GetAttribute("name")->Set(joint_sdf->GetAttribute("name")->GetAsString() +
                                             "_variant" +
                                             boost::lexical_cast< std::string >(variant_id));
        joint_sdf->GetElement("child")->Set(link_sdf->GetAttribute("name")->GetAsString());

        // place elements onto link sdf as long as length remains
        std::size_t step_count(0);
        while (len_left >= 0.) {

          // add <collision> of the element to link sdf
          for (std::size_t collision_id = 0;
               collision_id < _pattern_prop.elements[elem_id].collision_sdfs.size();
               ++collision_id) {
            // add new <collision> on the link sdf and copy base values
            const sdf::ElementPtr collision_elem(link_sdf->AddElement("collision"));
            collision_elem->Copy(_pattern_prop.elements[elem_id].collision_sdfs[collision_id]);
            // give <collision> a unique name
            collision_elem->GetAttribute("name")->Set(
                "segment" + boost::lexical_cast< std::string >(segm_id) + "_step" +
                boost::lexical_cast< std::string >(step_count) + "_collision" +
                boost::lexical_cast< std::string >(collision_id));
            // set <collision>/<pose>
            const sdf::ElementPtr pose_elem(collision_elem->GetElement("pose"));
            const im::Pose3d pose_offset(pose_elem->Get< im::Pose3d >());
            pose_elem->Set(pose_offset + base_pose);
          }

          // add <visual> of the element to link sdf
          for (std::size_t visual_id = 0;
               visual_id < _pattern_prop.elements[elem_id].visual_sdfs.size(); ++visual_id) {
            // add new <collision> on the link sdf and copy base values
            const sdf::ElementPtr visual_elem(link_sdf->AddElement("visual"));
            visual_elem->Copy(_pattern_prop.elements[elem_id].visual_sdfs[visual_id]);
            // give <visual> a unique name
            visual_elem->GetAttribute("name")->Set(
                "segment" + boost::lexical_cast< std::string >(segm_id) + "_step" +
                boost::lexical_cast< std::string >(step_count) + "_visual" +
                boost::lexical_cast< std::string >(visual_id));
            // set <visual>/<pose>
            const sdf::ElementPtr pose_elem(visual_elem->GetElement("pose"));
            const im::Pose3d pose_offset(pose_elem->Get< im::Pose3d >());
            pose_elem->Set(pose_offset + base_pose);
          }

          // step everything
          len_left -= len_step;
          len_traveled += len_step;
          elem_id = (elem_id + 1) % _pattern_prop.elements.size();
          base_pose = base_pose_step + base_pose;
          ++step_count;
        }

        // create link/joint for the current segment on the basis of updated sdf
        {
          Track::Belt::Segment::Variant variant;

          // link
          const physics::LinkPtr base_link(_traj_prop.segments[segm_id].joint->GetChild());
          // Physics()->CreateLink() does not register a new link to the model
          // and does not show up the link correctly on gzclient (gazebo7&9)
          variant.link = wrap::CreateLink(base_link->GetModel(),
                                          link_sdf->GetAttribute("name")->GetAsString());
          variant.link->Load(link_sdf);
          variant.link->Init();
          // copy base link pose because it may be changed by another plugin loaded before this
          variant.link->SetWorldPose(wrap::WorldPose(base_link));

          // joint
          const physics::JointPtr base_joint(_traj_prop.segments[segm_id].joint);
          // Physics()->CreateJoint() does not register a new joint to the model
          // and does not show up the joint correctly on gzclient (gazebo7&9)
          variant.joint = base_joint->GetParent()->GetModel()->CreateJoint(
              joint_sdf->GetAttribute("name")->GetAsString(),
              joint_sdf->GetAttribute("type")->GetAsString(), base_joint->GetParent(),
              variant.link);
          variant.joint->Load(joint_sdf);
          variant.joint->Init();
          // set initial zero velocity
          SetJointMotorVelocity(variant.joint, 0, 0.);

          _segments[segm_id].variants.push_back(variant);
          std::cout << "[" << plugin_name_ << "]:"
                    << " Created " << variant.link->GetScopedName() << " and "
                    << variant.joint->GetScopedName() << std::endl;
        }

        // update length traveled for the next segment
        len_traveled -= _segments[segm_id].length;
      }
    }
  }

  void FillSegmentLength(const Properties::Trajectory &_traj_prop,
                         std::vector< Track::Belt::Segment > &_segments, double &_perimeter) const {
    namespace im = ignition::math;

    _perimeter = 0.;

    for (const Properties::Trajectory::Segment &segment_prop : _traj_prop.segments) {
      Track::Belt::Segment segment;

      if (segment_prop.joint->HasType(physics::Joint::HINGE_JOINT)) {
        // calc radius of rotation (= distance between link position and joint axis)
        const im::Vector3d joint_pos(wrap::WorldPose(segment_prop.joint).Pos());
        const im::Vector3d joint_axis(wrap::GlobalAxis(segment_prop.joint, 0));
        im::Vector3d link_pos(wrap::WorldPose(segment_prop.joint->GetChild()).Pos());
        // Vector3d::DistToLine() is not a const function for some reason ...
        const double radius(link_pos.DistToLine(joint_pos, joint_pos + joint_axis));
        segment.joint_to_track = radius;
        // length of segment is radius * rotation-angle
        segment.length = radius * segment_prop.end_position;
      } else if (segment_prop.joint->HasType(physics::Joint::SLIDER_JOINT)) {
        // length of segment equals to end-position of joint
        segment.joint_to_track = 1.0;
        segment.length = segment_prop.end_position;
      } else {
        // never fall here as joint type has been checked in the property loader
        GZ_ASSERT(false, "Bug. Unexpected joint type.");
      }

      _segments.push_back(segment);

      _perimeter += segment.length;
    }
  }

  std::vector< sdf::ElementPtr >
  PopulateBaseSegmentLinkSDFs(const Properties::Trajectory &_traj_prop) const {
    std::vector< sdf::ElementPtr > sdfs;

    for (const Properties::Trajectory::Segment &segment_prop : _traj_prop.segments) {
      const sdf::ElementPtr sdf(segment_prop.joint->GetChild()->GetSDF()->Clone());
      // remove original <collision> and <visual>
      if (sdf->HasElement("collision") || sdf->HasElement("visual")) {
        std::cout << "[" << plugin_name_ << "]:"
                  << " Removed <collision> and <visual> of "
                  << segment_prop.joint->GetChild()->GetScopedName() << std::endl;
        while (sdf->HasElement("collision")) {
          sdf->RemoveChild(sdf->GetElement("collision"));
        }
        while (sdf->HasElement("visual")) {
          sdf->RemoveChild(sdf->GetElement("visual"));
        }
      }
      sdfs.push_back(sdf);
    }

    return sdfs;
  }

  std::vector< sdf::ElementPtr >
  PopulateBaseSegmentJointSDFs(const Properties::Trajectory &_traj_prop) const {
    std::vector< sdf::ElementPtr > sdfs;

    for (const Properties::Trajectory::Segment &segment_prop : _traj_prop.segments) {
      sdfs.push_back(segment_prop.joint->GetSDF()->Clone());
    }

    return sdfs;
  }

  ignition::math::Pose3d GetChildPoseOffset(const physics::JointPtr &_joint, const double _from,
                                            const double _to) const {
    namespace im = ignition::math;

    // When setting the joint position, preserveWorldVelocity should be **true**.
    // If false, the physics engine may change the world velocity of the child link
    // even if we retrieve the joint position.

    const double current(wrap::Position(_joint, 0));
    // child position when the joint position is <from>
    wrap::SetPosition(_joint, 0, _from, /* preserveWorldVelocity */ true);
    const im::Pose3d pose_from(wrap::WorldPose(_joint->GetChild()));
    // child position when the joint position is <to>
    wrap::SetPosition(_joint, 0, _to, true);
    const im::Pose3d pose_to(wrap::WorldPose(_joint->GetChild()));
    // retrieve the joint position
    wrap::SetPosition(_joint, 0, current, true);

    return pose_to - pose_from;
  }

  void RemoveExtraEntities(const Properties::Trajectory &_traj_prop) const {
    for (const Properties::Trajectory::Segment &segment_prop : _traj_prop.segments) {
      // get segment link before removing joint
      // because joint->GetChild() does not work once joint has been removed (= detached)
      const physics::LinkPtr link(segment_prop.joint->GetChild());
      // remove segment joint/link
      segment_prop.joint->GetParent()->GetModel()->RemoveJoint(segment_prop.joint->GetName());
      link->GetModel()->RemoveChild(boost::static_pointer_cast< physics::Entity >(link));

      std::cout << "[" << plugin_name_ << "]:"
                << " Removed " << segment_prop.joint->GetScopedName() << " and "
                << link->GetScopedName() << std::endl;
    }
  }

  // ********************
  // Updating simulation
  // ********************

  void UpdateTrack(const common::UpdateInfo &_info) {
    // enable visuals of the current variant and disable others
    PublishVisibleMsgs();

    // state of the track
    const double track_pos(wrap::Position(track_.sprocket.joint, 0) *
                           track_.sprocket.joint_to_track);
    const double track_vel(track_.sprocket.joint->GetVelocity(0) * track_.sprocket.joint_to_track);

    // new variant id to be enabled
    const std::size_t new_variant_id(CalcVariantId(track_pos));

    if (track_.belt.variant_id != new_variant_id) {
      // schedule enabling visuals of new variant & disabling visuals of last variant
      // (actual publishment is performed at the begging of the next step
      //  because the position of the new variant has not been updated)
      for (const Track::Belt::Segment &segment : track_.belt.segments) {
        QueueVisibleMsgs(segment.variants[new_variant_id].link, true);
        QueueVisibleMsgs(segment.variants[track_.belt.variant_id].link, false);
      }
      track_.belt.variant_id = new_variant_id;
    }

    // length which a element is distributed along the track
    const double len_per_element(track_.belt.perimeter / track_.belt.elements_per_round);

    // set enabled or disabled for all links.
    // this is because all links have been enabled at the beginning of every world update
    // as the links are connected to another link which is enabled.
    for (const Track::Belt::Segment &segment : track_.belt.segments) {
      for (std::size_t variant_id = 0; variant_id < segment.variants.size(); ++variant_id) {
        if (variant_id == track_.belt.variant_id) {
          // enable the link (should happen nothing but just in case)
          segment.variants[variant_id].link->SetEnabled(true);

          // track pos normalized in [-len_per_element / 2, len_per_element / 2)
          const double track_pos_per_element(
              track_pos - len_per_element * std::floor(track_pos / len_per_element) -
              len_per_element / 2.);

          // set position & velocity
          wrap::SetPosition(segment.variants[variant_id].joint, 0,
                            track_pos_per_element / segment.joint_to_track, true);
          SetJointMotorVelocity(segment.variants[variant_id].joint, 0,
                                track_vel / segment.joint_to_track);
        } else {
          segment.variants[variant_id].link->SetEnabled(false);
        }
      }
    }
  }

  std::size_t CalcVariantId(const double _track_pos) const {
    // length which a element, or a set of unique elements is distributed along the track
    const double len_per_element(track_.belt.perimeter / track_.belt.elements_per_round);
    const double len_per_elements(len_per_element * track_.belt.segments[0].variants.size());

    // track pos normalized in [0, len_per_elements)
    const double track_pos_per_elements(_track_pos - len_per_elements *
                                                         std::floor(_track_pos / len_per_elements));

    // new variant id to be enabled
    return static_cast< std::size_t >(std::floor(track_pos_per_elements / len_per_element));
  }

  void SetJointMotorVelocity(const physics::JointPtr &_joint, const unsigned int _index,
                             const double _velocity) const {
    // using ODE's joint motors function

    // set force/torque limit
    const double effort_limit(_joint->GetEffortLimit(_index));
    _joint->SetParam("fmax", _index,
                     // negative value means unlimited
                     effort_limit > 0. ? effort_limit : std::numeric_limits< double >::max());

    // set velocity clamped by velocity limit
    const double velocity_limit(_joint->GetVelocityLimit(_index));
    _joint->SetParam("vel", 0,
                     // negative value means unlimited
                     velocity_limit > 0.
                         ? ignition::math::clamp(_velocity, -velocity_limit, velocity_limit)
                         : _velocity);
  }

  // ****************************
  // Transporting visual messages
  // ****************************

  void InitVisualPublisher(const physics::ModelPtr &_model) {
    node_.reset(new transport::Node());
    node_->Init(wrap::Name(_model->GetWorld()));
    visual_publisher_ = node_->Advertise< msgs::Visual >("~/visual");
  }

  void QueueVisibleMsgs(const physics::LinkPtr &_link, const bool _visible) {
    msgs::VisualPtr msg(new msgs::Visual());
    msg->set_name(_link->GetScopedName());
    msg->set_parent_name(_link->GetModel()->GetScopedName());
    msg->set_visible(_visible);
    visible_msgs_.push(msg);
  }

  void PublishVisibleMsgs() {
    while (!visible_msgs_.empty()) {
      visual_publisher_->Publish(*visible_msgs_.front());
      visible_msgs_.pop();
    }
  }

private:
  // the name of this plugin. use as prefix of console message.
  std::string plugin_name_;
  // transport to toggling track visuals
  transport::NodePtr node_;
  transport::PublisherPtr visual_publisher_;
  std::queue< msgs::VisualPtr > visible_msgs_;
  // the track model
  Track track_;
  // callback connection handle
  event::ConnectionPtr update_connection_;
};

} // namespace gazebo

#endif