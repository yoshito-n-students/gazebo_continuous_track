#ifndef GAZEBO_CONTINUOUS_TRACK_PROPERTIES
#define GAZEBO_CONTINUOUS_TRACK_PROPERTIES

#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <ros/package.h>

namespace gazebo {

class ContinuousTrackProperties {

public:
  ContinuousTrackProperties(const physics::ModelPtr &_model, const sdf::ElementPtr &_sdf) {
    Load(_model, _sdf);
  }

  virtual ~ContinuousTrackProperties() {}

  // *****************
  // public properties
  // *****************

  struct Sprocket {
    physics::JointPtr joint;
    double pitch_diameter;
  };
  Sprocket sprocket;

  struct Trajectory {
    struct Segment {
      physics::JointPtr joint;
      double end_position;
    };
    std::vector< Segment > segments;
  };
  Trajectory trajectory;

  struct Pattern {
    std::size_t elements_per_round;
    struct Element {
      std::vector< sdf::ElementPtr > collision_sdfs;
      std::vector< sdf::ElementPtr > visual_sdfs;
    };
    std::vector< Element > elements;
  };
  Pattern pattern;

private:
  // ******************
  // loading properties
  // ******************

  // load all properties
  void Load(const physics::ModelPtr &_model, const sdf::ElementPtr &_sdf) {
    // assert the given sdf can be parsed as plugin property config
    const sdf::ElementPtr sdf(FormatAsPluginSDF(_sdf));

    LoadSprocket(_model, sdf);
    LoadTrajectory(_model, sdf);
    LoadPattern(_model, sdf);
  }

  void LoadSprocket(const physics::ModelPtr &_model, const sdf::ElementPtr &_sdf) {
    // format has been checked in Load(). no need to check if required elements exist.

    // [sprocket]
    const sdf::ElementPtr sprocket_elem(_sdf->GetElement("sprocket"));

    // []::[joint]
    sprocket.joint = _model->GetJoint(sprocket_elem->GetElement("joint")->Get< std::string >());
    GZ_ASSERT(sprocket.joint,
              "Cannot find a joint with the value of [sprocket]::[joint] element in sdf");
    GZ_ASSERT(sprocket.joint->GetType() & physics::Joint::HINGE_JOINT,
              "[sprocket]::[joint] must be a rotatinal joint");

    // []::[pitch_diameter]
    sprocket.pitch_diameter = sprocket_elem->GetElement("pitch_diameter")->Get< double >();
  }

  void LoadTrajectory(const physics::ModelPtr &_model, const sdf::ElementPtr &_sdf) {
    // format has been checked in LoadProperties(). no need to check if required elements exist.

    // [trajectory]
    const sdf::ElementPtr traj_elem(_sdf->GetElement("trajectory"));

    // []::[segment] (multiple, +)
    for (sdf::ElementPtr segment_elem = traj_elem->GetElement("segment"); segment_elem;
         segment_elem = segment_elem->GetNextElement("segment")) {
      Trajectory::Segment segment;

      // []::[]::[joint]
      segment.joint = _model->GetJoint(segment_elem->GetElement("joint")->Get< std::string >());
      GZ_ASSERT(segment.joint, "Cannot find a joint with the value of "
                               "[trajectory]::[segment]::[joint] element in sdf");
      GZ_ASSERT(segment.joint->GetType() & physics::Joint::HINGE_JOINT ||
                    segment.joint->GetType() & physics::Joint::SLIDER_JOINT,
                "[trajectory]::[segment]::[joint] must be a rotatinal or translational joint");

      // []::[]::[end_position]
      segment.end_position = segment_elem->GetElement("end_position")->Get< double >();
      GZ_ASSERT(segment.end_position > 0.,
                "[trajectory]::[segment]::[end_position] must be positive real number");

      trajectory.segments.push_back(segment);
    }
  }

  void LoadPattern(const physics::ModelPtr &_model, const sdf::ElementPtr &_sdf) {
    // format has been checked in LoadProperties(). no need to check if required elements exist.

    // [pattern]
    const sdf::ElementPtr pattern_elem(_sdf->GetElement("pattern"));

    // []::[elements_per_round]
    pattern.elements_per_round =
        pattern_elem->GetElement("elements_per_round")->Get< std::size_t >();
    GZ_ASSERT(pattern.elements_per_round > 0,
              "[pattern]::[elements_per_round] must be positive intger");

    // []::[element] (multiple, +)
    for (sdf::ElementPtr element_elem = pattern_elem->GetElement("element"); element_elem;
         element_elem = element_elem->GetNextElement("element")) {
      Pattern::Element element;

      // []::[]::[collision] (multiple, *)
      if (element_elem->HasElement("collision")) {
        for (sdf::ElementPtr collision_elem = element_elem->GetElement("collision"); collision_elem;
             collision_elem = collision_elem->GetNextElement("collision")) {
          element.collision_sdfs.push_back(collision_elem->Clone());
        }
      }

      // []::[]::[visual] (multiple, *)
      if (element_elem->HasElement("visual")) {
        for (sdf::ElementPtr visual_elem = element_elem->GetElement("visual"); visual_elem;
             visual_elem = visual_elem->GetNextElement("visual")) {
          element.visual_sdfs.push_back(visual_elem->Clone());
        }
      }

      pattern.elements.push_back(element);
    }
  }

  // **************
  // formatting sdf
  // **************

  // get a sdf element which has been initialized by the given format file.
  // the initialied sdf may look empty but have a format information.
  static sdf::ElementPtr InitializedPluginSDF() {
    const sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile(
        ros::package::getPath("gazebo_continuous_track") + "/sdf/continuous_track_plugin.sdf", sdf);
    return sdf;
  }

  // merge the plugin format sdf and the given sdf.
  // assert if the given sdf does not match the format
  // (ex. no required element, value type mismatch, ...).
  static sdf::ElementPtr FormatAsPluginSDF(const sdf::ElementPtr &_src) {
    static const sdf::ElementPtr seed(InitializedPluginSDF());
    const sdf::ElementPtr dst(seed->Clone());
    sdf::readString("<sdf version='" SDF_VERSION "'>" + _src->ToString("") + "</sdf>", dst);
    return dst;
  }
};
} // namespace gazebo

#endif