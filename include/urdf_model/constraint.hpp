#ifndef URDF_INTERFACE_CONSTRAINT_HPP
#define URDF_INTERFACE_CONSTRAINT_HPP

#include <limits>
#include <string>
#include <vector>

#include "urdf_model/pose.hpp"
#include "urdf_model/types.hpp"
namespace urdf {

class Constraint {
public:
  Constraint() { this->clear(); };

  std::string name;
  enum {
    UNKNOWN,
    REVOLUTE,
    PRISMATIC,
    UNIVERSAL,
    SPHERICAL,
    LINK
  } type;

  /// \brief     type_       cut element
  /// ------------------------------------------------------
  ///            UNKNOWN     unknown type
  ///            REVOLUTE    rotation joint
  ///            PRISMATIC   prismatic joint
  ///            UNIVERSAL   universal joint
  ///            SPHERICAL   spherical joint
  ///            LINK        link

  /// two joint axis to represent joints with more than one fixed axes, e.g.
  /// universal joints
  Vector3 parent_axis;
  Vector3 child_axis;

  /// child Link element
  ///   child_origin specifies the transform from Parent Link to Constraint
  ///   child Frame
  std::string child_link_name;
  /// transform from Parent Link frame to Constraint child frame
  Pose child_to_constraint_child_transform;

  /// parent Link element
  ///   parent_origin specifies the transform from Parent Link to Constraint
  ///   parent Frame
  std::string parent_link_name;
  /// transform from Parent Link frame to Constraint parent frame
  Pose parent_to_constraint_parent_transform;

  void clear() {
    this->parent_axis.clear();
    this->child_axis.clear();
    this->child_link_name.clear();
    this->parent_link_name.clear();
    this->child_to_constraint_child_transform.clear();
    this->parent_to_constraint_parent_transform.clear();
    this->type = UNKNOWN;
  };
};

} // namespace urdf
#endif // URDF_INTERFACE_CONSTRAINT_HPP