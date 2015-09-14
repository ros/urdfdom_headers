/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Steve Peters */

#ifndef URDF_MODEL_TYPES_H
#define URDF_MODEL_TYPES_H

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>


namespace urdf{

// shared pointer used in joint.h
typedef boost::shared_ptr<double> DoubleSharedPtr;

class Collision;
class Geometry;
class Inertial;
class Joint;
class JointCalibration;
class JointDynamics;
class JointLimits;
class JointMimic;
class JointSafety;
class Link;
class Material;
class Visual;

// typedef shared pointers
typedef boost::shared_ptr<Collision> CollisionSharedPtr;
typedef boost::shared_ptr<Geometry> GeometrySharedPtr;
typedef boost::shared_ptr<Inertial> InertialSharedPtr;
typedef boost::shared_ptr<Joint> JointSharedPtr;
typedef boost::shared_ptr<JointCalibration> JointCalibrationSharedPtr;
typedef boost::shared_ptr<JointDynamics> JointDynamicsSharedPtr;
typedef boost::shared_ptr<JointLimits> JointLimitsSharedPtr;
typedef boost::shared_ptr<JointMimic> JointMimicSharedPtr;
typedef boost::shared_ptr<JointSafety> JointSafetySharedPtr;
typedef boost::shared_ptr<Link> LinkSharedPtr;
typedef boost::shared_ptr<Material> MaterialSharedPtr;
typedef boost::shared_ptr<Visual> VisualSharedPtr;

// typedef const shared pointers
typedef boost::shared_ptr<const Collision> CollisionConstSharedPtr;
typedef boost::shared_ptr<const Geometry> GeometryConstSharedPtr;
typedef boost::shared_ptr<const Inertial> InertialConstSharedPtr;
typedef boost::shared_ptr<const Joint> JointConstSharedPtr;
typedef boost::shared_ptr<const JointCalibration> JointCalibrationConstSharedPtr;
typedef boost::shared_ptr<const JointDynamics> JointDynamicsConstSharedPtr;
typedef boost::shared_ptr<const JointLimits> JointLimitsConstSharedPtr;
typedef boost::shared_ptr<const JointMimic> JointMimicConstSharedPtr;
typedef boost::shared_ptr<const JointSafety> JointSafetyConstSharedPtr;
typedef boost::shared_ptr<const Link> LinkConstSharedPtr;
typedef boost::shared_ptr<const Material> MaterialConstSharedPtr;
typedef boost::shared_ptr<const Visual> VisualConstSharedPtr;

// typedef weak pointers
typedef boost::weak_ptr<Collision> CollisionWeakPtr;
typedef boost::weak_ptr<Geometry> GeometryWeakPtr;
typedef boost::weak_ptr<Inertial> InertialWeakPtr;
typedef boost::weak_ptr<Joint> JointWeakPtr;
typedef boost::weak_ptr<JointCalibration> JointCalibrationWeakPtr;
typedef boost::weak_ptr<JointDynamics> JointDynamicsWeakPtr;
typedef boost::weak_ptr<JointLimits> JointLimitsWeakPtr;
typedef boost::weak_ptr<JointMimic> JointMimicWeakPtr;
typedef boost::weak_ptr<JointSafety> JointSafetyWeakPtr;
typedef boost::weak_ptr<Link> LinkWeakPtr;
typedef boost::weak_ptr<Material> MaterialWeakPtr;
typedef boost::weak_ptr<Visual> VisualWeakPtr;

}

#endif
