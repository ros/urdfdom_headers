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

#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#define URDF_TYPEDEF_CLASS_POINTER(Class) \
class Class; \
typedef boost::shared_ptr<Class> Class##SharedPtr; \
typedef boost::shared_ptr<const Class> Class##ConstSharedPtr; \
typedef boost::weak_ptr<Class> Class##WeakPtr

namespace urdf{

// shared pointer used in joint.h
typedef boost::shared_ptr<double> DoubleSharedPtr;

URDF_TYPEDEF_CLASS_POINTER(Box);
URDF_TYPEDEF_CLASS_POINTER(Collision);
URDF_TYPEDEF_CLASS_POINTER(Cylinder);
URDF_TYPEDEF_CLASS_POINTER(Geometry);
URDF_TYPEDEF_CLASS_POINTER(Inertial);
URDF_TYPEDEF_CLASS_POINTER(Joint);
URDF_TYPEDEF_CLASS_POINTER(JointCalibration);
URDF_TYPEDEF_CLASS_POINTER(JointDynamics);
URDF_TYPEDEF_CLASS_POINTER(JointLimits);
URDF_TYPEDEF_CLASS_POINTER(JointMimic);
URDF_TYPEDEF_CLASS_POINTER(JointSafety);
URDF_TYPEDEF_CLASS_POINTER(Link);
URDF_TYPEDEF_CLASS_POINTER(Material);
URDF_TYPEDEF_CLASS_POINTER(Mesh);
URDF_TYPEDEF_CLASS_POINTER(Sphere);
URDF_TYPEDEF_CLASS_POINTER(Visual);

// create *_pointer_cast functions in urdf namespace
template<class T, class U>
boost::shared_ptr<T> const_pointer_cast(boost::shared_ptr<U> const & r)
{
  return boost::const_pointer_cast<T>(r);
}

template<class T, class U>
boost::shared_ptr<T> dynamic_pointer_cast(boost::shared_ptr<U> const & r)
{
  return boost::dynamic_pointer_cast<T>(r);
}

template<class T, class U>
boost::shared_ptr<T> static_pointer_cast(boost::shared_ptr<U> const & r)
{
  return boost::static_pointer_cast<T>(r);
}

}

#endif
