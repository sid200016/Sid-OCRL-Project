// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>
#include <thread>
#include <cmath>

#include "opengl_urdf_visualizer.h"
#include "math/tiny/tiny_double_utils.h"
#include "utils/file_utils.hpp"
#include "urdf/urdf_parser.hpp"
#include "urdf/urdf_to_multi_body.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "urdf/urdf_cache.hpp"
#include "tiny_visual_instance_generator.h"
#include "math/tiny/tiny_algebra.hpp"
#include "math/tiny/tiny_dual.h"
#include "math/tiny/tiny_dual_double_utils.h"
#include "math/tiny/tiny_dual_utils.h"
using namespace tds;
using namespace TINY;

#ifdef USE_TINY

typedef double TinyDualScalar;
typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
typedef TinyVector3<double, DoubleUtils> Vector3;
typedef TinyQuaternion<double, DoubleUtils> Quaternion;
typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;

#else
#include "math/eigen_algebra.hpp"
typedef EigenAlgebra MyAlgebra;
typedef typename MyAlgebra::Scalar MyScalar;
typedef typename MyAlgebra::Vector3 Vector3;
typedef typename MyAlgebra::Quaternion Quaternion;
typedef typename MyAlgebra::VectorX VectorX;
typedef typename MyAlgebra::Matrix3 Matrix3;
typedef typename MyAlgebra::Matrix3X Matrix3X;
typedef typename MyAlgebra::MatrixX MatrixX;
#endif
//std::string urdf_path = "../robot_arm_3/urdf/robot_arm_3.urdf";
template <typename Algebra>


//Helper function for initial setup of the 
tds::UrdfStructures<Algebra> setup_robot(OpenGLUrdfVisualizer<Algebra>& visualizer,
  tds::UrdfCache<Algebra>& cache,
  const std::string& urdf_path,
  tds::MultiBody<Algebra>* mb, World<Algebra>& world)
  {
    
 

  mb->base_X_world().translation = typename Algebra::Vector3(0, 0, 0);
  mb->base_X_world().rotation = Algebra::Matrix3::Identity();

  for (int i = 0; i < mb->dof(); ++i) {
    mb->q(i) = 0.0;
  }

  mb->initialize();
  forward_kinematics(*mb);
  std::string file_and_path;
  FileUtils::find_file(urdf_path, file_and_path);
  char search_path[TINY_MAX_EXE_PATH_LEN];
  FileUtils::extract_path(file_and_path.c_str(), search_path, TINY_MAX_EXE_PATH_LEN);
  visualizer.m_path_prefix = search_path;

  auto urdf_struct = cache.retrieve(urdf_path);
  visualizer.convert_visuals(urdf_struct, "");

  TinyVector3f pos(0, 0, 0);
  TinyQuaternionf orn(0, 0, 0, 1);
  TinyVector3f scale(1, 1, 1);
std::cout<<"base"<<std::endl;
std::cout << urdf_struct.base_links.size()<<std::endl;
std::cout<<"links"<<std::endl;
std::cout << urdf_struct.links.size()<<std::endl;
std::cout<<"dofs"<<std::endl;
std::cout<<mb->dof()<<std::endl;
std::cout<<link_names<<std::endl;
for (int j = 0; j < urdf_struct.links.size();j++)
{
    std::cout << "Link " << j << ": " << urdf_struct.links[j].link_name << std::endl;
}
  for (const auto& vis_shape : urdf_struct.base_links[0].urdf_visual_shapes) {
    if (visualizer.m_b2vis.count(vis_shape.visual_shape_uid) == 0) continue;
    const auto& vis_link = visualizer.m_b2vis[vis_shape.visual_shape_uid];
    for (int shape_id : vis_link.visual_shape_uids) {
      int instance = visualizer.m_opengl_app.m_renderer->register_graphics_instance(
        shape_id, pos, orn, {1, 1, 1}, scale);
      mb->visual_instance_uids().push_back(instance);
    }
  }

  for (int i = 0; i < mb->num_links(); ++i) {
    for (const auto& vis_shape : urdf_struct.links[i].urdf_visual_shapes) {
      if (visualizer.m_b2vis.count(vis_shape.visual_shape_uid) == 0) continue;
      const auto& vis_link = visualizer.m_b2vis[vis_shape.visual_shape_uid];
      for (int shape_id : vis_link.visual_shape_uids) {
        int instance = visualizer.m_opengl_app.m_renderer->register_graphics_instance(
          shape_id, pos, orn, {1, 1, 1}, scale);
        mb->links_[i].visual_instance_uids.push_back(instance);
      }
    }
  }
  forward_kinematics(*mb);
  visualizer.sync_visual_transforms(mb);
  visualizer.render();
  return urdf_struct;
  }
  template <typename Algebra>
  void update_render(OpenGLUrdfVisualizer<Algebra>& visualizer,
    tds::MultiBody<Algebra>* mb,  std::vector<double>& joint_angles){
      

      for (int i = 0; i < mb->dof(); ++i) {
      mb->q(i) = joint_angles[i];
      }
      forward_kinematics(*mb);
      visualizer.sync_visual_transforms(mb);
      visualizer.render();
    
  }
  //Main Loop
int main(int argc, char* argv[]) {
  UrdfCache<MyAlgebra> cache;
  World<MyAlgebra> world;
  std::string urdf_path = "../robot_arm_3/urdf/robot_arm_3.urdf";
  auto* mb = cache.construct(urdf_path, world, false); // fixed base
  OpenGLUrdfVisualizer<MyAlgebra> visualizer;
  visualizer.delete_all();
  tds::UrdfStructures<MyAlgebra> urdf_struct = setup_robot(visualizer, cache, urdf_path, mb, world);
  std::vector<double> joint_angles = {0., 3.14/2, 0.,0.,0., 0.};
  while (!visualizer.m_opengl_app.m_window->requested_exit()) {
    
    
    update_render(visualizer, mb,joint_angles);
    sleep(0.01);

  
}
return 0;
}
