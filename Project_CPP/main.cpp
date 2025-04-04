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
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"

//#include "math/tiny/tiny_dual_utils.h"
using namespace tds;
using namespace TINY;

#ifdef USE_TINY

//typedef double TinyDualScalar;
//typedef double MyScalar;
//typedef ::TINY::DoubleUtils MyTinyConstants;
//typedef TinyVector3<double, DoubleUtils> Vector3;
//typedef TinyQuaternion<double, DoubleUtils> Quaternion;
//typedef TinyAlgebra<double, MyTinyConstants> MyAlgebra;

//#else
//#include "math/eigen_algebra.hpp"
//typedef EigenAlgebra MyAlgebra;
//typedef typename MyAlgebra::Scalar MyScalar;
//typedef typename MyAlgebra::Vector3 Vector3;
//typedef typename MyAlgebra::Quaternion Quaternion;
//typedef typename MyAlgebra::VectorX VectorX;
//typedef typename MyAlgebra::Matrix3 Matrix3;
//typedef typename MyAlgebra::Matrix3X Matrix3X;
//typedef typename MyAlgebra::MatrixX MatrixX;
#endif
typedef TinyDual<double> TinyDualScalar;
typedef TinyAlgebra<TinyDualScalar, TinyDualDoubleUtils> MyAlgebra;
//std::string urdf_path = "../robot_arm_3/urdf/robot_arm_3.urdf";
MyAlgebra::Vector3 gravity = MyAlgebra::Vector3(MyAlgebra::from_double(0.),
                                                 MyAlgebra::from_double(0.),
                                                 MyAlgebra::from_double(-9.81));
double dt = 0.01;
//Multiply Vector by a certain value val, elementwise
std::vector<double> multVec(std::vector<double> v1, double val)
{
  std::vector<double> newVec(v1.size());
  std::transform(v1.begin(), v1.end(), newVec.begin(), [val](double x){return x*val;});
  return newVec;
}
//Divide Vector by a certain value val, elementwise
std::vector<double> divVec(std::vector<double> v1, double val)
{
  std::vector<double> newVec(v1.size());
  std::transform(v1.begin(), v1.end(), newVec.begin(), [val](double x){return x/val;});
  return newVec;
}
//add 2 vectors element wise
std::vector<double> addVec(std::vector<double> v1, std::vector<double> v2)
{
  std::vector<double> newVec(v1.size());
  std::transform(v1.begin(), v1.end(), v2.begin(), newVec.begin(), [](double a,double b) {return a+b;});
  return newVec;
}

//Get state from mb (stacked q + dq)
//Template Algebra to easily switch from data types: Double, eigen, tinydual, etc
template <typename Algebra>
std::vector<double> get_state(tds::MultiBody<Algebra>* mb) {
  std::vector<double> state;
  for (int i = 0; i < mb->dof(); i++) {
    state.push_back(Algebra::to_double(mb->q(i)));
  }
  for (int i = 0; i < mb->dof(); i++) {
    state.push_back(Algebra::to_double(mb->qd(i)));
  }
  return state;
}
//set mb state(q+dq)
//Template Algebra to easily switch from data types: Double, eigen, tinydual, etc
template <typename Algebra>
void set_state(tds::MultiBody<Algebra>* mb, const std::vector<double>& state)
{
  for (int i = 0; i < 2*mb->dof(); i++)
  {
    if (i < mb->dof())
    {
      mb->q(i) = Algebra::from_double(state[i]);
    }
    else
    {
      mb->dq(i) = Algebra::from_double(state[i]);
    }
  }
}

//Helper function for initial setup of the visulaizer
//Template Algebra to easily switch from data types: Double, eigen, tinydual, etc
template <typename Algebra>
tds::UrdfStructures<Algebra> setup_robot(OpenGLUrdfVisualizer<Algebra>& visualizer,
  tds::UrdfCache<Algebra>& cache,
  const std::string& urdf_path,
  tds::MultiBody<Algebra>* mb, World<Algebra>& world)
  {
    
 

  mb->base_X_world().translation = typename Algebra::Vector3(Algebra::from_double(0), Algebra::from_double(0), Algebra::from_double(0));
  mb->base_X_world().rotation = Algebra::Matrix3::get_identity();

  for (int i = 0; i < mb->dof(); ++i) {
    mb->q(i) = Algebra::from_double(0.0);
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
std::cout<<"link_names"<<std::endl;
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
  //Update render with required joint angle. Call FK and resync, otherwise all urdfs will just go to origin
  //Template Algebra to easily switch from data types: Double, eigen, tinydual, etc
  template <typename Algebra>
  void update_render(OpenGLUrdfVisualizer<Algebra>& visualizer,
    tds::MultiBody<Algebra>* mb,  std::vector<double>& joint_angles){
      

      for (int i = 0; i < mb->dof(); i++) {
      mb->q(i) = Algebra::from_double(joint_angles[i]);
      }
      forward_kinematics(*mb);
      visualizer.sync_visual_transforms(mb);
      visualizer.render();
    
  }
  //RK4 integrator
  //TDS provides semi-implicit euler, so this is extra
  //Setup in a way that it can easily be ported over to the intergrator.h header file
  //Template Algebra to easily switch from data types: Double, eigen, tinydual, etc
  template <typename Algebra>
  std::vector<double> RK4_Integrator(tds::MultiBody<Algebra>* mb, bool rbdl, World<Algebra>& world){
  std::vector<double> state_origin = get_state(mb);
  //k1
  tds::forward_dynamics(*mb, world.get_gravity(), rbdl);
  std::vector<double>k1;
  for (int i = 0; i < 2*mb->dof(); i++) {
    if (i < mb->dof()){
      k1.push_back(MyAlgebra::to_double(mb->dq(i))*dt);
    }
    else{
      k1.push_back(MyAlgebra::to_double(mb->ddq(i-mb->dof()))*dt);
    }
  }
  //k2
  std::vector<double> new_x = addVec(state_origin, divVec(k1, 2));
  set_state(mb, new_x);
  std::vector<double>k2;
  tds::forward_dynamics(*mb, world.get_gravity(), rbdl);
  for (int i = 0; i < 2*mb->dof(); i++) {
    if (i < mb->dof()){
      k2.push_back(MyAlgebra::to_double(mb->dq(i))*dt);
    }
    else{
      k2.push_back(MyAlgebra::to_double(mb->ddq(i-mb->dof()))*dt);
    }
  }
  //k3
  std::vector<double> new_x2 = addVec(state_origin, divVec(k2, 2));
  set_state(mb, new_x2);
  std::vector<double>k3;
  tds::forward_dynamics(*mb, world.get_gravity(), rbdl);
  for (int i = 0; i < 2*mb->dof(); i++) {
    if (i < mb->dof()){
      k3.push_back(MyAlgebra::to_double(mb->dq(i))*dt);
    }
    else{
      k3.push_back(MyAlgebra::to_double(mb->ddq(i-mb->dof()))*dt);
    }
  }
  //k4
  std::vector<double> new_x3 = addVec(state_origin, k3);
  set_state(mb, new_x3);
  std::vector<double>k4;
  tds::forward_dynamics(*mb, world.get_gravity(), rbdl);
  for (int i = 0; i < 2*mb->dof(); i++) {
    if (i < mb->dof()){
      k4.push_back(MyAlgebra::to_double(mb->dq(i))*dt);
    }
    else{
      k4.push_back(MyAlgebra::to_double(mb->ddq(i-mb->dof()))*dt);
    }
  }
  final_state = addVec(state_origin, 
                       divVec(
                        addVec(k1, 
                          addVec(
                            multVec(k2, 2), 
                            addVec(
                              multVec(k3, 2), k4)
                            )
                          )
                        ), 
                        6.0);
  set_state(mb, state_origin);
  return final_state;
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
