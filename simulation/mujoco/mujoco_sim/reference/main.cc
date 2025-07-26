// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mujoco_sim.h"

using namespace mujoco_sim;

int main(int argc, char** argv) {

  std::string mjcf_path = "";
  if (argc >  1) {
    mjcf_path = std::string(argv[1]);
  }

  MujocoSim mujoco_sim(mjcf_path);
  
  // thread starting sequence: physics -> render
  std::thread physicsthreadhandle(&MujocoSim::PhysicsThread, &mujoco_sim);
  mj::Simulate* sim_ptr = mujoco_sim.getSimPtr();
  sim_ptr->RenderLoop();

  // thread ending sequence: render -> physics
  physicsthreadhandle.join();
  
  return 0;
}