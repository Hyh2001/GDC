// Translated from mujoco/simulate/main.cc
// Reformulate it as a class exposing access to the "sim" object for external use

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include "glfw_adapter.h"
#include "simulate.h"
#include "array_safety.h"

#ifndef MUJOCO_SIMULATE_MUJOCO_PREBUILT_GUI_H_
#define MUJOCO_SIMULATE_MUJOCO_PREBUILT_GUI_H_

namespace mujoco_sim{

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #if defined(__APPLE__)
    #include <mach-o/dyld.h>
  #endif
  #include <sys/errno.h>
  #include <unistd.h>
#endif
}

namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

using Seconds = std::chrono::duration<double>;

// constants
const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;          // load error string length

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char* title, const char* msg);
static const char* rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char* msg) {
  rosetta_error_msg = msg;
}
#endif

class MujocoSim{
public:
  MujocoSim(const std::string& mjcf_path="");
  // expose sim ptr for external use (eg: control IO)
  mj::Simulate* getSimPtr() { return sim_ptr.get(); };
  // this function runs physics simulation
  void PhysicsThread();

private:
  // plugin handling
  std::string getExecutableDir();
  void scanPluginLibraries();
  // simulation
  const char* Diverged(int disableflags, const mjData* d);
  mjModel* LoadModel(const char* file, mj::Simulate& sim);
  void PhysicsLoop(mj::Simulate& sim);

  // model and data
  mjModel* m = nullptr;
  mjData* d = nullptr;

  // mjcf file name
  std::string mjcf_path_;

  // camera, option, perturb
  mjvCamera cam_;
  mjvOption opt_;
  mjvPerturb pert_;
  
  // ptr to mj::Simulate object
  std::unique_ptr<mj::Simulate> sim_ptr;

};
}  // namespace mujoco_sim

#endif  // MUJOCO_SIMULATE_MUJOCO_PREBUILT_GUI_H_