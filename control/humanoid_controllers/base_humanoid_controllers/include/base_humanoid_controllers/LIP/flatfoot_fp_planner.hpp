#pragma once
#ifndef FLATFOOT_FP_PLANNER_HPP
#define FLATFOOT_FP_PLANNER_HPP

#include <Eigen/Dense>

#include "base_humanoid_controllers/LIP/planner_types.hpp"

namespace base_humanoid_controllers
{
   /**
    * @class FlatFootFPPlanner
    * @brief A base class for flat-footed foot placement planners.
    * @ingroup group_ro_planner
    *
    * This abstract class defines the interface for flat-footed foot placement planners.
    * It provides methods for initialization, parameter updates, and plan updates.
    * Derived classes must implement these methods to provide specific planning algorithms.
    */

   class FlatFootFPPlanner
   {
   public:
      FlatFootFPPlanner() {};
      virtual ~FlatFootFPPlanner() = default;

      virtual void Init(const PlannerParams &params) = 0;

      virtual void UpdateParams(const PlannerParams &params) = 0;

      virtual PlannerOutput UpdatePlan(const PlannerInput &input) = 0;
   };

} // namespace base_humanoid_controllers

#endif // FLATFOOT_FP_PLANNER_HPP
