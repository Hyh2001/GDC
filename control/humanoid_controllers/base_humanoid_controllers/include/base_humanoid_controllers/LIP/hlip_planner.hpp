#pragma once
#ifndef BIPED_PLANNER_HLIPPLANNER_HPP
#define BIPED_PLANNER_HLIPPLANNER_HPP

#include "base_humanoid_controllers/LIP/flatfoot_fp_planner.hpp"
#include "base_humanoid_controllers/LIP/hlip.hpp"
#include "base_humanoid_controllers/LIP/planner_types.hpp"

namespace base_humanoid_controllers
{
   /**
    * @class HLIPPlanner
    * @brief A class for planning using the Hybrid Linear Inverted Pendulum (HLIP) model.
    * @ingroup group_ro_planner
    */
   class HLIPPlanner : public FlatFootFPPlanner
   {
   public:
      HLIPPlanner();
      // Constructor that initializes the HLIPPlanner with given parameters
      HLIPPlanner(const PlannerParams &params);
      ~HLIPPlanner() override = default;
      // Initialize the HLIPPlanner with parameters
      void Init(const PlannerParams &params) override;

      void UpdateParams(const PlannerParams &params) override;

      PlannerOutput UpdatePlan(const PlannerInput &input) override;

   protected:
      HLIP HLIP_sag;
      HLIP HLIP_lat;
   };
} // namespace base_humanoid_controllers

#endif // BIPED_PLANNER_HLIPPLANNER_HPP
