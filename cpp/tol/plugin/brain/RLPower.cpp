/*
* Copyright (C) 2017 Vrije Universiteit Amsterdam
*
* Licensed under the Apache License, Version 2.0 (the "License");
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* Description: TODO: <Add brief description about file purpose>
* Description: Milan Jelisavcic
* Author: Marsh 28, 2016
*
*/

#include <string>
#include <vector>

#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

#include "Helper.h"
#include "RLPower.h"

using namespace tol;

RLPower::RLPower(std::string modelName,
                 sdf::ElementPtr brain,
                 EvaluatorPtr evaluator,
                 std::vector< revolve::gazebo::MotorPtr > &actuators,
                 std::vector< revolve::gazebo::SensorPtr > &sensors)
        : revolve::brain::RLPower(modelName,
                                  parseSDF(brain),
                                  evaluator,
                                  actuators.size())
{
}

RLPower::~RLPower()
{
}

void RLPower::update(const std::vector< revolve::gazebo::MotorPtr > &actuators,
                     const std::vector< revolve::gazebo::SensorPtr > &sensors,
                     double t,
                     double step)
{
  revolve::brain::RLPower::update(Helper::createWrapper(actuators),
                                  Helper::createWrapper(sensors),
                                  t,
                                  step);
}

RLPower::Config RLPower::parseSDF(sdf::ElementPtr brain)
{
  // Read out brain configuration attributes
  Config config;

  config.algorithm_type =
          brain->HasAttribute("type") ?
          brain->GetAttribute("type")->GetAsString() : "A";
  config.evaluation_rate =
          brain->HasAttribute("evaluation_rate") ?
          std::stod(brain->GetAttribute("evaluation_rate")->GetAsString()) :
          RLPower::EVALUATION_RATE;
  config.interpolation_spline_size =
          brain->HasAttribute("interpolation_spline_size") ?
          std::stoul(brain->GetAttribute("interpolation_spline_size")
                          ->GetAsString()) :
          RLPower::INTERPOLATION_CACHE_SIZE;
  config.max_evaluations =
          brain->HasAttribute("max_evaluations") ?
          std::stoul(brain->GetAttribute("max_evaluations")->GetAsString()) :
          RLPower::MAX_EVALUATIONS;
  config.max_ranked_policies =
          brain->HasAttribute("max_ranked_policies") ?
          std::stoul(brain->GetAttribute("max_ranked_policies")
                          ->GetAsString()) :
          RLPower::MAX_RANKED_POLICIES;
  config.noise_sigma =
          brain->HasAttribute("init_sigma") ?
          std::stod(brain->GetAttribute("init_sigma")->GetAsString()) :
          RLPower::SIGMA_START_VALUE;
  config.sigma_tau_correction =
          brain->HasAttribute("sigma_tau_correction") ?
          std::stod(brain->GetAttribute("sigma_tau_correction")
                         ->GetAsString()) :
          RLPower::SIGMA_TAU_CORRECTION;
  config.source_y_size =
          brain->HasAttribute("init_spline_size") ?
          std::stoul(brain->GetAttribute("init_spline_size")->GetAsString()) :
          RLPower::INITIAL_SPLINE_SIZE;
  config.update_step =
          brain->HasAttribute("update_step") ?
          std::stoul(brain->GetAttribute("update_step")->GetAsString()) :
          RLPower::UPDATE_STEP;
  config.policy_load_path =
          brain->HasAttribute("policy_load_path") ?
          brain->GetAttribute("policy_load_path")->GetAsString() : "";

  return config;
}
