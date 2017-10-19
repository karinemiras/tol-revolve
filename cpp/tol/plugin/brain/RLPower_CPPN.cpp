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
* Author: Milan Jelisavcic
* Date: March 28, 2016
*
*/

#include <map>
#include <string>
#include <vector>

#include "RLPower_CPPN.h"

#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

#include "BodyParser.h"
#include "Helper.h"
#include "brain/Conversion.h"
#include "brain/controller/ExtCPPNWeights.h"

namespace rb = revolve::brain;
namespace rg = revolve::gazebo;

using namespace tol;

RLPower_CPG::RLPower_CPG(
        const std::string &_name,
        sdf::ElementPtr _brain,
        EvaluatorPtr _evaluator,
        std::vector< rg::MotorPtr > &_actuators,
        std::vector< rg::SensorPtr > &_sensors
)
        : rb::ConverterSplitBrain< std::vector< double >, rb::PolicyPtr >
        (&rb::convertPolicyToDouble,
         &rb::convertDoubleToNull,
         _name)
{
  // initialise controller
  std::string name(_name.substr(0, _name.find("-")) + ".yaml");
  BodyParser body(name);

  std::tie(rb::InputMap, rb::OutputMap) = body.InputOutputMap(
          _actuators,
          _sensors);

  controller_ = boost::shared_ptr<rb::ExtNNController>(new rb::ExtNNController(
          _name,
          rb::convertForController(body.CoupledCpgNetwork()),
          Helper::createWrapper(_actuators),
          Helper::createWrapper(_sensors)));
  auto config = parseSDF(_brain);
  config.source_y_size = (size_t)controller_->getPhenotype().size();

  learner_ = boost::shared_ptr< rb::RLPowerLearner >(new rb::RLPowerLearner(
          _name,
          config,
          1));

  evaluator_ = _evaluator;
}

RLPower_CPG::~RLPower_CPG()
{
}

void RLPower_CPG::update(
        const std::vector< rg::MotorPtr > &actuators,
        const std::vector< rg::SensorPtr > &sensors,
        double t,
        double step)
{
  rb::ConverterSplitBrain< std::vector< double >, rb::PolicyPtr >::update(
          Helper::createWrapper(actuators),
          Helper::createWrapper(sensors),
          t,
          step);
}

rb::RLPowerLearner::Config RLPower_CPG::parseSDF(sdf::ElementPtr brain)
{
  rb::RLPowerLearner::Config config;

  // Read out brain configuration attributes
  config.algorithmType =
          brain->HasAttribute("type") ?
          brain->GetAttribute("type")->GetAsString() : "A";
  config.evaluationRate =
          brain->HasAttribute("evaluation_rate") ?
          std::stod(brain->GetAttribute("evaluation_rate")->GetAsString()) :
          rb::RLPowerLearner::EVALUATION_RATE;
  config.interpolationSplineSize =
          brain->HasAttribute("interpolation_spline_size") ?
          std::stoul(brain->GetAttribute("interpolation_spline_size")
                          ->GetAsString()) :
          rb::RLPowerLearner::INTERPOLATION_CACHE_SIZE;
  config.maxEvaluations =
          brain->HasAttribute("max_evaluations") ?
          std::stoul(brain->GetAttribute("max_evaluations")->GetAsString()) :
          rb::RLPowerLearner::MAX_EVALUATIONS;
  config.maxRankedPolicies =
          brain->HasAttribute("max_ranked_policies") ?
          std::stoul(brain->GetAttribute("max_ranked_policies")
                          ->GetAsString()) :
          rb::RLPowerLearner::MAX_RANKED_POLICIES;
  config.noiseSigma =
          brain->HasAttribute("init_sigma") ?
          std::stod(brain->GetAttribute("init_sigma")->GetAsString()) :
          rb::RLPowerLearner::SIGMA_START_VALUE;
  config.sigmaTauCorrection =
          brain->HasAttribute("sigma_tau_correction") ?
          std::stod(brain->GetAttribute("sigma_tau_correction")
                         ->GetAsString()) :
          rb::RLPowerLearner::SIGMA_TAU_CORRECTION;
  config.updateStep = 0;
  config.policyLoadPath = "";

  return config;
}
