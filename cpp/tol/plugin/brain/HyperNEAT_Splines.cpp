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
* Author: TODO <Add proper author>
*
*/

#include <string>
#include <vector>

#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

#include "../Actuator.h"
#include "BodyParser.h"
#include "Helper.h"
#include "brain/Conversion.h"

#include "HyperNEAT_Splines.h"

namespace rb = revolve::brain;
namespace rg = revolve::gazebo;

using namespace tol;

HyperNEAT_Splines::HyperNEAT_Splines(std::string modelName,
        sdf::ElementPtr brain,
        tol::EvaluatorPtr evaluator,
        const std::vector< rg::MotorPtr > &actuators,
        const std::vector< rg::SensorPtr > &sensors)
        : rb::ConverterSplitBrain< rb::PolicyPtr, cppneat::GeneticEncodingPtr >
                  (&rb::convertForSplinesFromHyper,
                   &rb::convertForHyperFromSplines,
                   modelName)
{
//  sleep(20);
  std::string name(modelName.substr(0, modelName.find("-")) + ".yaml");
  BodyParser body(name);
  rb::sorted_coordinates = body.SortedCoordinates(actuators);

  // Prepare for controller
  rb::RLPowerLearner::Config conf = parseSDF(brain);
  rb::update_rate = conf.update_step;
  rb::spline_size = conf.source_y_size;
  std::string mutator_path =
          brain->HasAttribute("path_to_mutator") ?
          brain->GetAttribute("path_to_mutator")->GetAsString() : "none";

  // Prepare for learner
  cppneat::NEATLearner::LearningConfiguration
          learn_conf = parseLearningSDF(brain);
  cppneat::MutatorPtr mutator(
          new cppneat::Mutator(rb::brain_spec,
                               0.8,
                               learn_conf.start_from
                                                   ->RangeInnovationNumbers().second,
                               100,
                               std::vector< cppneat::Neuron::Ntype >()));

  rb::SetBrainSpec(true);
  learn_conf.start_from = rb::HyperNeatSplines();

  // initialise controller
  controller_ = boost::shared_ptr< rb::PolicyController >(
          new rb::PolicyController(
                  rb::sorted_coordinates.size(),
                  conf.interpolation_spline_size));

  // initialise learner
  learner_ = boost::shared_ptr< cppneat::NEATLearner >
          (new cppneat::NEATLearner(mutator, mutator_path, learn_conf));

  // initialise evaluator
  evaluator_ = evaluator;
}

HyperNEAT_Splines::~HyperNEAT_Splines()
{
}

void HyperNEAT_Splines::update(const std::vector< rg::MotorPtr > &actuators,
                               const std::vector< rg::SensorPtr > &sensors,
                               double t,
                               double step)
{
  rb::ConverterSplitBrain< rb::PolicyPtr, cppneat::GeneticEncodingPtr >::update(
          Helper::createWrapper(actuators),
          Helper::createWrapper(sensors),
          t,
          step);
}

rb::RLPowerLearner::Config HyperNEAT_Splines::parseSDF(sdf::ElementPtr brain)
{
  rb::RLPowerLearner::Config config;

  // Read out brain configuration attributes
  config.evaluation_rate =
          brain->HasAttribute("evaluation_rate") ?
          std::stod(brain->GetAttribute("evaluation_rate")->GetAsString()) :
          rb::RLPowerLearner::EVALUATION_RATE;
  config.interpolation_spline_size =
          brain->HasAttribute("interpolation_spline_size") ?
          std::stoul(brain->GetAttribute("interpolation_spline_size")
                          ->GetAsString()) :
          rb::RLPowerLearner::INTERPOLATION_CACHE_SIZE;
  config.source_y_size =
          brain->HasAttribute("init_spline_size") ?
          std::stoul(brain->GetAttribute("init_spline_size")->GetAsString()) :
          rb::RLPowerLearner::INITIAL_SPLINE_SIZE;
  config.update_step =
          brain->HasAttribute("update_step") ?
          std::stoul(brain->GetAttribute("update_step")->GetAsString()) :
          rb::RLPowerLearner::UPDATE_STEP;

  return config;
}

cppneat::NEATLearner::LearningConfiguration HyperNEAT_Splines::parseLearningSDF(
        sdf::ElementPtr brain)
{
  cppneat::NEATLearner::LearningConfiguration config;

  // Read out brain configuration attributes
  config.asexual =
          brain->HasAttribute("asexual") ?
          (brain->GetAttribute("asexual")->GetAsString() == "true") :
          cppneat::NEATLearner::ASEXUAL;
  config.pop_size =
          brain->HasAttribute("pop_size") ?
          std::stoi(brain->GetAttribute("pop_size")->GetAsString()) :
          cppneat::NEATLearner::POP_SIZE;
  config.tournament_size =
          brain->HasAttribute("tournament_size") ?
          std::stoi(brain->GetAttribute("tournament_size")->GetAsString()) :
          cppneat::NEATLearner::TOURNAMENT_SIZE;
  config.num_children =
          brain->HasAttribute("num_children") ?
          std::stoi(brain->GetAttribute("num_children")->GetAsString()) :
          cppneat::NEATLearner::NUM_CHILDREN;
  config.weight_mutation_probability =
          brain->HasAttribute("weight_mutation_probability") ?
          std::stod(brain->GetAttribute("weight_mutation_probability")
                         ->GetAsString()) :
          cppneat::NEATLearner::WEIGHT_MUTATION_PROBABILITY;
  config.weight_mutation_sigma =
          brain->HasAttribute("weight_mutation_sigma") ?
          std::stod(brain->GetAttribute("weight_mutation_sigma")
                         ->GetAsString()) :
          cppneat::NEATLearner::WEIGHT_MUTATION_SIGMA;
  config.param_mutation_probability =
          brain->HasAttribute("param_mutation_probability") ?
          std::stod(brain->GetAttribute("param_mutation_probability")
                         ->GetAsString()) :
          cppneat::NEATLearner::PARAM_MUTATION_PROBABILITY;
  config.param_mutation_sigma =
          brain->HasAttribute("param_mutation_sigma") ?
          std::stod(brain->GetAttribute("param_mutation_sigma")
                                               ->GetAsString()) :
          cppneat::NEATLearner::PARAM_MUTATION_SIGMA;
  config.structural_augmentation_probability =
          brain->HasAttribute("structural_augmentation_probability") ?
          std::stod(brain->GetAttribute("structural_augmentation_probability")
                         ->GetAsString()) :
          cppneat::NEATLearner::STRUCTURAL_AUGMENTATION_PROBABILITY;
  config.structural_removal_probability =
          brain->HasAttribute("structural_removal_probability") ?
          std::stod(brain->GetAttribute("structural_removal_probability")
                         ->GetAsString()) :
          cppneat::NEATLearner::STRUCTURAL_REMOVAL_PROBABILITY;
  config.max_generations =
          brain->HasAttribute("max_generations") ?
          std::stoi(brain->GetAttribute("max_generations")->GetAsString()) :
          cppneat::NEATLearner::MAX_GENERATIONS;
  config.speciation_threshold =
          brain->HasAttribute("speciation_threshold") ?
          std::stod(brain->GetAttribute("speciation_threshold")
                         ->GetAsString()) :
          cppneat::NEATLearner::SPECIATION_TRESHOLD;
  config.repeat_evaluations =
          brain->HasAttribute("repeat_evaluations") ?
          std::stoi(brain->GetAttribute("repeat_evaluations")->GetAsString()) :
          cppneat::NEATLearner::REPEAT_EVALUATIONS;
  config.initial_structural_mutations =
          brain->HasAttribute("initial_structural_mutations") ?
          std::stoi(brain->GetAttribute("initial_structural_mutations")
                         ->GetAsString()) :
          cppneat::NEATLearner::INITIAL_STRUCTURAL_MUTATIONS;
  config.interspecies_mate_probability =
          brain->HasAttribute("interspecies_mate_probability") ?
          std::stod(brain->GetAttribute("interspecies_mate_probability")
                         ->GetAsString()) :
          cppneat::NEATLearner::INTERSPECIES_MATE_PROBABILITY;
  return config;
}