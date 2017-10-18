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

#include <map>
#include <string>
#include <vector>

#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

#include "../Actuator.h"
#include "BodyParser.h"
#include "Helper.h"
#include "brain/Conversion.h"

#include "NEAT_CPPN.h"

using namespace tol;

NeatExtNN::NeatExtNN(
        std::string modelName,
        sdf::ElementPtr node,
        tol::EvaluatorPtr evaluator,
        const std::vector< revolve::gazebo::MotorPtr > &actuators,
        const std::vector< revolve::gazebo::SensorPtr > &sensors)
        : revolve::brain::ConverterSplitBrain
        < boost::shared_ptr< revolve::brain::CPPNConfig >,
          cppneat::GeneticEncodingPtr >(
        &revolve::brain::convertForController,
        &revolve::brain::convertForLearner,
        modelName)
{
  // initialise controller
  std::string name(modelName.substr(0, modelName.find("-")) + ".yaml");
  BodyParser body(name);
  auto in_out = body.InputOutputMap(actuators, sensors);
  revolve::brain::InputMap = in_out.first;
  revolve::brain::OutputMap = in_out.second;
  auto learn_conf = parseLearningSDF(node);
  learn_conf.start_from = body.CoupledCpgNetwork();
  revolve::brain::RafCPGControllerPtr new_controller(
          new revolve::brain::RafCPGController(
                  modelName,
                  revolve::brain::convertForController(learn_conf.start_from),
                  Helper::createWrapper(actuators),
                  Helper::createWrapper(sensors)));

  auto innovationNumber = body.InnovationNumber();
  controller_ = new_controller;

  // initialise learner
  revolve::brain::SetBrainSpec(false);
  cppneat::MutatorPtr mutator(new cppneat::Mutator(
          revolve::brain::brain_spec,
          0.8,
          innovationNumber,
          100,
          std::vector< cppneat::Neuron::Ntype >()));
  auto mutator_path =
          node->HasAttribute("path_to_mutator") ?
          node->GetAttribute("path_to_mutator")->GetAsString() : "none";

  learner_ = boost::shared_ptr< cppneat::NEATLearner >(new cppneat::NEATLearner(
          mutator,
          mutator_path,
          learn_conf));

  evaluator_ = evaluator;
}

NeatExtNN::~NeatExtNN()
{
}

void NeatExtNN::update(
        const std::vector< revolve::gazebo::MotorPtr > &actuators,
        const std::vector< revolve::gazebo::SensorPtr > &sensors,
        double t,
        double step)
{
  revolve::brain::ConverterSplitBrain< revolve::brain::CPPNConfigPtr,
                                       cppneat::GeneticEncodingPtr >::update(
          Helper::createWrapper(actuators),
          Helper::createWrapper(sensors),
          t,
          step);
}

cppneat::NEATLearner::LearningConfiguration NeatExtNN::parseLearningSDF(
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
