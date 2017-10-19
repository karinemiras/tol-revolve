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

namespace rb = revolve::brain;
namespace rg = revolve::gazebo;

using namespace tol;

NeatExtNN::NeatExtNN(
        std::string modelName,
        sdf::ElementPtr node,
        tol::EvaluatorPtr evaluator,
        const std::vector< rg::MotorPtr > &actuators,
        const std::vector< rg::SensorPtr > &sensors
)
        : rb::ConverterSplitBrain
        < boost::shared_ptr< rb::CPPNConfig >, cppneat::GeneticEncodingPtr >(
        &rb::convertForController,
        &rb::convertForLearner,
        modelName)
{
  // initialise controller
  std::string name(modelName.substr(0, modelName.find("-")) + ".yaml");
  BodyParser body(name);
  std::tie(rb::InputMap, rb::OutputMap) = body.InputOutputMap(
          actuators,
          sensors);

  auto learn_conf = parseLearningSDF(node);
  learn_conf.startFrom = body.CoupledCpgNetwork();
  rb::RafCPGControllerPtr new_controller(new rb::RafCPGController(
          modelName,
          rb::convertForController(learn_conf.startFrom),
          Helper::createWrapper(actuators),
          Helper::createWrapper(sensors)));

  auto innovationNumber = body.InnovationNumber();
  this->controller_ = new_controller;

  // initialise learner
  rb::SetBrainSpec(false);
  cppneat::MutatorPtr mutator(new cppneat::Mutator(
          rb::brain_spec,
          0.8,
          innovationNumber,
          100,
          std::vector< cppneat::Neuron::Ntype >()));
  auto mutator_path =
          node->HasAttribute("path_to_mutator") ?
          node->GetAttribute("path_to_mutator")->GetAsString() : "none";

  this->learner_ = boost::shared_ptr< cppneat::NEATLearner >(
          new cppneat::NEATLearner(
                  mutator,
                  mutator_path,
                  learn_conf));

  this->evaluator_ = evaluator;
}

NeatExtNN::~NeatExtNN()
{
}

void NeatExtNN::update(
        const std::vector< rg::MotorPtr > &actuators,
        const std::vector< rg::SensorPtr > &sensors,
        double t,
        double step)
{
  rb::ConverterSplitBrain< rb::CPPNConfigPtr,
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
  config.popSize =
          brain->HasAttribute("pop_size") ?
          std::stoi(brain->GetAttribute("pop_size")->GetAsString()) :
          cppneat::NEATLearner::POP_SIZE;
  config.tournamentSize =
          brain->HasAttribute("tournament_size") ?
          std::stoi(brain->GetAttribute("tournament_size")->GetAsString()) :
          cppneat::NEATLearner::TOURNAMENT_SIZE;
  config.numChildren =
          brain->HasAttribute("num_children") ?
          std::stoi(brain->GetAttribute("num_children")->GetAsString()) :
          cppneat::NEATLearner::NUM_CHILDREN;
  config.weightMutationProbability =
          brain->HasAttribute("weight_mutation_probability") ?
          std::stod(brain->GetAttribute("weight_mutation_probability")
                         ->GetAsString()) :
          cppneat::NEATLearner::WEIGHT_MUTATION_PROBABILITY;
  config.weightMutationSigma =
          brain->HasAttribute("weight_mutation_sigma") ?
          std::stod(brain->GetAttribute("weight_mutation_sigma")
                         ->GetAsString()) :
          cppneat::NEATLearner::WEIGHT_MUTATION_SIGMA;
  config.paramMutationProbability =
          brain->HasAttribute("param_mutation_probability") ?
          std::stod(brain->GetAttribute("param_mutation_probability")
                         ->GetAsString()) :
          cppneat::NEATLearner::PARAM_MUTATION_PROBABILITY;
  config.paramMutationSigma =
          brain->HasAttribute("param_mutation_sigma") ?
          std::stod(brain->GetAttribute("param_mutation_sigma")
                         ->GetAsString()) :
          cppneat::NEATLearner::PARAM_MUTATION_SIGMA;
  config.structuralAugmentationProbability =
          brain->HasAttribute("structural_augmentation_probability") ?
          std::stod(brain->GetAttribute("structural_augmentation_probability")
                         ->GetAsString()) :
          cppneat::NEATLearner::STRUCTURAL_AUGMENTATION_PROBABILITY;
  config.structuralRemovalProbability =
          brain->HasAttribute("structural_removal_probability") ?
          std::stod(brain->GetAttribute("structural_removal_probability")
                         ->GetAsString()) :
          cppneat::NEATLearner::STRUCTURAL_REMOVAL_PROBABILITY;
  config.maxGenerations =
          brain->HasAttribute("max_generations") ?
          std::stoi(brain->GetAttribute("max_generations")->GetAsString()) :
          cppneat::NEATLearner::MAX_GENERATIONS;
  config.speciationThreshold =
          brain->HasAttribute("speciation_threshold") ?
          std::stod(brain->GetAttribute("speciation_threshold")
                         ->GetAsString()) :
          cppneat::NEATLearner::SPECIATION_TRESHOLD;
  config.repeat_evaluations =
          brain->HasAttribute("repeat_evaluations") ?
          std::stoi(brain->GetAttribute("repeat_evaluations")->GetAsString()) :
          cppneat::NEATLearner::REPEAT_EVALUATIONS;
  config.initialStructuralMutations =
          brain->HasAttribute("initial_structural_mutations") ?
          std::stoi(brain->GetAttribute("initial_structural_mutations")
                         ->GetAsString()) :
          cppneat::NEATLearner::INITIAL_STRUCTURAL_MUTATIONS;
  config.interspeciesMateProbability =
          brain->HasAttribute("interspecies_mate_probability") ?
          std::stod(brain->GetAttribute("interspecies_mate_probability")
                         ->GetAsString()) :
          cppneat::NEATLearner::INTERSPECIES_MATE_PROBABILITY;
  return config;
}
