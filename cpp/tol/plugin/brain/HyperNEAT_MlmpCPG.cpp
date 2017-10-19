/*
 * Copyright (C) 2015-2017 Vrije Universiteit Amsterdam
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
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
 * Date: August 2, 2017.
 *
 */

#include <utility>
#include <map>
#include <string>
#include <vector>

#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

#include "../Actuator.h"
#include "BodyParser.h"
#include "Helper.h"
#include "brain/Conversion.h"

#include "HyperNEAT_MlmpCPG.h"

namespace rb = revolve::brain;
namespace rg = revolve::gazebo;

using namespace tol;

HyperNEAT_MlmpCPG::HyperNEAT_MlmpCPG(
        std::string modelName,
        sdf::ElementPtr brain,
        tol::EvaluatorPtr evaluator,
        const std::vector< rg::MotorPtr > &actuators,
        const std::vector< rg::SensorPtr > &sensors
)
        : rb::ConverterSplitBrain< rb::CPPNConfigPtr,
                                   cppneat::GeneticEncodingPtr >
        (&rb::convertGEtoNN,
         &rb::convertNNtoGE,
         modelName)
{
  // Initialise controller
  std::string name(modelName.substr(0, modelName.find('-')) + ".yaml");
  BodyParser body(name);

  auto in_out = body.InputOutputMap(actuators, sensors);
  rb::InputMap = in_out.first;
  rb::OutputMap = in_out.second;
  rb::RafCpgNetwork = rb::convertForController(body.CoupledCpgNetwork());
  rb::neuronCoordinates = body.IdToCoordinatesMap();

  // Initialise controller
  controller_ = rb::RafCPGControllerPtr(new rb::RafCPGController(
          modelName,
          rb::RafCpgNetwork,
          Helper::createWrapper(actuators),
          Helper::createWrapper(sensors)));

  // Initialise learner
  auto learn_conf = parseLearningSDF(brain);
  rb::SetBrainSpec(true);
  learn_conf.startFrom = body.CppnNetwork();
  cppneat::MutatorPtr mutator(new cppneat::Mutator(
          rb::brain_spec,
          0.8,
          learn_conf.startFrom->RangeInnovationNumbers().second,
          100,
          std::vector< cppneat::Neuron::Ntype >()));
  auto mutator_path =
          brain->HasAttribute("path_to_mutator") ?
          brain->GetAttribute("path_to_mutator")->GetAsString() : "none";

  // initialise learner
  learner_ = boost::shared_ptr< cppneat::NEATLearner >(
          new cppneat::NEATLearner(
                  mutator,
                  mutator_path,
                  learn_conf));

  // initialise starting population
  auto numBrainsFirst =
          brain->HasAttribute("number_of_brains_from_first") ?
          std::stoi(brain->GetAttribute("number_of_brains_from_first")
                         ->GetAsString()) : 0;
  auto numBrainsSecond =
          brain->HasAttribute("number_of_brains_from_second") ?
          std::stoi(brain->GetAttribute("number_of_brains_from_second")
                         ->GetAsString()) : 0;
  auto path_to_first_brains =
          brain->HasAttribute("path_to_first_brains") ?
          brain->GetAttribute("path_to_first_brains")->GetAsString() : "";

  auto brains_from_init = boost::dynamic_pointer_cast< cppneat::NEATLearner >(
          learner_)->InitBrains();
  std::vector< cppneat::GeneticEncodingPtr > brains_from_first;
  if ("-" == path_to_first_brains or "none" == path_to_first_brains)
  {
    numBrainsFirst = 0;
  }
  else
  {
    brains_from_first = boost::dynamic_pointer_cast< cppneat::NEATLearner >(
            learner_)->YamlBrains(path_to_first_brains, -1);
  }
  auto path_to_second_brains =
          brain->HasAttribute("path_to_second_brains") ?
          brain->GetAttribute("path_to_second_brains")->GetAsString() : "";
  std::vector< cppneat::GeneticEncodingPtr > brains_from_second;
  if ("-" == path_to_second_brains or "none" == path_to_second_brains)
  {
    numBrainsSecond = 0;
  }
  else
  {
    brains_from_second = boost::dynamic_pointer_cast< cppneat::NEATLearner >(
            learner_)->YamlBrains(path_to_second_brains, -1);
  }

  std::vector< cppneat::GeneticEncodingPtr > initBrains;
  int cur_number = 0;
  for (size_t i = 0; cur_number < numBrainsFirst; ++i)
  {
    initBrains.push_back(brains_from_first[i]);
    cur_number++;
  }
  for (size_t i = 0; cur_number < numBrainsFirst + numBrainsSecond; ++i)
  {
    initBrains.push_back(brains_from_second[i]);
    cur_number++;
  }
  for (size_t i = 0; cur_number < learn_conf.popSize; ++i)
  {
    initBrains.push_back(brains_from_init[i]);
    cur_number++;
  }
  boost::dynamic_pointer_cast< cppneat::NEATLearner >(learner_)->Initialise(
          initBrains);

  // initialise evaluator
  evaluator_ = evaluator;
}

HyperNEAT_MlmpCPG::~HyperNEAT_MlmpCPG()
{
}

void HyperNEAT_MlmpCPG::update(
        const std::vector< rg::MotorPtr > &actuators,
        const std::vector< rg::SensorPtr > &sensors,
        double t,
        double step)
{
  rb::ConverterSplitBrain< boost::shared_ptr< rb::CPPNConfig >,
                           cppneat::GeneticEncodingPtr >::update(
          Helper::createWrapper(actuators),
          Helper::createWrapper(sensors),
          t,
          step);
}

cppneat::NEATLearner::LearningConfiguration
HyperNEAT_MlmpCPG::parseLearningSDF(sdf::ElementPtr brain)
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
