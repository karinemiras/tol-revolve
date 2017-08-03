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

//
// Created by matteo on 3/15/17.
//

#include "GenericLearnerBrain.h"
#include "Helper.h"

using namespace tol;

GenericLearnerBrain::GenericLearnerBrain(std::unique_ptr<revolve::brain::BaseLearner> learner)
    : revolve::brain::GenericLearnerBrain(std::move(learner))
{}

GenericLearnerBrain::GenericLearnerBrain(revolve::brain::BaseLearner *learner)
    : revolve::brain::GenericLearnerBrain(learner)
{}

void GenericLearnerBrain::update(const std::vector<revolve::gazebo::MotorPtr> &actuators,
                                 const std::vector<revolve::gazebo::SensorPtr> &sensors,
                                 double t, double step)
{
  revolve::brain::GenericLearnerBrain::update(
      Helper::createWrapper(actuators),
      Helper::createWrapper(sensors),
      t,
      step
  );
}
