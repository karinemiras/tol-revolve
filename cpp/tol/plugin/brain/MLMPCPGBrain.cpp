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
* Author: Matteo De Carlo
*
*/

#include <iostream>
#include <string>
#include <vector>

#include "MLMPCPGBrain.h"
#include "Helper.h"

namespace rb  = revolve::brain;
namespace rg  = revolve::gazebo;

using namespace tol;

MlmpCPGBrain::MlmpCPGBrain(
        std::string robot_name,
        tol::EvaluatorPtr evaluator,
        size_t n_actuators,
        size_t n_sensors
)
        : rb::CPGBrain(robot_name, evaluator, n_actuators, n_sensors)
{
}

MlmpCPGBrain::~MlmpCPGBrain()
{
}

void MlmpCPGBrain::update(
        const std::vector< rg::MotorPtr > &actuators,
        const std::vector< rg::SensorPtr > &sensors,
        double t,
        double step)
{
  try
  {
    rb::CPGBrain::update(Helper::createWrapper(actuators),
                         Helper::createWrapper(sensors),
                         t,
                         step);
  } catch (std::exception &e)
  {
    std::cerr << "exception: " << e.what() << std::endl;
  }
}
