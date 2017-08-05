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

#include "revolve/gazebo/sensors/VirtualSensor.h"
#include "revolve/gazebo/motors/Motor.h"

#include "../Sensor.h"
#include "../Actuator.h"
#include "Helper.h"

#include "SUPGBrain.h"

using namespace tol;

SUPGBrain::SUPGBrain(
        const std::string &robot_name,
        revolve::brain::EvaluatorPtr evaluator,
        const std::vector< std::vector< float > > &neuron_coordinates,
        const std::vector< revolve::gazebo::MotorPtr > &motors,
        const std::vector< revolve::gazebo::SensorPtr > &sensors)
        : revolve::brain::SUPGBrain(
        robot_name,
        evaluator,
        neuron_coordinates,
        Helper::createWrapper(motors),
        Helper::createWrapper(sensors))
{
  std::cerr << "tol::SUPGBrain::SUPGBrain()" << std::endl;
}


SUPGBrain::~SUPGBrain()
{
}

void tol::SUPGBrain::update(
        const std::vector< revolve::gazebo::MotorPtr > &motors,
        const std::vector< revolve::gazebo::SensorPtr > &sensors,
        double t,
        double step)
{
  revolve::brain::SUPGBrain::update(
          Helper::createWrapper(motors),
          Helper::createWrapper(sensors),
          t, step);
}
