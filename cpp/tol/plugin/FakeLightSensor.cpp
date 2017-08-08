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

#include <string>

#include "FakeLightSensor.h"

using namespace tol;

FakeLightSensor::FakeLightSensor(std::string name,
                                 float fov,
                                 ignition::math::Vector3d light_pos)
        : revolve::brain::FakeLightSensor(fov)
        , revolve::gazebo::VirtualSensor(nullptr, name, name, 1)
        , sensor_name(name)
        , light_pos(light_pos)
{
}

FakeLightSensor::~FakeLightSensor()
{
//     std::cout << "~FakeLightSensor()" << std::endl;
}

double FakeLightSensor::light_distance()
{
  return 1 / (robot_position.Pos() - light_pos).Length();
}

double FakeLightSensor::light_angle()
{
  return 0;
}

std::string FakeLightSensor::sensorId() const
{
  return this->sensor_name;
}

void FakeLightSensor::updateRobotPosition(
        const ignition::math::Pose3d &robot_position)
{
  this->robot_position = ignition::math::Pose3d(robot_position);
}
