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
#include <vector>

#include <boost/make_shared.hpp>

#include "Helper.h"

#include "SUPGBrainPhototaxis.h"

using namespace tol;

SUPGBrainPhototaxis::SUPGBrainPhototaxis(
        const std::string &robot_name,
        revolve::brain::EvaluatorPtr evaluator,
        double light_radius_distance,
        const std::vector< std::vector< float>> &neuron_coordinates,
        const std::vector< revolve::gazebo::MotorPtr > &actuators,
        std::vector< revolve::gazebo::SensorPtr > &sensors)
        : revolve::brain::SUPGBrainPhototaxis(
        robot_name,
        evaluator,
        nullptr,
        nullptr,
        light_radius_distance,
        neuron_coordinates,
        Helper::createWrapper(actuators),
        createEnhancedSensorWrapper(sensors))
{
  light_constructor_left = [this](std::vector< float > coordinates)
          -> boost::shared_ptr< FakeLightSensor >
  {
    ignition::math::Vector3d
            offset(coordinates[0] / 100, coordinates[1] / 100, 0);
    ignition::math::Vector3d
            light_pos = this->robot_position.CoordPositionAdd(offset);
    // this function is not supposed to delete the light
    light_sensor_left.reset(new FakeLightSensor("sensor_left", 160, light_pos));
    return light_sensor_left;
  };

  light_constructor_right = [this](std::vector< float > coordinates)
          -> boost::shared_ptr< FakeLightSensor >
  {
    ignition::math::Vector3d
            offset(coordinates[0] / 100, coordinates[1] / 100, 0);
    ignition::math::Vector3d
            light_pos = this->robot_position.CoordPositionAdd(offset);
    // this function is not supposed to delete the light
    light_sensor_right.reset(
            new FakeLightSensor("sensor_right", 160, light_pos));
    return light_sensor_right;
  };

  light_constructor_left({1, 1});
  light_constructor_right({1, 1});
  sensors.push_back(light_sensor_left);
  sensors.push_back(light_sensor_right);
}

SUPGBrainPhototaxis::~SUPGBrainPhototaxis()
{}

void SUPGBrainPhototaxis::update(
        const std::vector< revolve::gazebo::MotorPtr > &motors,
        const std::vector< revolve::gazebo::SensorPtr > &sensors,
        double t,
        double step)
{
  revolve::brain::SUPGBrainPhototaxis::update(Helper::createWrapper(motors),
                                              Helper::createWrapper(sensors),
                                              t, step);
}

void SUPGBrainPhototaxis::updateRobotPosition(
        ignition::math::Pose3d &robot_position)
{
  this->robot_position = ignition::math::Pose3d(robot_position);
  light_sensor_left->updateRobotPosition(robot_position);
  light_sensor_right->updateRobotPosition(robot_position);
}

const std::vector< revolve::brain::SensorPtr >
tol::SUPGBrainPhototaxis::createEnhancedSensorWrapper(
        const std::vector< revolve::gazebo::SensorPtr > &original)
{
  std::vector< revolve::brain::SensorPtr >
          result = Helper::createWrapper(original);
  result.push_back(boost::make_shared< tol::FakeLightSensor >(
          "sensor_1_fake_filler", 0, ignition::math::Vector3d()));
  result.push_back(boost::make_shared< tol::FakeLightSensor >(
          "sensor_2_fake_filler", 0, ignition::math::Vector3d()));

  return result;
}
