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

namespace rb = revolve::brain;
namespace rg = revolve::gazebo;

using namespace tol;

SUPGBrainPhototaxis::SUPGBrainPhototaxis(
        const std::string &robot_name,
        rb::EvaluatorPtr evaluator,
        double light_radius_distance,
        const std::vector< std::vector< float>> &neuron_coordinates,
        const std::vector< rg::MotorPtr > &actuators,
        std::vector< rg::SensorPtr > &sensors)
        : rb::SUPGBrainPhototaxis(robot_name,
                                  evaluator,
                                  nullptr,
                                  nullptr,
                                  light_radius_distance,
                                  neuron_coordinates,
                                  Helper::createWrapper(actuators),
                                  createEnhancedSensorWrapper(sensors))
{
  this->light_constructor_left = [this](std::vector< float > coordinates)
          -> boost::shared_ptr< FakeLightSensor >
  {
    ignition::math::Vector3d offset(coordinates[0] / 100,
                                    coordinates[1] / 100, 0);
    ignition::math::Vector3d light_pos =
            this->robotPosition_.CoordPositionAdd(offset);
    // this function is not supposed to delete the light
    this->lightSensorLeft_.reset(new FakeLightSensor("sensor_left",
                                                      160,
                                                      light_pos));
    return this->lightSensorLeft_;
  };

  this->light_constructor_right = [this](std::vector< float > coordinates)
          -> boost::shared_ptr< FakeLightSensor >
  {
    ignition::math::Vector3d offset(coordinates[0] / 100,
                                    coordinates[1] / 100, 0);
    ignition::math::Vector3d light_pos =
            this->robotPosition_.CoordPositionAdd(offset);
    // this function is not supposed to delete the light
    this->lightSensorRight_.reset(new FakeLightSensor("sensor_right",
                                                       160,
                                                       light_pos));
    return this->lightSensorRight_;
  };

  this->light_constructor_left({1, 1});
  this->light_constructor_right({1, 1});
  sensors.push_back(this->lightSensorLeft_);
  sensors.push_back(this->lightSensorRight_);
}

SUPGBrainPhototaxis::~SUPGBrainPhototaxis()
{
}

void SUPGBrainPhototaxis::update(const std::vector< rg::MotorPtr > &motors,
                                 const std::vector< rg::SensorPtr > &sensors,
                                 double t,
                                 double step)
{
  rb::SUPGBrainPhototaxis::update(Helper::createWrapper(motors),
                                  Helper::createWrapper(sensors),
                                  t,
                                  step);
}

void SUPGBrainPhototaxis::updateRobotPosition(
        ignition::math::Pose3d &robot_position)
{
  this->robotPosition_ = ignition::math::Pose3d(robot_position);
  this->lightSensorLeft_->updateRobotPosition(robot_position);
  this->lightSensorRight_->updateRobotPosition(robot_position);
}

const std::vector< rb::SensorPtr >
tol::SUPGBrainPhototaxis::createEnhancedSensorWrapper(
        const std::vector< rg::SensorPtr > &original)
{
  std::vector< rb::SensorPtr > result = Helper::createWrapper(original);
  result.push_back(boost::make_shared< tol::FakeLightSensor >(
          "sensor_1_fake_filler", 0, ignition::math::Vector3d()));
  result.push_back(boost::make_shared< tol::FakeLightSensor >(
          "sensor_2_fake_filler", 0, ignition::math::Vector3d()));

  return result;
}
