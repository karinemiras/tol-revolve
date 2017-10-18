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

#ifndef TESTSUPGBRAINPHOTOTAXIS_H
#define TESTSUPGBRAINPHOTOTAXIS_H

#include <vector>

#include "../brain/SUPGBrainPhototaxis.h"

class testSUPGBrainPhototaxis
        : private tol::SUPGBrainPhototaxis
{
  public:
  testSUPGBrainPhototaxis(
          revolve::brain::EvaluatorPtr evaluator,
          std::function< revolve::brain::FakeLightSensor *(
                  std::vector< float > coordinates) > _light_constructor_left,
          std::function< revolve::brain::FakeLightSensor *(
                  std::vector< float > coordinates) > _light_constructor_right,
          double light_radius_distance,
          const std::vector< std::vector< float > > &neuron_coordinates,
          const std::vector< revolve::gazebo::MotorPtr > &motors,
          const std::vector< revolve::gazebo::SensorPtr > &sensors);

  void update(
          const std::vector< revolve::gazebo::MotorPtr > &motors,
          const std::vector< revolve::gazebo::SensorPtr > &sensors,
          double t,
          double step);
};

#endif  //  TESTSUPGBRAINPHOTOTAXIS_H
