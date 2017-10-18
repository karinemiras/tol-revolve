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

#ifndef SUPGBRAIN_H
#define SUPGBRAIN_H

#include <vector>
#include <memory>
#include <string>

#include <gazebo/gazebo.hh>

#include "Evaluator.h"
#include "brain/SUPGBrain.h"
#include "revolve/gazebo/brain/Brain.h"

namespace tol
{
  class SUPGBrain
          : public revolve::gazebo::Brain
            , private revolve::brain::SUPGBrain
  {
    public:
    /// \brief Constructor
    SUPGBrain(
            const std::string &robot_name,
            revolve::brain::EvaluatorPtr evaluator,
            const std::vector< std::vector< float > > &neuron_coordinates,
            const std::vector< revolve::gazebo::MotorPtr > &motors,
            const std::vector< revolve::gazebo::SensorPtr > &sensors);

    /// \brief Destructor
    ~SUPGBrain();

    using revolve::brain::SUPGBrain::update;

    /// \brief Update sensors reading, actuators position, and `brain` state
    /// \param[inout] actuators List of actuators
    /// \param[inout] sensors List of sensors
    /// \param[in] t Time value
    /// \param[in] step Time step
    virtual void update(
            const std::vector< revolve::gazebo::MotorPtr > &motors,
            const std::vector< revolve::gazebo::SensorPtr > &sensors,
            double t,
            double step) override;
  };
}

#endif  //  SUPGBRAIN_H
