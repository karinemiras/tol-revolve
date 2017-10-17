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

#ifndef MLMPCPGBRAIN_H
#define MLMPCPGBRAIN_H

#include <string>
#include <vector>

#include "brain/CPGBrain.h"
#include "Evaluator.h"
#include "revolve/gazebo/brain/Brain.h"

namespace rb  = revolve::brain;
namespace rg  = revolve::gazebo;

namespace tol
{
  class MlmpCPGBrain
          : public rg::Brain
            , private rb::CPGBrain
  {
    public:
    /// \brief Constructor
    MlmpCPGBrain(std::string robot_name,
                 EvaluatorPtr evaluator,
                 size_t n_actuators,
                 size_t n_sensors);

    /// \brief Destructor
    virtual ~MlmpCPGBrain();

    using rb::CPGBrain::update;

    /// \brief Update sensors reading, actuators position, and `brain` state
    /// \param[inout] actuators List of actuators
    /// \param[inout] sensors List of sensors
    /// \param[in] t Time value
    /// \param[in] step Time step
    virtual void update(
            const std::vector< rg::MotorPtr > &actuators,
            const std::vector< rg::SensorPtr > &sensors,
            double t,
            double step) override;
  };
}

#endif  //  MLMPCPGBRAIN_H
