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
* Date: March 15, 2017
*
*/

#ifndef TRIANGLEOFLIFE_GENERICLEARNERBRAIN_H
#define TRIANGLEOFLIFE_GENERICLEARNERBRAIN_H

#include <vector>

#include <revolve/gazebo/brain/Brain.h>
#include "brain/GenericLearnerBrain.h"

namespace tol
{
  class GenericLearnerBrain
          : public revolve::gazebo::Brain
            , private revolve::brain::GenericLearnerBrain
  {
    public:

    GenericLearnerBrain(std::unique_ptr< revolve::brain::BaseLearner > learner);

    GenericLearnerBrain(revolve::brain::BaseLearner *learner);

    using revolve::brain::GenericLearnerBrain::update;

    /// \brief Update sensors reading, actuators position, and `brain` state
    /// \param[inout] actuators List of actuators
    /// \param[inout] sensors List of sensors
    /// \param[in] t Time value
    /// \param[in] step Time step
    void update(const std::vector< revolve::gazebo::MotorPtr > &actuators,
                const std::vector< revolve::gazebo::SensorPtr > &sensors,
                double t,
                double step) override;
  };
}

#endif  // TRIANGLEOFLIFE_GENERICLEARNERBRAIN_H