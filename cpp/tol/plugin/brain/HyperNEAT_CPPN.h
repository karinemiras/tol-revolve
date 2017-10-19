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

#ifndef TOL_PLUGIN_HYPERNEAT_CPPN_H_
#define TOL_PLUGIN_HYPERNEAT_CPPN_H_

#include <string>
#include <vector>

#include <gazebo/gazebo.hh>

#include <revolve/msgs/neural_net.pb.h>
#include "revolve/gazebo/brain/Brain.h"

#include "Evaluator.h"
#include "brain/ConverterSplitBrain.h"
#include "brain/controller/ExtCPPNWeights.h"
#include "brain/learner/NEATLearner.h"
#include "brain/Types.h"

namespace rb = revolve::brain;
namespace rg = revolve::gazebo;

namespace tol
{
  class HyperNEAT_CPG
          : public rg::Brain
            , private rb::ConverterSplitBrain< rb::CPPNConfigPtr,
                                               cppneat::GeneticEncodingPtr >
  {
    public:

    /// \brief Constructor
    /// \param _name: name of the model
    /// \param _evaluator: pointer to the evaluator that is used
    /// \param node: the sdf file containing the necessary information to build
    /// the network
    /// \param _actuators: vector list of robot's actuators
    /// \param _sensors: vector list of robot's sensors
    /// \return pointer to the neural network
    HyperNEAT_CPG(
            const std::string &_name,
            sdf::ElementPtr _brain,
            tol::EvaluatorPtr _evaluator,
            const std::vector< rg::MotorPtr > &_actuators,
            const std::vector< rg::SensorPtr > &_sensors);

    /// \brief Destructor
    virtual ~HyperNEAT_CPG();

    using rb::ConverterSplitBrain< rb::CPPNConfigPtr,
                                   cppneat::GeneticEncodingPtr >::update;

    /// \brief Update sensors reading, actuators position, and `brain` state
    /// \param[inout] actuators List of actuators
    /// \param[inout] sensors List of sensors
    /// \param[in] t Time value
    /// \param[in] step Time step
    virtual void update(
            const std::vector< rg::MotorPtr > &actuators,
            const std::vector< rg::SensorPtr > &sensors,
            double t,
            double step);

    static cppneat::NEATLearner::LearningConfiguration
    parseLearningSDF(sdf::ElementPtr brain);
  };
}  /* namespace tol */

#endif  //  TOL_PLUGIN_HYPERNEAT_CPPN_H_
