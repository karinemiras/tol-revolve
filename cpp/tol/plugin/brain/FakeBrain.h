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

#ifndef TOL_PLUGIN_FAKEBRAIN_H_
#define TOL_PLUGIN_FAKEBRAIN_H_

#include <string>
#include <vector>

#include <gazebo/gazebo.hh>

#include <revolve/msgs/neural_net.pb.h>
#include "revolve/gazebo/brain/Brain.h"

namespace tol
{
  class FakeBrain
          : public revolve::gazebo::Brain
  {
    public:
    typedef const boost::shared_ptr<revolve::msgs::ModifyNeuralNetwork const>
            ConstModifyNeuralNetworkPtr;

    FakeBrain(std::string modelName,
              std::vector<revolve::gazebo::MotorPtr> &actuators,
              std::vector<revolve::gazebo::SensorPtr> &sensors);

    virtual ~FakeBrain();

    /**
     /// \param Motor list
     /// \param Sensor list
     */
    virtual void
    update(const std::vector<revolve::gazebo::MotorPtr> &motors,
           const std::vector<revolve::gazebo::SensorPtr> &sensors,
           double t,
           double step);

    protected:
    /**
     * Request handler to modify the neural network
     */
    void
    modify(ConstModifyNeuralNetworkPtr &req);

    // Mutex for stepping / updating the network§
    // boost::mutex networkMutex_;

    size_t nActuators_;
    size_t nSensors_;

    double start_eval_time_;

// private:
//    double cycle_start_time_;

    /**
     * Transport node
     */
//     ::gazebo::transport::NodePtr node_;

    /**
     * Network modification subscriber
     */
//     ::gazebo::transport::SubscriberPtr alterSub_;
  };
} /* namespace tol */

#endif  //  TOL_PLUGIN_FAKEBRAIN_H_
