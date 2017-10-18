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

#include "FakeBrain.h"

#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

using namespace tol;

FakeBrain::FakeBrain(
        std::string /*_name*/,
        std::vector< revolve::gazebo::MotorPtr > &actuators,
        std::vector< revolve::gazebo::SensorPtr > &sensors
)
        : nActuators_(actuators.size())
        , nSensors_(sensors.size())
        , start_eval_time_(0)
{
  std::cout << "FakeBrain::FakeBrain()" << std::endl;

// // Create transport node
// node_.reset(new ::gazebo::transport::Node());
// node_->Init();
//
// // Listen to network modification requests
// alterSub_ = node_->Subscribe("~/" + modelName
//             + "/modify_neural_network", &FakeBrain::modify, this);
}

FakeBrain::~FakeBrain()
{
}

void FakeBrain::modify(ConstModifyNeuralNetworkPtr &/*req*/)
{
  std::cout << "FakeBrain::modify()" << std::endl;
}

void FakeBrain::update(
        const std::vector< revolve::gazebo::MotorPtr > &/*motors*/,
        const std::vector< revolve::gazebo::SensorPtr > &/*sensors*/,
        double /*t*/,
        double /*step*/)
{
  std::cout << "FakeBrain::update()" << std::endl;
}
