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

#ifndef TOL_PLUGIN_HELPER_H_
#define TOL_PLUGIN_HELPER_H_

#include <vector>

#include "../Actuator.h"
#include "../Sensor.h"

namespace tol {

class Helper
{
public:
    static const std::vector<revolve::brain::ActuatorPtr>
    createWrapper(const std::vector<revolve::gazebo::MotorPtr> &original);

    static const std::vector<revolve::brain::SensorPtr>
    createWrapper(const std::vector<revolve::gazebo::SensorPtr> &original);

    enum RobotType {
        spider9,
        spider13,
        spider17,
        gecko7,
        gecko12,
        gecko17,
        snake5,
        snake7,
        snake9,
        babyA,
        babyB,
        babyC
    };

    static RobotType parseRobotType(const std::string &str);

private:
    explicit Helper()
    {}
};
}

std::ostream& operator<<(std::ostream& os, tol::Helper::RobotType type);

#endif // TOL_PLUGIN_HELPER_H_
