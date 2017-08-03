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

#ifndef TESTFAKELIGHTSENSOR_H
#define TESTFAKELIGHTSENSOR_H

#include "../FakeLightSensor.h"

class TestFakeLightSensor : public tol::FakeLightSensor
{
public:
    explicit TestFakeLightSensor(float fov,
                                 const ignition::math::Pose3d robot_sensor_offset,
                                 const ignition::math::Vector3d light_position)
        : tol::FakeLightSensor(
            "test_fake_light_sensor",
            fov,
            light_position
        )
    {
        this->updateRobotPosition(robot_sensor_offset);
    }

// Expose protected methods
    float expose_light_distance() {
        return light_distance();
    }

    float expose_light_angle() {
        return light_angle();
    }

};

#endif  //  TESTFAKELIGHTSENSOR_H
