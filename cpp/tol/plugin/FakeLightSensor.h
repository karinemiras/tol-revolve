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

#ifndef FAKELIGHTSENSOR_H
#define FAKELIGHTSENSOR_H

#include <string>

#include <ignition/math/Pose3.hh>

#include <revolve/gazebo/sensors/VirtualSensor.h>

#include "brain/FakeLightSensor.h"

namespace tol {

class FakeLightSensor : public revolve::brain::FakeLightSensor
                      , public revolve::gazebo::VirtualSensor
{
public:
    FakeLightSensor(std::string name,
                    float fov,
                    ignition::math::Vector3d light_pos);
    virtual ~FakeLightSensor();

    virtual double light_distance() override;
    virtual double light_angle() override;

    virtual std::string sensorId() const override;

    virtual void updateRobotPosition(const ignition::math::Pose3d &robot_position);

    /**
     * Reads the current value of this sensor into the given network
     * output array. This should fill the number of input neurons
     * the sensor specifies to have, i.e. if the sensor specifies 2
     * input neurons it should fill `input[0]` and `input[1]`
     */
    virtual void read(double * input) override {
//         std::cerr << "######## tol::FakeLightSensor::read()" << std::endl;
        revolve::brain::FakeLightSensor::read(input);
    }

private:
    std::string sensor_name;
    ignition::math::Vector3d light_pos;
    ignition::math::Pose3d robot_position;
};

}

#endif // FAKELIGHTSENSOR_H
