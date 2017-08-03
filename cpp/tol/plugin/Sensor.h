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

#ifndef TOL_PLUGIN_SENSOR_H_
#define TOL_PLUGIN_SENSOR_H_

#include "brain/Sensor.h"
#include "revolve/gazebo/sensors/Sensor.h"

namespace tol {

class Sensor
        : public revolve::brain::Sensor
{

public:
    explicit Sensor(revolve::gazebo::SensorPtr sensorPtr)
            :
            sensorPtr(sensorPtr)
    {}

    virtual unsigned int
    inputs() const
    {
      return sensorPtr->inputs();
    }

    virtual void
    read(double *input_vector)
    {
      sensorPtr->read(input_vector);
    }

    virtual std::string
    sensorId() const
    {
      return sensorPtr->sensorId();
    }

private:
    revolve::gazebo::SensorPtr sensorPtr;
};

}

#endif // TOL_PLUGIN_SENSOR_H_
