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

#ifndef TOL_PLUGIN_ACTUATOR_H_
#define TOL_PLUGIN_ACTUATOR_H_

#include "revolve/gazebo/motors/Motor.h"

#include "brain/Actuator.h"

namespace tol
{
  class Actuator
          : public revolve::brain::Actuator
  {
    public:
    explicit Actuator(revolve::gazebo::MotorPtr actuatorPtr)
            : actuatorPtr(actuatorPtr)
    {}

    virtual size_t outputs() const
    {
      return actuatorPtr->outputs();
    }

    virtual void update(double *output_vector,
                        double step)
    {
      size_t size = this->outputs();
      double *output_copy = new double[size];

      for (size_t i = 0; i < size; i++)
      {
        output_copy[i] = (output_vector[i] + 1) / 2;
      }

      actuatorPtr->update(output_copy, step);

      delete[] output_copy;
    }

    private:
    revolve::gazebo::MotorPtr actuatorPtr;
  };
}

#endif  //  TOL_PLUGIN_ACTUATOR_H_
