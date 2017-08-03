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

#ifndef TOL_SUPGBRAINPHOTOTAXIS_H
#define TOL_SUPGBRAINPHOTOTAXIS_H

#include "revolve/gazebo/brain/Brain.h"

#include "brain/SUPGBrainPhototaxis.h"

#include "../FakeLightSensor.h"

namespace tol {

class SUPGBrainPhototaxis : public revolve::gazebo::Brain
                          , private revolve::brain::SUPGBrainPhototaxis
{
public:
    SUPGBrainPhototaxis(const std::string &robot_name,
                        revolve::brain::EvaluatorPtr evaluator,
                        double light_radius_distance,
                        const std::vector< std::vector< float > >& neuron_coordinates,
                        const std::vector< revolve::gazebo::MotorPtr >& actuators,
                        std::vector< revolve::gazebo::SensorPtr >& sensors);
    virtual ~SUPGBrainPhototaxis();

    void update(const std::vector< revolve::gazebo::MotorPtr > &motors,
                const std::vector< revolve::gazebo::SensorPtr > &sensors,
                double t, double step) override;

    void updateRobotPosition(ignition::math::Pose3d &robot_position);

private: // methods
    static const std::vector<revolve::brain::SensorPtr>
    createEnhancedSensorWrapper(const std::vector<revolve::gazebo::SensorPtr> &original);


private:
    boost::shared_ptr<FakeLightSensor> light_sensor_left,
                                       light_sensor_right;
    ignition::math::Pose3<double> robot_position;
};

}

#endif  //  TOL_SUPGBRAINPHOTOTAXIS_H
