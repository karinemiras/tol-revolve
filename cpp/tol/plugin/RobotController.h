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

/*
 * RobotController.h
 *
 *  Created on: May 9, 2015
 *      Author: elte
 */

#ifndef TOL_PLUGIN_ROBOTCONTROLLER_H_
#define TOL_PLUGIN_ROBOTCONTROLLER_H_

#include <revolve/gazebo/plugin/RobotController.h>

#include "brain/Evaluator.h"
#include "brain/RLPower.h"

namespace tol {

class RobotController
        : public revolve::gazebo::RobotController
{
public:
    RobotController();

    virtual ~RobotController();

    virtual void
    Load(::gazebo::physics::ModelPtr _parent,
         sdf::ElementPtr _sdf);

    virtual void
    LoadBrain(sdf::ElementPtr sdf);

    virtual void
    DoUpdate(const gazebo::common::UpdateInfo info);

private:
    class Evaluator
            : public tol::Evaluator
    {
    public:
        Evaluator();

        virtual void
        start();

        virtual double
        fitness();

        void
        updatePosition(const ignition::math::Pose3d pose);

        ignition::math::Pose3d currentPosition_;
        ignition::math::Pose3d previousPosition_;
    };

    boost::shared_ptr<Evaluator> evaluator_;
};

} /* namespace tol */

GZ_REGISTER_MODEL_PLUGIN(tol::RobotController)

#endif // TTOL_PLUGIN_ROBOTCONTROLLER_H_
