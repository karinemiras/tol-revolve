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
* Author: Elte Hupkes
* Date: September 20, 2015
*
*/

#ifndef TOL_PLUGIN_INTERACTIVEEVOLUTION_H_
#define TOL_PLUGIN_INTERACTIVEEVOLUTION_H_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829

# include <gazebo/common/KeyEvent.hh>
# include <gazebo/common/Plugin.hh>
# include <gazebo/gui/gui.hh>
# include <gazebo/gui/GuiPlugin.hh>
# include <gazebo/rendering/UserCamera.hh>
# include <gazebo/transport/transport.hh>

#endif

namespace tol
{
  class InteractiveEvolutionPlugin
          : public ::gazebo::GUIPlugin
  {
    Q_OBJECT

    public:
    InteractiveEvolutionPlugin();

    ~InteractiveEvolutionPlugin();

    protected slots:

    void OnReproduceButton();

    private:
    bool OnKeyDown(const ::gazebo::common::KeyEvent _event);

    // Transport nodes for the contact messages
    ::gazebo::transport::NodePtr node_;

    // Key publisher
    ::gazebo::transport::PublisherPtr keyPub_;
  };
}

#endif  //  TOL_PLUGIN_INTERACTIVEEVOLUTION_H_
