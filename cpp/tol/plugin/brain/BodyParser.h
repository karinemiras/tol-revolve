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

#ifndef TOL_PLUGIN_BODY_H_
#define TOL_PLUGIN_BODY_H_

#include <vector>
#include <map>
#include <string>
#include <tuple>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "revolve/gazebo/motors/Motor.h"
#include "revolve/gazebo/sensors/Sensor.h"

#include "brain/learner/cppneat/CPPNNeuron.h"
#include "brain/learner/cppneat/GeneticEncoding.h"

namespace tol
{
  const std::string CORE = "Core";

  const std::string A_HINGE = "ActiveHinge";

  const std::string P_HINGE = "PassiveHinge";

  const std::string BRICK = "FixedBrick";

  const size_t MAX_SLOTS = 4;

  class BodyParser
  {
    struct BodyPart
    {
      std::string name;
      std::string type;
      int arity;
      int rotation;          // 0 -> 0, 1 -> 90 degrees, etc.
      int coordinates[2];    // the coordinates of this `bodypart`
      BodyPart *neighbours;  // the neighbours of this `bodypart`
      cppneat::NeuronGenePtr differential_oscillator[3];
    };
    public:
    BodyParser(const std::string &_yamlPath);

    ~BodyParser();

    cppneat::GeneticEncodingPtr CoupledCpgNetwork();

    std::pair<std::map<int, size_t>, std::map<int, size_t>>
    InputOutputMap(const std::vector<revolve::gazebo::MotorPtr> &actuators,
                   const std::vector<revolve::gazebo::SensorPtr> &sensors);

    cppneat::GeneticEncodingPtr CppnNetwork();

    std::map<std::string, std::tuple<int, int, int>> IdToCoordinatesMap();

    /// \brief returns the coordinates of the actuators matching the order the
    /// actuators give coordinate of actuators[0] is in sorted_coordinates[0]
    std::vector<std::pair<int, int>> get_coordinates_sorted(
            const std::vector<revolve::gazebo::MotorPtr> &actuators);

    int InnovationNumber()
    {
      return innov_number + 1;
    };

    private:
    static std::tuple< int, int > setCoordinates(const size_t _rotation,
                                                 const int _init_x,
                                                 const int _init_y);

    static size_t calculateRotation(const size_t _arity,
                                    const size_t _slot,
                                    const size_t _parents_rotation);

    void initPart(BodyPart *_part);

    private:
    void ParseYaml(BodyPart *_module,
                   YAML::Node &_yaml);

    void initParser(const YAML::Node &_yaml);

    // Network

    /// \brief innovation number
    int innov_number;

    std::vector<cppneat::NeuronGenePtr> neurons_;

    std::vector<cppneat::NeuronGenePtr> inputNeurons_;

    std::vector<cppneat::NeuronGenePtr> outputNeurons_;

    std::vector<cppneat::ConnectionGenePtr> connections_;

    std::map<cppneat::NeuronGenePtr, std::tuple<int, int, int>> coordinates_;

    // Body parsing
    std::map<std::string, int> arity_;
    std::vector<BodyPart *> toParse_;
    std::vector<BodyPart *> toDelete_;
    std::map<BodyPart *, YAML::Node> bodyToNode_;

    /// \brief A tree data structure representing a robot
    BodyPart *bodyMap_ = nullptr;
  };
}

#endif  //  TOL_PLUGIN_BODY_H_
