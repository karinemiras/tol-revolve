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

#include "brain/learner/cppneat/GeneticEncoding.h"

namespace tol
{
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
    BodyParser(std::string &yaml_path);

    BodyParser(const std::string &yaml_path,
               const bool _isMlmp);

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
    // Body parsing
    std::map<std::string, int> part_arity;
    std::vector<BodyPart *> to_parse;
    BodyPart *core;
    std::vector<BodyPart *> delete_later;
    std::map<BodyPart *, YAML::Node> body_to_node;

    static void SetCoordinates(int x,
                               int y,
                               BodyPart *part);

    static int CalculateRotation(int arity,
                                 int slot,
                                 int parent_rotation);

    void make_empty(BodyPart *part);

    private:
    void ParseYaml(BodyPart *part,
                   YAML::Node &node);

    // Network

    /// \brief innovation number
    int innov_number;

    /// \brief Is controller architecture MLMP; used as a small hack
    bool isMlmp_ = false;

    std::vector<cppneat::ConnectionGenePtr> connections;

    std::vector<cppneat::NeuronGenePtr> neurons;

    std::vector<cppneat::NeuronGenePtr> input_neurons;

    std::vector<cppneat::NeuronGenePtr> output_neurons;

    std::map<cppneat::NeuronGenePtr, std::tuple<int, int, int>>
            neuron_coordinates;
  };
}

#endif  //  TOL_PLUGIN_BODY_H_
