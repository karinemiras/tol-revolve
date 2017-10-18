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

#ifndef PARSEYAMLGENOME_YAMLBODYPARSER_H
#define PARSEYAMLGENOME_YAMLBODYPARSER_H

#include <string>
#include <memory>
#include <tuple>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace tol
{
  const std::string CORE = "Core";

  const std::string A_HINGE = "ActiveHinge";

  const std::string P_HINGE = "PassiveHinge";

  const std::string BRICK = "FixedBrick";

  const size_t MAX_SLOTS = 4;

  class BodyPart
  {
    public:
    BodyPart();

    BodyPart(
            const std::string &name,
            const std::string &type,
            int x,
            int y,
            size_t rotation);

    ~BodyPart();

    std::string name = "none";
    std::string type = "none";
    int x = 0;
    int y = 0;
    size_t id = 0;
    size_t arity = 4;
    size_t rotation = 0;
    BodyPart *neighbours[MAX_SLOTS];
  };

  typedef std::vector< std::vector< bool>> ConnectionMatrix;

  typedef std::vector< std::vector< float>> CoordinatesMatrix;

  class YamlBodyParser
  {
    public:
    /// \brief
    YamlBodyParser();

    /// \brief
    ~YamlBodyParser();

    /// \brief
    ConnectionMatrix connections();

    /// \brief
    CoordinatesMatrix coordinates();

    /// \brief
    void parseFile(const std::string &filename);

    /// \brief
    void parseCode(const std::string &code);

    private:
    /// \brief
    void init(const YAML::Node &root_genome_node);

    /// \brief
    BodyPart *parseModule(
            BodyPart *parent,
            const YAML::Node &offspring,
            const size_t rotation,
            int x,
            int y);

    /// \brief
    size_t calculateRotation(
            const size_t arity,
            const size_t slot,
            const size_t parents_rotation) const;

    /// \brief
    std::tuple< int, int > setCoordinates(
            const size_t rotation,
            const int init_x,
            const int init_y);

    /// \brief
    void setNormalisedCoordinates(
            BodyPart *module,
            const int range_x,
            const int range_y);

    /// \brief
    void setConnections(BodyPart *module);

    /// \brief
    size_t n_actuators_ = 0;

    /// \brief
    int max_x = 0;

    /// \brief
    int max_y = 0;

    /// \brief
    int min_x = 0;

    /// \brief
    int min_y = 0;

    /// \brief
    BodyPart *bodyMap_ = nullptr;

    /// \brief
    ConnectionMatrix connections_;

    /// \brief
    CoordinatesMatrix coordinates_;
  };
}

#endif  // PARSEYAMLGENOME_YAMLBODYPARSER_H
