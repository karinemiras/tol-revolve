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

#include <map>
#include <string>
#include <vector>

#include "brain/learner/cppneat/CPPNNeuron.h"

#include "BodyParser.h"

namespace rg = revolve::gazebo;

using namespace tol;

BodyParser::BodyParser(const std::string &_yamlPath)
        : innov_number(0)
{
  YAML::Node yaml_genome = YAML::LoadFile(_yamlPath);
  if (yaml_genome.IsNull())
  {
    std::cerr << "Failed to load a YAML 'genome' file." << std::endl;
    return;
  }
  this->initParser(yaml_genome);
}

void BodyParser::initParser(const YAML::Node &_yaml)
{
  this->arity_["Core"] = 4;
  this->arity_["CoreWithMics"] = 4;
  this->arity_["FixedBrick"] = 4;
  this->arity_["ActiveHinge"] = 2;
  this->arity_["Hinge"] = 2;
  this->arity_["ParametricBarJoint"] = 2;

  // read body structure from yaml file
  // includes adding the rotation of the parts and their coordinates
  this->bodyMap_ = new BodyPart();
  this->toParse_.push_back(this->bodyMap_);
  this->bodyToNode_[bodyMap_] = _yaml["body"];
  this->initPart(bodyMap_);

  BodyPart *cur_part;
  while (not this->toParse_.empty())
  {
    cur_part = this->toParse_.back();
    this->toParse_.pop_back();
    this->ParseYaml(cur_part, this->bodyToNode_[cur_part]);
  }

  // couple neighbouring differential oscillators
  // here all oscillators must already be initialised
  this->toParse_.push_back(this->bodyMap_);
  while (not this->toParse_.empty())
  {
    cur_part = this->toParse_.back();
    this->toParse_.pop_back();
    if (cur_part->type != "ActiveHinge")
    {
      bool neighbour_is_motor[4];
      for (int i = 0; i < 4; i++)
      {
        neighbour_is_motor[i] = cur_part->neighbours[i].type == "ActiveHinge";
      }
      // add connections between each of the first neurons of an oscillator
      for (int i = 0; i < 4; i++)
      {
        for (int j = 0; j < 4; j++)
        {
          // no need to add both connections other one will be added for
          // exchanged i,j and // dont add selfreferential connection
          if (neighbour_is_motor[i] and neighbour_is_motor[j] and i != j)
          {
            cppneat::ConnectionGenePtr first_to_first(
                    new cppneat::ConnectionGene(
                            cur_part->neighbours[i].differential_oscillator[0]
                                    ->getInnovNumber(),
                            cur_part->neighbours[j].differential_oscillator[0]
                                    ->getInnovNumber(),
                            0,
                            ++innov_number,
                            true));
            this->connections_.push_back(first_to_first);
          }
        }
      }
    }

    // add neighbours to queue
    // watch out for parent
    if (cur_part == this->bodyMap_)
    {
      if (cur_part->neighbours[0].name != "empty")
      {
        this->toParse_.push_back(&(cur_part->neighbours[0]));
      }
    }
    for (int i = 1; i < 4; i++)
    {
      if (cur_part->neighbours[i].name != "empty")
      {
        this->toParse_.push_back(&(cur_part->neighbours[i]));
      }
    }
  }

  // add input neurons and add connections between them and neurons on core
  for (int i = 0; i < 6; i++)
  {
    // add neuron
    std::map< std::string, double > empty;

    cppneat::NeuronPtr neuron(
            new cppneat::Neuron(bodyMap_->name + "-in-" + std::to_string(i),
                                cppneat::Neuron::INPUT_LAYER,
                                cppneat::Neuron::INPUT,
                                empty));
    cppneat::NeuronGenePtr neuron_gene(
            new cppneat::NeuronGene(neuron, ++innov_number, true));
    this->neurons_.push_back(neuron_gene);
    this->inputNeurons_.push_back(neuron_gene);
    std::tuple< int, int, int > coord(0, 0, 0);
    this->coordinates_[neuron_gene] = coord;

    // connect to neighbouring oscillators
    for (int i = 0; i < 4; i++)
    {
      if (this->bodyMap_->neighbours[i].type == "ActiveHinge")
      {
        cppneat::ConnectionGenePtr input_to_first(
                new cppneat::ConnectionGene(
                        bodyMap_->neighbours[i].differential_oscillator[0]
                                ->getInnovNumber(),
                        neuron_gene->getInnovNumber(),
                        0,
                        ++innov_number,
                        true));
        this->connections_.push_back(input_to_first);
      }
    }
  }
}

BodyParser::~BodyParser()
{
  for (BodyPart *todel : this->toDelete_)
  {
    delete[] todel;
  }
  delete this->bodyMap_;
}

cppneat::GeneticEncodingPtr
BodyParser::CoupledCpgNetwork()
{
  cppneat::GeneticEncodingPtr ret(
          new cppneat::GeneticEncoding(this->neurons_, this->connections_));
  return ret;
}

std::pair< std::map< int, size_t >, std::map< int, size_t>>
BodyParser::InputOutputMap(const std::vector< rg::MotorPtr > &actuators,
                           const std::vector< rg::SensorPtr > &sensors)
{
  size_t p = 0;
  std::map< int, size_t > output_map;
  // Admapd output neurons to motors:

  // map of numbers of output neurons for each body part
  std::map< std::string, size_t > outputCountMap;

  for (auto it = actuators.begin(); it != actuators.end(); ++it)
  {
    auto motor = *it;
    auto partId = motor->partId();

    if (not outputCountMap.count(partId))
    {
      outputCountMap[partId] = 0;
    }

    for (size_t i = 0, l = motor->outputs(); i < l; ++i)
    {
      std::stringstream neuronId;
      neuronId << partId << "-out-" << outputCountMap[partId];
      outputCountMap[partId]++;

      size_t j;
      for (j = 0; j < this->outputNeurons_.size(); j++)
      {
        if (this->outputNeurons_[j]->neuron->neuron_id == neuronId.str())
        {
          break;
        }
      }
//      if (j == output_neurons.size())
//      {
//        std::cerr << "Required output neuron " << neuronId.str()
//                  << " for motor could not be located" << std::endl;
//        throw std::runtime_error("Robot brain error");
//      }
      output_map[this->outputNeurons_[j]->getInnovNumber()] = p++;
    }
  }

  p = 0;
  std::map< int, size_t > input_map;
  // Map input neurons to sensors:

  // map of number of input neurons for each part:

  std::map< std::string, size_t > inputCountMap;

  for (auto it = sensors.begin(); it != sensors.end(); ++it)
  {
    auto sensor = *it;
    auto partId = sensor->partId();

    if (not inputCountMap.count(partId))
    {
      inputCountMap[partId] = 0;
    }

    for (size_t i = 0, l = sensor->inputs(); i < l; ++i)
    {
      std::stringstream neuronId;
      neuronId << partId << "-in-" << inputCountMap[partId];
      inputCountMap[partId]++;

      size_t j;
      for (j = 0; j < this->inputNeurons_.size(); j++)
      {
        if (this->inputNeurons_[j]->neuron->neuron_id == neuronId.str())
        {
          break;
        }
      }
      if (j == this->inputNeurons_.size())
      {
        std::cerr << "Required input neuron " << neuronId.str()
                  << " for sensor could not be located" << std::endl;
        throw std::runtime_error("Robot brain error");
      }
      input_map[this->inputNeurons_[j]->getInnovNumber()] = p++;
    }
  }
  return std::make_pair(input_map, output_map);
}

cppneat::GeneticEncodingPtr
BodyParser::CppnNetwork()
{
  int innov_number = 1;
  cppneat::GeneticEncodingPtr ret(new cppneat::GeneticEncoding(true));
  // add inputs
  std::map< std::string, double > empty;
  for (int i = 0; i < 6; i++)
  {
    cppneat::NeuronPtr neuron(
            // better names (like input x1 etc) might help
            new cppneat::Neuron("Input-" + std::to_string(i),
                                cppneat::Neuron::INPUT_LAYER,
                                cppneat::Neuron::INPUT,
                                empty));
    // means innovation number is i + 1
    cppneat::NeuronGenePtr neuron_gene(
            new cppneat::NeuronGene(neuron, innov_number++, true));
    ret->add_neuron_gene(neuron_gene, 0, i == 0);
  }

  // add outputs
  empty["rv:bias"] = 0;
  empty["rv:gain"] = 0;
  cppneat::NeuronPtr weight_neuron(
          new cppneat::Neuron("weight",
                              cppneat::Neuron::OUTPUT_LAYER,
                              cppneat::Neuron::SIMPLE,
                              empty));
  cppneat::NeuronGenePtr weight_neuron_gene(
          new cppneat::NeuronGene(weight_neuron, innov_number++, true));
  ret->add_neuron_gene(weight_neuron_gene, 1, true);
  cppneat::NeuronPtr bias_neuron(
          new cppneat::Neuron("rv:bias",
                              cppneat::Neuron::OUTPUT_LAYER,
                              cppneat::Neuron::SIMPLE,
                              empty));
  cppneat::NeuronGenePtr bias_neuron_gene(
          new cppneat::NeuronGene(bias_neuron, innov_number++, true));
  ret->add_neuron_gene(bias_neuron_gene, 1, false);
  cppneat::NeuronPtr gain_neuron(
          new cppneat::Neuron("rv:gain",
                              cppneat::Neuron::OUTPUT_LAYER,
                              cppneat::Neuron::SIMPLE,
                              empty));
  cppneat::NeuronGenePtr gain_neuron_gene(
          new cppneat::NeuronGene(gain_neuron,
                                  innov_number++,
                                  true));
  ret->add_neuron_gene(gain_neuron_gene, 1, false);

  // connect every input with every output
  for (int i = 0; i < 6; i++)
  {
    cppneat::ConnectionGenePtr connection_to_weight(
            new cppneat::ConnectionGene(weight_neuron_gene->getInnovNumber(),
                                        i + 1, 0,
                                        innov_number++,
                                        true, ""));
    ret->add_connection_gene(connection_to_weight);

    cppneat::ConnectionGenePtr connection_to_bias(
            new cppneat::ConnectionGene(bias_neuron_gene->getInnovNumber(),
                                        i + 1, 0,
                                        innov_number++,
                                        true, ""));
    ret->add_connection_gene(connection_to_bias);

    cppneat::ConnectionGenePtr connection_to_gain(
            new cppneat::ConnectionGene(gain_neuron_gene->getInnovNumber(),
                                        i + 1, 0,
                                        innov_number++,
                                        true, ""));
    ret->add_connection_gene(connection_to_gain);
  }
  return ret;
}

std::map< std::string, std::tuple< int, int, int > >
BodyParser::IdToCoordinatesMap()
{
  std::map< std::string, std::tuple< int, int, int > > ret;
  for (auto pair : this->coordinates_)
  {
    ret[pair.first->neuron->neuron_id] = pair.second;
  }
  return ret;
}

std::vector< std::pair< int, int > > BodyParser::get_coordinates_sorted(
        const std::vector< rg::MotorPtr > &actuators)
{
  std::vector< std::pair< int, int>> coordinates;

  // map of numbers of output neurons for each body part
  std::map< std::string, size_t > outputCountMap;

  for (auto it = actuators.begin(); it != actuators.end(); ++it)
  {
    auto motor = *it;
    auto partId = motor->partId();

    if (not outputCountMap.count(partId))
    {
      outputCountMap[partId] = 0;
    }

    for (size_t i = 0, l = motor->outputs(); i < l; ++i)
    {
      std::stringstream neuronId;
      neuronId << partId << "-out-" << outputCountMap[partId];
      outputCountMap[partId]++;

      size_t j;
      for (j = 0; j < this->outputNeurons_.size(); j++)
      {
        if (this->outputNeurons_[j]->neuron->neuron_id == neuronId.str())
        {
          break;
        }
      }
      if (j == this->outputNeurons_.size())
      {
        std::cerr << "Required output neuron " << neuronId.str()
                  << " for motor could not be located" << std::endl;
        throw std::runtime_error("Robot brain error");
      }
      coordinates.push_back(std::pair< int, int >(
              std::get< 0 >(this->coordinates_[this->outputNeurons_[j]]),
              std::get< 1 >(this->coordinates_[this->outputNeurons_[j]])));
    }
  }
  return coordinates;
}

std::tuple< int, int > BodyParser::setCoordinates(const size_t _rotation,
                                                  const int _init_x,
                                                  const int _init_y)
{
  int x = _init_x;
  int y = _init_y;

  switch (_rotation)
  {
    case 0:
      x += 1;
      break;
    case 1:
      y += 1;
      break;
    case 2:
      x -= 1;
      break;
    case 3:
      y -= 1;
      break;
    default:
      std::cerr << "Wrong rotation calculated." << std::endl;
      std::exit(-1);
  }

  return std::make_tuple(x, y);
}

size_t BodyParser::calculateRotation(const size_t _arity,
                                     const size_t _slot,
                                     const size_t _parents_rotation)
{
  if (_arity == 2 or _arity == 4)
  {
    switch (_slot)
    {
      case 0:
        return (_parents_rotation + 2) % 4;
      case 1:
        return _parents_rotation;
      case 2:
        return (_parents_rotation + 3) % 4;
      case 3:
        return (_parents_rotation + 1) % 4;
      default:
        std::cerr << "Unsupported parents slot provided." << std::endl;
        std::exit(-1);
    }
  }
  else
  {
    std::cerr << "Unsupported module arity provided." << std::endl;
    std::exit(-1);
  }
}

void BodyParser::initPart(BodyPart *_part)
{
  _part->name = "empty";
  _part->rotation = 0;
  _part->coordinates[0] = 0;
  _part->coordinates[1] = 0;

  _part->neighbours = new BodyPart[4];
  this->toDelete_.push_back(_part->neighbours);
  for (int i = 0; i < 4; i++)
  {
    _part->neighbours[i].name = "empty";
  }
}

// only works for slot: 0 currently
void BodyParser::ParseYaml(BodyPart *_module,
                           YAML::Node &_yaml)
{
  std::string current_name = _yaml["id"].as< std::string >();
  _module->name = _yaml["id"].as< std::string >();
  _module->type = _yaml["type"].as< std::string >();
  _module->arity = arity_[_module->type];

  // add differential oscillator in case of the current part being an
  // ActiveHinge output is also added here needs to be done before adding the
  // parent as a neighbour to the children since the neighbours are remembered
  // by value and not reference
  if (_module->type == "ActiveHinge")
  {
    // add neurons
    std::map< std::string, double > params;
    {
      params["rv:bias"] = 0;
      params["rv:gain"] = 1;

      cppneat::NeuronPtr first_neuron(
              new cppneat::Neuron(_module->name + "-hidden-0",
                                  cppneat::Neuron::HIDDEN_LAYER,
                                  cppneat::Neuron::DIFFERENTIAL_CPG,
                                  params));
      _module->differential_oscillator[0] = cppneat::NeuronGenePtr(
              new cppneat::NeuronGene(first_neuron, ++innov_number, true));

      this->neurons_.push_back(_module->differential_oscillator[0]);
      std::tuple< int, int, int > first_coord(_module->coordinates[0],
                                              _module->coordinates[1],
                                              1);
      this->coordinates_[_module->differential_oscillator[0]] = first_coord;

      cppneat::NeuronPtr second_neuron(
              new cppneat::Neuron(_module->name + "-hidden-1",
                                  cppneat::Neuron::HIDDEN_LAYER,
                                  cppneat::Neuron::DIFFERENTIAL_CPG,
                                  params));
      _module->differential_oscillator[1] = cppneat::NeuronGenePtr(
              new cppneat::NeuronGene(second_neuron, ++innov_number, true));

      this->neurons_.push_back(_module->differential_oscillator[1]);
      std::tuple< int, int, int > second_coord(_module->coordinates[0],
                                               _module->coordinates[1],
                                               -1);
      this->coordinates_[_module->differential_oscillator[1]] = second_coord;
    }

    cppneat::NeuronPtr output_neuron(
            new cppneat::Neuron(_module->name + "-out-0",
                                cppneat::Neuron::OUTPUT_LAYER,
                                cppneat::Neuron::SIMPLE,
                                params));
    _module->differential_oscillator[2] = cppneat::NeuronGenePtr(
            new cppneat::NeuronGene(output_neuron, ++innov_number, true));

    this->neurons_.push_back(_module->differential_oscillator[2]);
    this->outputNeurons_.push_back(_module->differential_oscillator[2]);
    std::tuple< int, int, int > output_coord(_module->coordinates[0],
                                             _module->coordinates[1],
                                             0);
    this->coordinates_[_module->differential_oscillator[2]] = output_coord;

    // add connections between only these neurons (for now)
    cppneat::ConnectionGenePtr first_to_second(new cppneat::ConnectionGene(
            _module->differential_oscillator[1]->getInnovNumber(),
            _module->differential_oscillator[0]->getInnovNumber(),
            0,
            ++innov_number,
            true));
    this->connections_.push_back(first_to_second);

    cppneat::ConnectionGenePtr second_to_first(new cppneat::ConnectionGene(
            _module->differential_oscillator[0]->getInnovNumber(),
            _module->differential_oscillator[1]->getInnovNumber(),
            0,
            ++innov_number,
            true));
    this->connections_.push_back(second_to_first);

    cppneat::ConnectionGenePtr first_to_output(new cppneat::ConnectionGene(
            _module->differential_oscillator[2]->getInnovNumber(),
            _module->differential_oscillator[0]->getInnovNumber(),
            0,
            ++innov_number,
            true));
    this->connections_.push_back(first_to_output);
  }

  // parse body and add initialised children to parsing queue
  YAML::Node children = _yaml["children"];
  if (children.IsNull())
  {
      return;
  }

  if (_module->arity == 4)
  {
//    if (children[0].size() != 0 && part->type != "Core")
//    { // child in parent socket
//      std::cout
//              << "ERROR: child in parent socket (currently only 0 is "
//                      "accepted as parent socket)"
//              << std::endl;
//    }
    for (int i = 0; i < 4; i++)
    {
      if (children[i].size() == 0)
      {
        continue;
      }
      BodyPart *child = &_module->neighbours[i];
      this->initPart(child);
      child->rotation = this->calculateRotation(4, i, _module->rotation);

      int offsprings_x, offsprings_y;
      std::tie(offsprings_x, offsprings_y) =
              this->setCoordinates(child->rotation,
                                   _module->coordinates[0],
                                   _module->coordinates[1]);

      child->coordinates[0] = offsprings_x;
      child->coordinates[1] = offsprings_y;

      // add parent as neighbour of child
      child->neighbours[0] = *_module;

      this->bodyToNode_[child] = children[i];
      this->toParse_.push_back(child);
    }
  }
  else
  {
//    if (children[0].size() != 0)
//    { // child in parent socket
//      std::cout << "ERROR: child in parent socket "
//              "(currently only 0 is accepted as parent socket)" << std::endl;
//    }
    for (int i = 1; i < 2; i++)
    {
      if (children[i].size() == 0)
      {
        continue;
      }
      BodyPart *child = &_module->neighbours[i];
      initPart(child);
      child->rotation = calculateRotation(2, i, _module->rotation);

      int offsprings_x, offsprings_y;
      std::tie(offsprings_x, offsprings_y) =
              this->setCoordinates(child->rotation,
                                   _module->coordinates[0],
                                   _module->coordinates[1]);

      child->coordinates[0] = offsprings_x;
      child->coordinates[1] = offsprings_y;

      // add parent as neighbour of child
      child->neighbours[0] = *_module;

      this->bodyToNode_[child] = children[i];
      this->toParse_.push_back(child);
    }
  }
}


