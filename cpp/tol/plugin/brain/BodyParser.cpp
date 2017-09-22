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
* Author: Rafael Kiesel, modified Milan Jelisavcic
*
*/

#include <map>
#include <string>
#include <vector>

#include "brain/learner/cppneat/CPPNNeuron.h"

#include "BodyParser.h"

namespace rg = revolve::gazebo;

using namespace tol;

///////////////////////////////////////////////////
BPart::BPart()
{
  std::memset(neighbours, 0, sizeof(neighbours));
}

///////////////////////////////////////////////////
BPart::BPart(
        const std::string &name,
        const std::string &type,
        int x,
        int y,
        size_t rotation)
        : name(name)
        , type(type)
        , x(x)
        , y(y)
        , rotation(rotation)
{
  std::memset(neighbours, 0, sizeof(neighbours));
}

///////////////////////////////////////////////////
BPart::~BPart()
{
  // Delete dynamically allocated parents slots
  for (size_t parents_slot = 0; parents_slot < MAX_SLOTS; ++parents_slot)
  {
    delete neighbours[parents_slot];
  }
}

///////////////////////////////////////////////////
BodyParser::BodyParser(const std::string &_yamlPath)
        : innovation_number_(0)
{
  YAML::Node yaml_genome = YAML::LoadFile(_yamlPath);
  if (yaml_genome.IsNull())
  {
    std::cerr << "Failed to load a YAML 'genome' file." << std::endl;
    return;
  }
  this->initParser(yaml_genome);
//  this->init(yaml_genome);
}

void BodyParser::init(const YAML::Node &_yaml)
{
  this->bMap_ = parseModule(nullptr, _yaml["body"], 0, 0, 0);
}

///////////////////////////////////////////////////
BPart *BodyParser::parseModule(
        BPart *parent,
        const YAML::Node &offspring,
        const size_t rotation,
        int x,
        int y)
{
  BPart *module = nullptr;
  if (offspring.IsDefined())
  {
    module = new BPart();
    module->name = offspring["name"].IsDefined() ?
                   offspring["name"].as< std::string >() :
                   offspring["id"].as< std::string >();

    module->type = offspring["type"].as< std::string >();
    module->rotation = rotation;
    module->x = x;
    module->y = y;

//    if (A_HINGE == module->type)
//    {
//      this->n_actuators_ += 1;
//      module->id = this->n_actuators_;
//    }

    if (A_HINGE == module->type || P_HINGE == module->type)
    {
      module->arity = 2;
    }
    else if (CORE == module->type || BRICK == module->type)
    {
      module->arity = 4;
    }

    if (offspring["children"].IsDefined())
    {
      module->neighbours[0] = parent;
      size_t parents_slot = (CORE == module->type) ? 0 : 1;
      for (; parents_slot < MAX_SLOTS; ++parents_slot)
      {
        // Calculate coordinate for an offspring module
        int offsprings_x, offsprings_y;
        size_t offsprings_rotation =
                this->calculateRotation(module->arity,
                                        parents_slot,
                                        module->rotation);
        std::tie(offsprings_x, offsprings_y) =
                this->setCoordinates(offsprings_rotation, x, y);

        // Traverse recursively through each of offspring modules
        module->neighbours[parents_slot] =
                this->parseModule(module,
                                  offspring["children"][parents_slot],
                                  offsprings_rotation,
                                  offsprings_x,
                                  offsprings_y);
      }
    }
  }
  return module;
}

///////////////////////////////////////////////////
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

  BodyPart *current_part;
  while (not this->toParse_.empty())
  {
    current_part = this->toParse_.back();
    this->toParse_.pop_back();
    this->ParseYaml(current_part, this->bodyToNode_[current_part]);
  }

  // couple neighbouring differential oscillators
  // here all oscillators must already be initialised
  this->ConnectOscillators(current_part);
}

///////////////////////////////////////////////////
BodyParser::~BodyParser()
{
  for (BodyPart *todel : this->toDelete_)
  {
    delete[] todel;
  }
  delete this->bodyMap_;
}

///////////////////////////////////////////////////
cppneat::GeneticEncodingPtr BodyParser::CoupledCpgNetwork()
{
  cppneat::GeneticEncodingPtr ret(
          new cppneat::GeneticEncoding(this->neurons_, this->connections_));
  return ret;
}

///////////////////////////////////////////////////
std::pair< std::map< int, size_t >, std::map< int, size_t>>
BodyParser::InputOutputMap(
        const std::vector< rg::MotorPtr > &actuators,
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
      ++outputCountMap[partId];

      size_t j;
      for (j = 0; j < this->outputNeurons_.size(); ++j)
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
      for (j = 0; j < this->inputNeurons_.size(); ++j)
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

///////////////////////////////////////////////////
cppneat::GeneticEncodingPtr BodyParser::CppnNetwork()
{
  int innov_number = 1;
  cppneat::GeneticEncodingPtr ret(new cppneat::GeneticEncoding(true));
  // add inputs
  std::map< std::string, double > empty;
  for (size_t i = 0; i < 6; ++i)
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
  for (size_t i = 0; i < 6; ++i)
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

///////////////////////////////////////////////////
std::map< std::string, CoordsTriple > BodyParser::IdToCoordinatesMap()
{
  std::map< std::string, CoordsTriple > ret;
  for (auto pair : this->coordinates_)
  {
    ret[pair.first->neuron->neuron_id] = pair.second;
  }
  return ret;
}

///////////////////////////////////////////////////
std::vector< std::pair< int, int > > BodyParser::SortedCoordinates(
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
      for (j = 0; j < this->outputNeurons_.size(); ++j)
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

///////////////////////////////////////////////////
std::tuple< int, int > BodyParser::setCoordinates(
        const size_t _rotation,
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

///////////////////////////////////////////////////
size_t BodyParser::calculateRotation(
        const size_t _arity,
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

///////////////////////////////////////////////////
void BodyParser::initPart(BodyPart *_part)
{
  _part->rotation = 0;
  _part->x = 0;
  _part->y = 0;

  _part->neighbours = new BodyPart[MAX_SLOTS];
  this->toDelete_.push_back(_part->neighbours);
  for (size_t i = 0; i < MAX_SLOTS; ++i)
  {
    _part->neighbours[i].name = "empty";
  }
}

///////////////////////////////////////////////////
void BodyParser::GenerateDifferentialNeuron(BodyPart *_module,
                                            size_t _position)
{
  std::map< std::string, double > params;
  {
    params["rv:bias"] = 0;
    params["rv:gain"] = 1;
  }
  int z_coordinate;
  std::string name;
  cppneat::Neuron::Layer layer;
  cppneat::Neuron::Ntype type;

  switch (_position)
  {
    case 0:
      z_coordinate = 1;
      name = _module->name + "-hidden-0";
      layer = cppneat::Neuron::HIDDEN_LAYER;
      type = cppneat::Neuron::DIFFERENTIAL_CPG;
      break;
    case 1:
      z_coordinate = -1;
      name = _module->name + "-hidden-1";
      layer = cppneat::Neuron::HIDDEN_LAYER;
      type = cppneat::Neuron::DIFFERENTIAL_CPG;
      break;
    case 2:
      z_coordinate = 0;
      name = _module->name + "-out-0";
      layer = cppneat::Neuron::OUTPUT_LAYER;
      type = cppneat::Neuron::SIMPLE;
      break;
    default:
      std::cerr << "Unsupported neuron position provided." << std::endl;
      std::exit(1);
  }

  cppneat::NeuronPtr neuron(new cppneat::Neuron(name, layer, type, params));
  _module->differential_oscillator[_position] = cppneat::NeuronGenePtr(
          new cppneat::NeuronGene(neuron, ++innovation_number_, true));

  this->neurons_.push_back(_module->differential_oscillator[_position]);
  CoordsTriple coordinate(_module->x, _module->y, z_coordinate);
  this->coordinates_[_module->differential_oscillator[_position]] = coordinate;
}

///////////////////////////////////////////////////
void BodyParser::GenerateConnection(BodyPart *_module,
                                    size_t _from,
                                    size_t _to)
{
  cppneat::ConnectionGenePtr connection(new cppneat::ConnectionGene(
          _module->differential_oscillator[_to]->getInnovNumber(),
          _module->differential_oscillator[_from]->getInnovNumber(),
          0,
          ++innovation_number_,
          true));
  this->connections_.push_back(connection);
}

///////////////////////////////////////////////////
void BodyParser::GenerateOscillator(BodyPart *_module)
{
  if (_module->type == "ActiveHinge")
  {
    this->GenerateDifferentialNeuron(_module, 0);
    this->GenerateDifferentialNeuron(_module, 1);

    // TODO: Fix this. For unknown reason it does not work for a output neuron
    // this->GenerateDifferentialNeuron(_module, 2);
    std::map< std::string, double > params;
    {
      params["rv:bias"] = 0;
      params["rv:gain"] = 1;
    }
    cppneat::NeuronPtr output_neuron(
            new cppneat::Neuron(_module->name + "-out-0",
                                cppneat::Neuron::OUTPUT_LAYER,
                                cppneat::Neuron::SIMPLE,
                                params));
    _module->differential_oscillator[2] = cppneat::NeuronGenePtr(
            new cppneat::NeuronGene(output_neuron, ++innovation_number_, true));

    this->neurons_.push_back(_module->differential_oscillator[2]);
    this->outputNeurons_.push_back(_module->differential_oscillator[2]);
    CoordsTriple output_coord(_module->x, _module->y, 0);
    this->coordinates_[_module->differential_oscillator[2]] = output_coord;

    // add connections between only these neurons (for now)
    this->GenerateConnection(_module, 0, 1);
    this->GenerateConnection(_module, 1, 0);
    this->GenerateConnection(_module, 0, 2);
  }
}

///////////////////////////////////////////////////
void BodyParser::ConnectOscillators(BodyPart *_lastPart)
{
  this->toParse_.push_back(this->bodyMap_);
  while (not this->toParse_.empty())
  {
    _lastPart = this->toParse_.back();
    this->toParse_.pop_back();

    bool isNeighbourActuator[MAX_SLOTS];
    for (size_t i = 0; i < MAX_SLOTS; ++i)
    {
      isNeighbourActuator[i] =
              (_lastPart->neighbours[i].type == "ActiveHinge");
    }
    // add connections between each of the first neurons of an oscillator
    for (size_t i = 0; i < MAX_SLOTS; ++i)
    {
      for (size_t j = 0; j < MAX_SLOTS; ++j)
      {
        // no need to add both connections other one will be added for
        // exchanged i,j and // dont add selfreferential connection
        if (isNeighbourActuator[i] and isNeighbourActuator[j] and i != j)
        {
          cppneat::ConnectionGenePtr first_to_first(
                  new cppneat::ConnectionGene(
                          _lastPart->neighbours[i].differential_oscillator[0]
                                  ->getInnovNumber(),
                          _lastPart->neighbours[j].differential_oscillator[0]
                                  ->getInnovNumber(),
                          0,
                          ++innovation_number_,
                          true));
          this->connections_.push_back(first_to_first);
        }
      }
    }

    // add neighbours to queue
    // watch out for parent
    if (_lastPart == this->bodyMap_)
    {
      if (_lastPart->neighbours[0].name != "empty")
      {
        this->toParse_.push_back(&(_lastPart->neighbours[0]));
      }
    }
    for (size_t i = 1; i < MAX_SLOTS; ++i)
    {
      if (_lastPart->neighbours[i].name != "empty")
      {
        this->toParse_.push_back(&(_lastPart->neighbours[i]));
      }
    }
  }

  // add input neurons and add connections between them and neurons on core
  for (size_t i = 0; i < 6; ++i)
  {
    // add neuron
    std::map< std::string, double > empty;

    cppneat::NeuronPtr neuron(
            new cppneat::Neuron(bodyMap_->name + "-in-" + std::to_string(i),
                                cppneat::Neuron::INPUT_LAYER,
                                cppneat::Neuron::INPUT,
                                empty));
    cppneat::NeuronGenePtr neuron_gene(
            new cppneat::NeuronGene(neuron, ++innovation_number_, true));
    this->neurons_.push_back(neuron_gene);
    this->inputNeurons_.push_back(neuron_gene);
    CoordsTriple coord(0, 0, 0);
    this->coordinates_[neuron_gene] = coord;

    // connect to neighbouring oscillators
    for (size_t i = 0; i < MAX_SLOTS; ++i)
    {
      if (this->bodyMap_->neighbours[i].type == "ActiveHinge")
      {
        cppneat::ConnectionGenePtr input_to_first(
                new cppneat::ConnectionGene(
                        bodyMap_->neighbours[i].differential_oscillator[0]
                                ->getInnovNumber(),
                        neuron_gene->getInnovNumber(),
                        0,
                        ++innovation_number_,
                        true));
        this->connections_.push_back(input_to_first);
      }
    }
  }
}

///////////////////////////////////////////////////
// only works for slot: 0 currently
void BodyParser::ParseYaml(
        BodyPart *_module,
        YAML::Node &_yaml)
{
  _module->name = _yaml["id"].as< std::string >();
  _module->type = _yaml["type"].as< std::string >();
  _module->arity = arity_[_module->type];

  // add differential oscillator in case of the current part being an
  // ActiveHinge output is also added here needs to be done before adding the
  // parent as a neighbour to the children since the neighbours are remembered
  // by value and not reference
  this->GenerateOscillator(_module);

  if (_yaml["children"].IsDefined())
  {
    YAML::Node yaml_offsprings = _yaml["children"];
    for (size_t parents_slot = 0; parents_slot < MAX_SLOTS; ++parents_slot)
    {
      if (yaml_offsprings[parents_slot].IsDefined())
      {
        BodyPart *child = &_module->neighbours[parents_slot];
        this->initPart(child);
        // add parent as neighbour of child
        child->neighbours[0] = *_module;

        child->rotation = this->calculateRotation(
                (_module->arity == 4) ? 4 : 2,
                parents_slot,
                _module->rotation);

        int offsprings_x, offsprings_y;
        std::tie(offsprings_x, offsprings_y) =
                this->setCoordinates(child->rotation, _module->x, _module->y);
        child->x = offsprings_x;
        child->y = offsprings_y;

        this->bodyToNode_[child] = yaml_offsprings[parents_slot];
        this->toParse_.push_back(child);
      }
    }
  }
}
