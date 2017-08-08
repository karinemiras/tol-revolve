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

#include <string>
#include <vector>

#include <boost/make_shared.hpp>

#include "Helper.h"

using namespace tol;

const std::vector< revolve::brain::ActuatorPtr > Helper::createWrapper(
        const std::vector< revolve::gazebo::MotorPtr > &original)
{
  std::vector< revolve::brain::ActuatorPtr > result;
  for (unsigned int i = 0; i < original.size(); i++)
  {
    result.push_back(boost::make_shared< tol::Actuator >(
            tol::Actuator(original[i])));
  }

  return result;
}

const std::vector< revolve::brain::SensorPtr > Helper::createWrapper(
        const std::vector< revolve::gazebo::SensorPtr > &original)
{
  std::vector< revolve::brain::SensorPtr > result;
  for (unsigned int i = 0; i < original.size(); i++)
  {
    result.push_back(boost::make_shared< tol::Sensor >(
            tol::Sensor(original[i])));
  }

  return result;
}

Helper::RobotType Helper::parseRobotType(const std::string &value)
{
  if (value.compare("spider9") == 0)
  {
    return RobotType::spider9;
  }
  if (value.compare("spider13") == 0)
  {
    return RobotType::spider13;
  }
  if (value.compare("spider17") == 0)
  {
    return RobotType::spider17;
  }

  if (value.compare("gecko7") == 0)
  {
    return RobotType::gecko7;
  }
  if (value.compare("gecko12") == 0)
  {
    return RobotType::gecko12;
  }
  if (value.compare("gecko17") == 0)
  {
    return RobotType::gecko17;
  }

  if (value.compare("snake5") == 0)
  {
    return RobotType::snake5;
  }
  if (value.compare("snake7") == 0)
  {
    return RobotType::snake7;
  }
  if (value.compare("snake9") == 0)
  {
    return RobotType::snake9;
  }

  if (value.compare("babyA") == 0)
  {
    return RobotType::babyA;
  }
  if (value.compare("babyB") == 0)
  {
    return RobotType::babyB;
  }
  if (value.compare("babyC") == 0)
  {
    return RobotType::babyC;
  }

  // default value
  std::cerr << "Impossible to parse robot type (" << value
            << ")\nThrowing exception!" << std::endl;
  throw std::invalid_argument("robot type impossible to parse");
}

std::ostream &operator<<(std::ostream &os,
                         tol::Helper::RobotType type)
{
  switch (type)
  {
    case tol::Helper::RobotType::spider9 :
      os << "spider9";
      break;
    case tol::Helper::RobotType::spider13:
      os << "spider13";
      break;
    case tol::Helper::RobotType::spider17:
      os << "spider17";
      break;
    case tol::Helper::RobotType::gecko7  :
      os << "gecko7";
      break;
    case tol::Helper::RobotType::gecko12 :
      os << "gecko12";
      break;
    case tol::Helper::RobotType::gecko17 :
      os << "gecko17";
      break;
    case tol::Helper::RobotType::snake5  :
      os << "snake5";
      break;
    case tol::Helper::RobotType::snake7  :
      os << "snake7";
      break;
    case tol::Helper::RobotType::snake9  :
      os << "snake9";
      break;
    case tol::Helper::RobotType::babyA   :
      os << "babyA";
      break;
    case tol::Helper::RobotType::babyB   :
      os << "babyB";
      break;
    case tol::Helper::RobotType::babyC   :
      os << "babyC";
      break;
    default      :
      os.setstate(std::ios_base::failbit);
  }
  return os;
}
