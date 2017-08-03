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

#ifndef TESTYAMLBODYPARSER_H
#define TESTYAMLBODYPARSER_H

#include <string>

namespace tol {

static const std::string Spider9_yaml_source = "---\n"
        "body:\n"
        "  id          : Core\n"
        "  type        : Core\n"
        "  children    :\n"
        "    0:\n"
        "      id          : Leg00Joint\n"
        "      type        : ActiveHinge\n"
        "      orientation : 90\n"
        "      children    :\n"
        "        1:\n"
        "          id          : Leg00\n"
        "          type        : FixedBrick\n"
        "          orientation : -90\n"
        "          children    :\n"
        "            1:\n"
        "              id          : Leg01Joint\n"
        "              type        : ActiveHinge\n"
        "              children    :\n"
        "                1:\n"
        "                  id          : Leg01\n"
        "                  type        : FixedBrick\n"
        "                  orientation : 0\n"
        "    1:\n"
        "      id          : Leg10Joint\n"
        "      type        : ActiveHinge\n"
        "      orientation : 90\n"
        "      children    :\n"
        "        1:\n"
        "          id          : Leg10\n"
        "          type        : FixedBrick\n"
        "          orientation : -90\n"
        "          children    :\n"
        "            1:\n"
        "              id          : Leg11Joint\n"
        "              type        : ActiveHinge\n"
        "              children    :\n"
        "                1:\n"
        "                  id          : Leg11\n"
        "                  type        : FixedBrick\n"
        "                  orientation : 0\n"
        "    2:\n"
        "      id          : Leg20Joint\n"
        "      type        : ActiveHinge\n"
        "      orientation : 90\n"
        "      children    :\n"
        "        1:\n"
        "          id          : Leg20\n"
        "          type        : FixedBrick\n"
        "          orientation : -90\n"
        "          children    :\n"
        "            1:\n"
        "              id          : Leg21Joint\n"
        "              type        : ActiveHinge\n"
        "              children    :\n"
        "                1:\n"
        "                  id          : Leg21\n"
        "                  type        : FixedBrick\n"
        "                  orientation : 0\n"
        "    3:\n"
        "      id          : Leg30Joint\n"
        "      type        : ActiveHinge\n"
        "      orientation : 90\n"
        "      children    :\n"
        "        1:\n"
        "          id          : Leg30\n"
        "          type        : FixedBrick\n"
        "          orientation : -90\n"
        "          children    :\n"
        "            1:\n"
        "              id          : Leg31Joint\n"
        "              type        : ActiveHinge\n"
        "              children    :\n"
        "                1:\n"
        "                  id          : Leg31\n"
        "                  type        : FixedBrick\n"
        "                  orientation : 0";

static const std::string Spider13_yaml_source = "---\n"
        "body:\n"
        "  id          : Core\n"
        "  type        : Core\n"
        "  children    :\n"
        "    0:\n"
        "      id          : Leg00Joint\n"
        "      type        : ActiveHinge\n"
        "      orientation : 90\n"
        "      children    :\n"
        "        1:\n"
        "          id          : Leg00\n"
        "          type        : FixedBrick\n"
        "          orientation : -90\n"
        "          children    :\n"
        "            1:\n"
        "              id          : Leg01Joint\n"
        "              type        : ActiveHinge\n"
        "              orientation : 0\n"
        "              children    :\n"
        "                1:\n"
        "                  id          : Leg01\n"
        "                  type        : FixedBrick\n"
        "                  orientation : 0\n"
        "                  children    :\n"
        "                    1:\n"
        "                      id          : Leg02Joint\n"
        "                      type        : ActiveHinge\n"
        "                      orientation : 90\n"
        "                      children    :\n"
        "                        1:\n"
        "                          id          : Leg02\n"
        "                          type        : FixedBrick\n"
        "                          orientation : -90\n"
        "    1:\n"
        "      id          : Leg10Joint\n"
        "      type        : ActiveHinge\n"
        "      orientation : 90\n"
        "      children    :\n"
        "        1:\n"
        "          id          : Leg10\n"
        "          type        : FixedBrick\n"
        "          orientation : -90\n"
        "          children    :\n"
        "            1:\n"
        "              id          : Leg11Joint\n"
        "              type        : ActiveHinge\n"
        "              orientation : 0\n"
        "              children    :\n"
        "                1:\n"
        "                  id          : Leg11\n"
        "                  type        : FixedBrick\n"
        "                  orientation : 0\n"
        "                  children    :\n"
        "                    1:\n"
        "                      id          : Leg12Joint\n"
        "                      type        : ActiveHinge\n"
        "                      orientation : 90\n"
        "                      children    :\n"
        "                        1:\n"
        "                          id          : Leg12\n"
        "                          type        : FixedBrick\n"
        "                          orientation : -90\n"
        "    2:\n"
        "      id          : Leg20Joint\n"
        "      type        : ActiveHinge\n"
        "      orientation : 90\n"
        "      children    :\n"
        "        1:\n"
        "          id          : Leg20\n"
        "          type        : FixedBrick\n"
        "          orientation : -90\n"
        "          children    :\n"
        "            1:\n"
        "              id          : Leg21Joint\n"
        "              type        : ActiveHinge\n"
        "              orientation : 0\n"
        "              children    :\n"
        "                1:\n"
        "                  id          : Leg21\n"
        "                  type        : FixedBrick\n"
        "                  orientation : 0\n"
        "                  children    :\n"
        "                    1:\n"
        "                      id          : Leg22Joint\n"
        "                      type        : ActiveHinge\n"
        "                      orientation : 90\n"
        "                      children    :\n"
        "                        1:\n"
        "                          id          : Leg22\n"
        "                          type        : FixedBrick\n"
        "                          orientation : -90\n"
        "    3:\n"
        "      id          : Leg30Joint\n"
        "      type        : ActiveHinge\n"
        "      orientation : 90\n"
        "      children    :\n"
        "        1:\n"
        "          id          : Leg30\n"
        "          type        : FixedBrick\n"
        "          orientation : -90\n"
        "          children    :\n"
        "            1:\n"
        "              id          : Leg31Joint\n"
        "              type        : ActiveHinge\n"
        "              orientation : 0\n"
        "              children    :\n"
        "                1:\n"
        "                  id          : Leg31\n"
        "                  type        : FixedBrick\n"
        "                  orientation : 0\n"
        "                  children    :\n"
        "                    1:\n"
        "                      id          : Leg32Joint\n"
        "                      type        : ActiveHinge\n"
        "                      orientation : 90\n"
        "                      children    :\n"
        "                        1:\n"
        "                          id          : Leg32\n"
        "                          type        : FixedBrick\n"
        "                          orientation : -90";

static const std::string Spider17_yaml_source = "---\n"
        "body:\n"
        "  id          : Core\n"
        "  type        : Core\n"
        "  children    :\n"
        "    0:\n"
        "      id          : Leg00Joint\n"
        "      type        : ActiveHinge\n"
        "      orientation : 90\n"
        "      children    :\n"
        "        1:\n"
        "          id          : Leg00\n"
        "          type        : FixedBrick\n"
        "          orientation : -90\n"
        "          children    :\n"
        "            1:\n"
        "              id          : Leg01Joint\n"
        "              type        : ActiveHinge\n"
        "              orientation : 0\n"
        "              children    :\n"
        "                1:\n"
        "                  id          : Leg01\n"
        "                  type        : FixedBrick\n"
        "                  orientation : 0\n"
        "                  children    :\n"
        "                    1:\n"
        "                      id          : Leg02Joint\n"
        "                      type        : ActiveHinge\n"
        "                      orientation : 90\n"
        "                      children    :\n"
        "                        1:\n"
        "                          id          : Leg02\n"
        "                          type        : FixedBrick\n"
        "                          orientation : -90\n"
        "                          children    :\n"
        "                           1:\n"
        "                             id          : Leg03Joint\n"
        "                             type        : ActiveHinge\n"
        "                             orientation : 0\n"
        "                             children    :\n"
        "                               1:\n"
        "                                 id          : Leg03\n"
        "                                 type        : FixedBrick\n"
        "                                 orientation : 0\n"
        "    1:\n"
        "      id          : Leg10Joint\n"
        "      type        : ActiveHinge\n"
        "      orientation : 90\n"
        "      children    :\n"
        "        1:\n"
        "          id          : Leg10\n"
        "          type        : FixedBrick\n"
        "          orientation : -90\n"
        "          children    :\n"
        "            1:\n"
        "              id          : Leg11Joint\n"
        "              type        : ActiveHinge\n"
        "              orientation : 0\n"
        "              children    :\n"
        "                1:\n"
        "                  id          : Leg11\n"
        "                  type        : FixedBrick\n"
        "                  orientation : 0\n"
        "                  children    :\n"
        "                    1:\n"
        "                      id          : Leg12Joint\n"
        "                      type        : ActiveHinge\n"
        "                      orientation : 90\n"
        "                      children    :\n"
        "                        1:\n"
        "                          id          : Leg12\n"
        "                          type        : FixedBrick\n"
        "                          orientation : -90\n"
        "                          children    :\n"
        "                           1:\n"
        "                             id          : Leg13Joint\n"
        "                             type        : ActiveHinge\n"
        "                             orientation : 0\n"
        "                             children    :\n"
        "                               1:\n"
        "                                 id          : Leg13\n"
        "                                 type        : FixedBrick\n"
        "                                 orientation : 0\n"
        "    2:\n"
        "      id          : Leg20Joint\n"
        "      type        : ActiveHinge\n"
        "      orientation : 90\n"
        "      children    :\n"
        "        1:\n"
        "          id          : Leg20\n"
        "          type        : FixedBrick\n"
        "          orientation : -90\n"
        "          children    :\n"
        "            1:\n"
        "              id          : Leg21Joint\n"
        "              type        : ActiveHinge\n"
        "              orientation : 0\n"
        "              children    :\n"
        "                1:\n"
        "                  id          : Leg21\n"
        "                  type        : FixedBrick\n"
        "                  orientation : 0\n"
        "                  children    :\n"
        "                    1:\n"
        "                      id          : Leg22Joint\n"
        "                      type        : ActiveHinge\n"
        "                      orientation : 90\n"
        "                      children    :\n"
        "                        1:\n"
        "                          id          : Leg22\n"
        "                          type        : FixedBrick\n"
        "                          orientation : -90\n"
        "                          children    :\n"
        "                           1:\n"
        "                             id          : Leg23Joint\n"
        "                             type        : ActiveHinge\n"
        "                             orientation : 0\n"
        "                             children    :\n"
        "                               1:\n"
        "                                 id          : Leg23\n"
        "                                 type        : FixedBrick\n"
        "                                 orientation : 0\n"
        "    3:\n"
        "      id          : Leg30Joint\n"
        "      type        : ActiveHinge\n"
        "      orientation : 90\n"
        "      children    :\n"
        "        1:\n"
        "          id          : Leg30\n"
        "          type        : FixedBrick\n"
        "          orientation : -90\n"
        "          children    :\n"
        "            1:\n"
        "              id          : Leg31Joint\n"
        "              type        : ActiveHinge\n"
        "              orientation : 0\n"
        "              children    :\n"
        "                1:\n"
        "                  id          : Leg31\n"
        "                  type        : FixedBrick\n"
        "                  orientation : 0\n"
        "                  children    :\n"
        "                    1:\n"
        "                      id          : Leg32Joint\n"
        "                      type        : ActiveHinge\n"
        "                      orientation : 90\n"
        "                      children    :\n"
        "                        1:\n"
        "                          id          : Leg32\n"
        "                          type        : FixedBrick\n"
        "                          orientation : -90\n"
        "                          children    :\n"
        "                           1:\n"
        "                             id          : Leg33Joint\n"
        "                             type        : ActiveHinge\n"
        "                             orientation : 0\n"
        "                             children    :\n"
        "                               1:\n"
        "                                 id          : Leg33\n"
        "                                 type        : FixedBrick\n"
        "                                 orientation : 0";

}

#endif // TESTYAMLBODYPARSER_H
