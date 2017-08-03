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

#include "test_FakeLightSensor.h"

#define BOOST_TEST_MODULE FakeLightSensor test
#define BOOST_TEST_DYN_LINK
#include <boost/test/included/unit_test.hpp>

#include <iostream>
#include <cmath>

const double pi = std::acos(-1);
const double angle_15 = pi/12;
const double angle_52_5 = 7*pi/24;

void test_sensor_angle(double target_angle, double fov,
                       const ignition::math::Pose3d robot_sensor_offset,
                       const ignition::math::Vector3d light_position,
                       double eps = 0.0001,
                       std::string text = ""
                      )
{

    TestFakeLightSensor sensor(
        fov,
        robot_sensor_offset,
        light_position
    );

    auto angle = sensor.expose_light_angle();
    BOOST_TEST((std::fabs(angle - target_angle) <= eps),
        text << angle << " - " << target_angle << " <= " << eps
        << " does not hold (angle test)"
    );
}

void test_sensor_distance(double target_distance, double fov,
                          const ignition::math::Pose3d robot_sensor_offset,
                          const ignition::math::Vector3d light_position,
                          double eps = 0.0001,
                          std::string text = ""
                         )
{

    TestFakeLightSensor sensor(
        fov,
        robot_sensor_offset,
        light_position
    );

    auto distance = sensor.expose_light_distance();
    BOOST_TEST((std::fabs(distance - target_distance) <= eps),
        text << distance << " - " << target_distance << " <= " << eps
        << " does not hold (distance test)"
    );
}


BOOST_AUTO_TEST_CASE(fake_sensor_distance_different_lights)
{

    ignition::math::Pose3d robot_pose(0,0,0,0,0,0);

    test_sensor_distance(
        50,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,50,0)
    );

    test_sensor_distance(
        50,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,0,50)
    );

    test_sensor_distance(
        5,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,0,5)
    );

    test_sensor_distance(
        50,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,0,-50)
    );

    test_sensor_distance(
        5,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,0,-5)
    );

    test_sensor_distance(
        std::sin(pi/4),
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,.5,.5)
    );

    test_sensor_distance(
        std::sin(pi/4)*2,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,1,1)
    );

    test_sensor_distance(
        .5,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(1,0,.5)
    );

    test_sensor_distance(
        .5,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(-1,0,.5)
    );

    test_sensor_distance(
        std::sin(pi/4),
        150.0f,
        robot_pose,
        ignition::math::Vector3d(100,.5,.5)
    );
}

BOOST_AUTO_TEST_CASE(fake_sensor_angle_different_lights)
{

    ignition::math::Pose3d robot_pose(0,0,0,0,0,0);

    test_sensor_angle(
        0,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,50,0)
    );

    test_sensor_angle(
        1,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,0,50)
    );

    test_sensor_angle(
        1,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,0,5)
    );

    test_sensor_angle(
        -1,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,0,-50)
    );

    test_sensor_angle(
        -1,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,0,-5)
    );

    test_sensor_angle(
        std::sin(pi/4),
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,50,50)
    );
    test_sensor_angle(
        std::sin(pi/4),
        150.0f,
        robot_pose,
        ignition::math::Vector3d(0,.5,.5)
    );


    test_sensor_angle(
        1,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(1,0,.5)
    );

    test_sensor_angle(
        1,
        150.0f,
        robot_pose,
        ignition::math::Vector3d(-1,0,.5)
    );

    test_sensor_angle(
        std::sin(pi/4),
        150.0f,
        robot_pose,
        ignition::math::Vector3d(100,.5,.5)
    );
}

BOOST_AUTO_TEST_CASE(fake_sensor_angle_different_sensors_positions)
{

    ignition::math::Vector3d light_position(0,0,0);

    test_sensor_angle(
        0,
        150.0f,
        ignition::math::Pose3d(0,0,0,0,0,0),
        light_position
    );

    test_sensor_angle(
        1,
        150.0f,
        ignition::math::Pose3d(0,0,-50,0,0,0),
        light_position
    );

    test_sensor_angle(
        -1,
        150.0f,
        ignition::math::Pose3d(0,0,50,0,0,0),
        light_position
    );

    test_sensor_angle(
        1,
        150.0f,
        ignition::math::Pose3d(0,0,-1,0,0,0),
        light_position
    );

    test_sensor_angle(
        -1,
        150.0f,
        ignition::math::Pose3d(0,0,1,0,0,0),
        light_position
    );

    test_sensor_angle(
        std::sin(pi/4),
        150.0f,
        ignition::math::Pose3d(0,-1,-1,0,0,0),
        light_position
    );
}

BOOST_AUTO_TEST_CASE(fake_sensor_angle_different_sensors_angles)
{

    ignition::math::Vector3d light_position(0,0,0);

    test_sensor_angle(
        .1,
        150.0f,
        ignition::math::Pose3d(0,.5,0,1,0,0),
        light_position
    );

    test_sensor_angle(
        .1,
        150.0f,
        ignition::math::Pose3d(0,.5,0,0,-1,0),
        light_position
    );

    test_sensor_angle(
        .1,
        150.0f,
        ignition::math::Pose3d(0,.5,0,0,0,.1),
        light_position
    );

    test_sensor_angle(
        .1,
        150.0f,
        ignition::math::Pose3d(0,.5,0,0,0,0),
        light_position
    );

    test_sensor_angle(
        .1,
        150.0f,
        ignition::math::Pose3d(0,.5,0,0,0,0),
        light_position
    );

    test_sensor_angle(
        std::sin(pi/4),
        150.0f,
        ignition::math::Pose3d(0,.5,0,0,0,0),
        light_position
    );
}

