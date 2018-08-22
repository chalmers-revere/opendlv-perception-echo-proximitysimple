/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <thread>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ( (0 == commandlineArguments.count("cid")) || (0 == commandlineArguments.count("a0")) || (0 == commandlineArguments.count("a1")) || (0 == commandlineArguments.count("z0")) || (0 == commandlineArguments.count("z1")) || (0 == commandlineArguments.count("id")) ) {
    std::cerr << argv[0] << " accesses video data using shared memory and sends it as a stream over an OD4 session." << std::endl;
    std::cerr << "         --verbose:          enable diagnostic output" << std::endl;
    std::cerr << "         --a0:               angle range, lower (radians)" << std::endl;
    std::cerr << "         --a1:               angle range, upper (radians)" << std::endl;
    std::cerr << "         --z0:               height range, lower (meters)" << std::endl;
    std::cerr << "         --z1:               height range, upper (meters)" << std::endl;
    std::cerr << "         --id:               sender stamp of output distance reading" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111 --a0=-0.2 --a1=0.2 --z0=1.0 --z1=1.3 --id=0 --verbose" << std::endl;
    retCode = 1;
  } else {
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};

    float const A0{static_cast<float>(std::stof(commandlineArguments["a0"]))};
    float const A1{static_cast<float>(std::stof(commandlineArguments["a1"]))};
    float const Z0{static_cast<float>(std::stof(commandlineArguments["z0"]))};
    float const Z1{static_cast<float>(std::stof(commandlineArguments["z1"]))};

    uint16_t const CID{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    uint32_t const ID{static_cast<uint32_t>(std::stoi(commandlineArguments["id"]))};

    float verticalAngles16[] = {-15.0f, -13.0f, -11.0f, -9.0f, -7.0f, -5.0f, -3.0f, -1.0f, 1.0f, 3.0f, 5.0f, 7.0f, 9.0f, 11.0f, 13.0f, 15.0f};
    float verticalAngles12[] = {-30.67f, -29.33f, -25.33f, -21.32f, -17.32f, -13.31f, -9.31f, -5.3f, -1.3f, 2.71f, 6.71f, 10.72f};
    float verticalAngles11[] = {-28.0f, -26.66f, -22.66f, -18.65f, -14.65f, -10.64f, -6.64f, -2.63f, 1.37f, 5.38f, 9.38f};
    float verticalAngles9[] = {-23.99f, -19.99f, -15.98f, -11.98f, -7.97f, -3.97f, 0.04f, 4.04f, 8.05f};

    cluon::OD4Session od4{CID};

    auto onPointCloudReading{[&A0, &A1, &Z0, &Z1, &ID, &VERBOSE, &od4, &verticalAngles16, &verticalAngles12, &verticalAngles11, verticalAngles9](cluon::data::Envelope &&envelope)
      {
        auto pointCloudReading = cluon::extractMessage<opendlv::proxy::PointCloudReading>(std::move(envelope));

        auto distances = pointCloudReading.distances();
        float const startAzimuth = pointCloudReading.startAzimuth();
        float const endAzimuth = pointCloudReading.endAzimuth();
        uint8_t const entriesPerAzimuth = pointCloudReading.entriesPerAzimuth();

        if (entriesPerAzimuth != 12) {
          return;
        }
        
        uint32_t numberOfPoints = distances.size() / 2;
        uint32_t numberOfAzimuths = numberOfPoints / entriesPerAzimuth;
        float azimuthIncrement = (endAzimuth - startAzimuth) / numberOfAzimuths;

        bool foundObject = false;
        float distanceToObject = 0.0f;

        uint32_t index = 0;
        float azimuth = startAzimuth;
        for (uint32_t azimuthIndex = 0; azimuthIndex < numberOfAzimuths; azimuthIndex++) {
          for (uint32_t sensorIndex = 0; sensorIndex < entriesPerAzimuth; sensorIndex++) {
        
            if (azimuth > A0 && azimuth < A1) {
            
              float verticalAngle = 0;
              if (16 == entriesPerAzimuth) {
                verticalAngle = verticalAngles16[sensorIndex];
              } else if (12 == entriesPerAzimuth) {
                verticalAngle = verticalAngles12[sensorIndex];
              } else if (11 == entriesPerAzimuth) {
                verticalAngle = verticalAngles11[sensorIndex];
              } else if (9 == entriesPerAzimuth) {
                verticalAngle = verticalAngles9[sensorIndex];
              }

              auto byte0 = distances[index++];
              auto byte1 = distances[index++];
              float distance = static_cast<float>( ((0xff & byte0) << 8) | (0xff & byte1) );
        
              distance /= 100.0f;

              if (distance > 1.0f) {
                float z = distance * sinf(verticalAngle * 3.14f / 180.0f);

                if (z > Z0 && z < Z1) {
                  float xyDistance = distance * cosf(verticalAngle * 3.14f / 180.0f);
                  if (!foundObject || (foundObject && xyDistance < distanceToObject)) {
                    distanceToObject = xyDistance;
                  }
                  foundObject = true;
                }
              }
            }
          }
           
          azimuth += azimuthIncrement;
        }

        if (foundObject) {
          opendlv::proxy::DistanceReading distanceReading;
          distanceReading.distance(distanceToObject);

          cluon::data::TimeStamp sampleTime = cluon::time::now();
          od4.send(distanceReading, sampleTime, ID);

          if (VERBOSE) {
            std::cout << "Sent distance: " << distanceToObject << " with id " << ID << std::endl;
          }
        }
      }};

    od4.dataTrigger(opendlv::proxy::PointCloudReading::ID(), onPointCloudReading);

    while (od4.isRunning()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }

  return retCode;
}

