#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "particle_filter.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos)
  {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  //Set up parameters here
  double deltaT = 0.1; // Time elapsed between measurements [sec]
  double sensorRange = 50; // Sensor range [m]

  double sigmaPos [3] = {0.3, 0.3, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
  double sigmaLandmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]

  // Read map data
  Map map;
  if (!read_map_data("../data/map_data.txt", map))
  {
    cout << "Error: Could not open map file" << endl;
    return -1;
  }

  // Create particle filter
  ParticleFilter pf;

  h.onMessage([&pf,&map,&deltaT,&sensorRange,&sigmaPos,&sigmaLandmark](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
      if (s != "")
      {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          if (!pf.isInitialized())
          {
            // Sense noisy position data from the simulator
            double senseX = std::stod(j[1]["sense_x"].get<std::string>());
            double senseY = std::stod(j[1]["sense_y"].get<std::string>());
            double senseTheta = std::stod(j[1]["sense_theta"].get<std::string>());
            pf.init(senseX, senseY, senseTheta, sigmaPos);
          }
          else
          {
            // Predict the vehicle's next state from previous (noiseless control) data.
            double previous_velocity = std::stod(j[1]["previous_velocity"].get<std::string>());
            double previous_yawrate = std::stod(j[1]["previous_yawrate"].get<std::string>());
            pf.prediction(deltaT, sigmaPos, previous_velocity, previous_yawrate);
          }

          // receive noisy observation data from the simulator
          // sense_observations in JSON format [{obs_x,obs_y},{obs_x,obs_y},...{obs_x,obs_y}]
          vector<LandmarkObs> noisyObservations;
          string senseObservationsX = j[1]["sense_observations_x"];
          string senseObservationsY = j[1]["sense_observations_y"];

          std::vector<float> xSense;
          std::istringstream issX(senseObservationsX);

          std::copy(std::istream_iterator<float>(issX),
                    std::istream_iterator<float>(),
                    std::back_inserter(xSense));

          std::vector<float> ySense;
          std::istringstream issY(senseObservationsY);

          std::copy(std::istream_iterator<float>(issY),
                    std::istream_iterator<float>(),
                    std::back_inserter(ySense));

          for(int i = 0; i < xSense.size(); i++)
          {
            LandmarkObs obs;
            obs.x = xSense[i];
            obs.y = ySense[i];
            noisyObservations.push_back(obs);
          }

          // Update the weights and resample
          pf.updateWeights(sensorRange, sigmaLandmark, noisyObservations, map);
          pf.resample();

          // Calculate and output the average weighted error of the particle filter over all time steps so far.
          const Particle *bestParticle(pf.getBestParticle());
          if (bestParticle)
          {
            json msgJson;
            msgJson["best_particle_x"] = bestParticle->x;
            msgJson["best_particle_y"] = bestParticle->y;
            msgJson["best_particle_theta"] = bestParticle->theta;

            //Optional message data used for debugging particle's sensing and associations
            msgJson["best_particle_associations"] = bestParticle->getAssociations();
            msgJson["best_particle_sense_x"] = bestParticle->getSenseX();
            msgJson["best_particle_sense_y"] = bestParticle->getSenseY();

            auto msg = "42[\"best_particle\"," + msgJson.dump() + "]";
            // std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      }
      else
      {
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
  {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
  {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}























































































