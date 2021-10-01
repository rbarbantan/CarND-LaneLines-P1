#include <uWS/uWS.h>
#include <fstream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "trajectory.h"
#include "evaluate.h"
#include "vehicle.h"
#include "path_planner.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  const int MAX_PROPOSALS = 20;
  PathPlanner planner;
  Vehicle plan = Vehicle(0, 0, 0, 0, "CS");
  int frame = 0;
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  planner.setWaypoints(map_waypoints_s, map_waypoints_x, map_waypoints_y);

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy, &plan, &frame, &planner](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                     uWS::OpCode opCode)
              {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                if (length && length > 2 && data[0] == '4' && data[1] == '2')
                {

                  auto s = hasData(data);

                  if (s != "")
                  {
                    auto j = json::parse(s);

                    string event = j[0].get<string>();

                    if (event == "telemetry")
                    {
                      // j[1] is the data JSON object

                      // Main car's localization Data
                      double car_x = j[1]["x"];
                      double car_y = j[1]["y"];
                      double car_s = j[1]["s"];
                      double car_d = j[1]["d"];
                      double car_yaw = j[1]["yaw"];
                      double car_speed = j[1]["speed"];

                      // Previous path data given to the Planner
                      auto previous_path_x = j[1]["previous_path_x"];
                      auto previous_path_y = j[1]["previous_path_y"];
                      // Previous path's end s and d values
                      double end_path_s = j[1]["end_path_s"];
                      double end_path_d = j[1]["end_path_d"];

                      // Sensor Fusion Data, a list of all other cars on the same side
                      //   of the road.
                      auto sensor_fusion = j[1]["sensor_fusion"];
                      //measure_prediction_error(sensor_fusion);

                      json msgJson;

                      vector<vector<double>> solution;

                      //solution = get_simple_frenet(car_s, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                      int next_wp = NextWaypoint(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
                      //std::cout << car_s << ", "  << car_yaw  << "," << map_waypoints_s[next_wp] <<"\n";
                      
                      car_speed = car_speed / MPS_TO_MPH;
                      if (frame%1 == 0) {// compute every xth frame a new plan
                        // configuration data: speed limit, num_lanes, goal_s, goal_lane, and max_acceleration
                        vector<int> ego_config = {int(MAX_VELOCITY), 3, int(car_s+30), int(car_d/4), int(MAX_ACCELERATION-1)};
                        Vehicle ego = Vehicle(int(car_d/4), car_s, car_speed, 0, plan.state);
                        ego.configure(ego_config);
                        map<int, vector<Vehicle>> preds;
                        for (auto car : sensor_fusion) {
                          //std::cout << car[0] << ", " << car[1] << ", " << car[2] << ", " << car[3] << ", " << car[4] << ", " << car[5] << ", " << car[6] << "\n";
                          double s = car[5];
                          double d = car[6];
                          double vx = car[3];
                          double vy = car[4];
                          double v = sqrt(vx*vx + vy*vy);
                          if (d >0) {
                            Vehicle vehicle = Vehicle(int(d/4), s, v, 0);
                            preds.insert(std::pair<int, vector<Vehicle>>(car[0], vehicle.generate_predictions()));
                          }
                        }
                        vector<Vehicle> trajectory = ego.choose_next_state(preds);
                        //std::cout << trajectory[0].state << ", " << trajectory[1].state << "\n";
                        //std::cout << trajectory[1].lane << ", " << trajectory[1].s << ", " << trajectory[1].v << ", ";
                        //std::cout << car_d << ", " << car_s << ", " << car_speed << "\n";
                        if (trajectory[1].lane != plan.lane) {
                          printf("switch from %d to %d\n", plan.lane, trajectory[1].lane);
                        }
                        plan = trajectory[1];
                      }

                      planner.updateEgo(car_x, car_y, car_yaw, car_s, car_d, car_speed, previous_path_x, previous_path_y);
                      solution = planner.plan_trajectory(plan.s, plan.lane*4+2, plan.v);


                      frame += 1;
                      //evaluate_acceleration(solution);
                      //std::cout << "\n ============================== \n";
                      //double lowest_cost = 1.0;
                      //for (int i = 0; i < MAX_PROPOSALS; ++i)
                      //{
                      //  vector<vector<double>> proposal = propose_trajectory(car_x, car_y, car_yaw, car_s, previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                      //  double cost = evaluate_trajectory(proposal, sensor_fusion, map_waypoints_x, map_waypoints_y);
                      //  if (cost <= lowest_cost)
                      //  {
                      //    lowest_cost = cost;
                      //    solution = proposal;
                      //  }
                      //}
                      //std::cout << lowest_cost << "\n";

                      //print_vector(next_x_vals);
                      //std::cout << "---\n";
                      //print_vector(next_y_vals);
                      //std::cout << "---------------\n";

                      msgJson["next_x"] = solution[0];
                      msgJson["next_y"] = solution[1];

                      auto msg = "42[\"control\"," + msgJson.dump() + "]";

                      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    } // end "telemetry" if
                  }
                  else
                  {
                    // Manual driving
                    std::string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                  }
                } // end websocket if
              }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
                 { std::cout << "Connected!!!" << std::endl; });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length)
                    {
                      ws.close();
                      std::cout << "Disconnected" << std::endl;
                    });

  int port = 4567;
  if (h.listen("0.0.0.0", port))
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