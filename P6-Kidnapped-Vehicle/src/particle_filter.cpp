/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * DONE: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * DONE: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // DONE: Set the number of particles
  std::default_random_engine gen;

  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  for (int i=0; i<num_particles; ++i) {
    Particle p;
    
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1;

    particles.push_back(p);
    weights.push_back(1.0);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * DONE: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine gen;

  for (auto& p: particles) {
    double prev_x = p.x;
    double prev_y = p.y;
    double prev_theta = p.theta;

    double x, y, theta;

    
    if (fabs(yaw_rate) < 0.001) {
      theta = prev_theta;
      x = prev_x + velocity * delta_t * cos(prev_theta);
      y = prev_y + velocity * delta_t * sin(prev_theta);
    } else {
      theta = prev_theta + yaw_rate * delta_t;
      x = prev_x + velocity / yaw_rate * (sin(theta) - sin(prev_theta));
      y = prev_y + velocity / yaw_rate * (cos(prev_theta) - cos(theta));
    }

    std::normal_distribution<double> dist_x(x, std_pos[0]);
    std::normal_distribution<double> dist_y(y, std_pos[1]);
    std::normal_distribution<double> dist_theta(theta, std_pos[2]);

    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * DONE: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  for (auto& o: observations) {
    double min_distance = std::numeric_limits<double>::max();

    for (const auto& p: predicted) {
      double distance = dist(o.x, o.y, p.x, p.y);
      
      if (distance < min_distance) {
        min_distance = distance;
        o.id = p.id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * DONE: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  double sigma_x = std_landmark[0];
  double sigma_y = std_landmark[1];
  double gauss_norm = 1 / (2 * M_PI * sigma_x * sigma_y);

  double total_weight = 0.0;

  for (auto& p: particles) {
    vector<LandmarkObs> predictions;

    // select valid landmarks only
    for (const auto& landmark: map_landmarks.landmark_list) {
      if (dist(p.x, p.y, landmark.x_f, landmark.y_f) <= sensor_range) {
        predictions.push_back(LandmarkObs{landmark.id_i, landmark.x_f, landmark.y_f});
      }
    }

    // convert observation to global (map) coordinates
    double cos_theta = cos(p.theta);
    double sin_theta = sin(p.theta);

    vector<LandmarkObs> obs_global;
    for (const auto& obs: observations) {
      LandmarkObs obs_g;
      obs_g.id = obs.id;
      obs_g.x = p.x + cos_theta * obs.x - sin_theta * obs.y;
      obs_g.y = p.y + sin_theta * obs.x + cos_theta * obs.y;
      obs_global.push_back(obs_g);
    }

    // find associations
    dataAssociation(predictions, obs_global);

    // reset weight
    p.weight = 1;

    // update weights based on associations
    for (const auto& obs: obs_global) {
      Map::single_landmark_s landmark;
      for (const auto& lm: map_landmarks.landmark_list) {
        if (lm.id_i == obs.id) {
          landmark = lm;
          break;
        }
      }
      
      double exponent = pow(obs.x - landmark.x_f, 2) / (2 * pow(sigma_x, 2))
                    + pow(obs.y - landmark.y_f, 2) / (2 * pow(sigma_y, 2));
      double weight = gauss_norm * exp(-exponent);
      p.weight *= weight;
    }
    total_weight += p.weight;
  }

  // normalize weights
  for (int i=0; i < num_particles; ++i) {
    particles[i].weight /= total_weight;
    weights[i] = particles[i].weight;
  }
}

void ParticleFilter::resample() {
  /**
   * DONE: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  std::default_random_engine gen;
  std::discrete_distribution<> d(weights.begin(), weights.end());
  d.probabilities();
  vector<Particle> new_particles;
  for (int i=0; i < num_particles; ++i) {
    new_particles.push_back(particles[d(gen)]);
  }
  particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}