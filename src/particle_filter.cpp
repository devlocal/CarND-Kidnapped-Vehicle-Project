/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <sstream>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Sets the number of particles. Initializes all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Adds random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  double stddev_x = std[0];
  double stddev_y = std[1];
  double stddev_theta = std[2];

  // Create normal distributions for x, y and theta
  std::normal_distribution<double> nd_x(x, stddev_x);
  std::normal_distribution<double> nd_y(y, stddev_y);
  std::normal_distribution<double> nd_theta(theta, stddev_theta);

  // Initialize particles
  particles.resize(num_particles);
  for (auto i = 0; i < num_particles; ++i) {
    Particle &particle = particles[i];
    particle.id = i;
    particle.x = nd_x(gen);
    particle.y = nd_y(gen);
    particle.theta = nd_theta(gen);
    particle.weight = 1;
  }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Adds measurements to each particle and adds random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  double stddev_x = std_pos[0];
  double stddev_y = std_pos[1];
  double stddev_theta = std_pos[2];
  double theta_dot_dt = yaw_rate * delta_t;
  double velocity_over_yaw = velocity / yaw_rate;

  for (Particle &particle: particles) {
    // Predict particles state
    double x = particle.x + velocity_over_yaw * (sin(particle.theta + theta_dot_dt) - sin(particle.theta));
    double y = particle.y + velocity_over_yaw * (cos(particle.theta) - cos(particle.theta + theta_dot_dt));
    double theta = particle.theta + theta_dot_dt;

    // Add gaussian noise
    std::normal_distribution<double> nd_x(x, stddev_x);
    std::normal_distribution<double> nd_y(y, stddev_y);
    std::normal_distribution<double> nd_theta(theta, stddev_theta);
    particle.x = nd_x(gen);
    particle.y = nd_y(gen);
    particle.theta = nd_theta(gen);
  }
}

void ParticleFilter::dataAssociation(const std::vector<LandmarkObs>& predicted, std::vector<LandmarkObs>& observations) {
	// Finds the predicted measurement that is closest to each observed measurement and assigns the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

  for (LandmarkObs &observation: observations) {
    auto compare = [observation](const LandmarkObs& landmark_a, const LandmarkObs& landmark_b) {
      double dist_sq_a = pow(landmark_a.x - observation.x, 2) + pow(landmark_a.y - observation.y, 2);
      double dist_sq_b = pow(landmark_b.x - observation.x, 2) + pow(landmark_b.y - observation.y, 2);
      return dist_sq_a < dist_sq_b;
    };
    auto landmark = std::min_element(predicted.cbegin(), predicted.cend(), compare);
    observation.id = landmark->id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// Updates the weights of each particle using a multi-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

  double sensor_range_sq = pow(sensor_range, 2);
  double sigma_x = std_landmark[0];
  double sigma_y = std_landmark[1];
  double _2_sigma_x_sq = 2.0 * pow(sigma_x, 2);
  double _2_sigma_y_sq = 2.0 * pow(sigma_y, 2);

  for (Particle &particle: particles) {
    double xp = particle.x;
    double yp = particle.y;
    double thetap = particle.theta;
    double cos_thetap = cos(thetap);
    double sin_thetap = sin(thetap);

    // Build a list of predicted observations within sensor_range radius from particle location
    std::vector<LandmarkObs> predicted_observations;
    for (const Map::single_landmark_s &landmark: map_landmarks.landmark_list) {
      double dist_sq = pow(landmark.x_f - xp, 2) + pow(landmark.y_f - yp, 2);
      if (dist_sq <= sensor_range_sq) {
        LandmarkObs obs;
        obs.id = landmark.id_i;
        obs.x = landmark.x_f;
        obs.y = landmark.y_f;
        predicted_observations.push_back(obs);
      }
    }

    // Convert landmark observations from particle coordinates to map coordinates
    unsigned long n_observations = observations.size();
    std::vector<LandmarkObs> map_observations(n_observations);

    for (int i = 0; i < n_observations; ++i) {
      double xc = observations[i].x;
      double yc = observations[i].y;

      // Apply homogeneous transformation to transform observation from car coordinates to map coordinates
      map_observations[i].x = xc * cos_thetap - yc * sin_thetap + xp;
      map_observations[i].y = xc * sin_thetap + yc * cos_thetap + yp;
    }

    // Associate observations with landmarks
    dataAssociation(predicted_observations, map_observations);

    // Set particle associations (used for visualization)
    std::vector<int> associations(n_observations);
    std::vector<double> sense_x(n_observations);
    std::vector<double> sense_y(n_observations);
    std::transform(map_observations.cbegin(), map_observations.cend(), associations.begin(),
                   [](const LandmarkObs& obs){ return obs.id; });
    std::transform(map_observations.cbegin(), map_observations.cend(), sense_x.begin(),
                   [](const LandmarkObs& obs){ return obs.x; });
    std::transform(map_observations.cbegin(), map_observations.cend(), sense_y.begin(),
                   [](const LandmarkObs& obs){ return obs.y; });
    SetAssociations(particle, associations, sense_x, sense_y);

    // Compute particle weight
    particle.weight = 1;
    for (const LandmarkObs &observation: map_observations) {
      int landmark_id = observation.id;
      // Note: landmark indices are 1-based
      Map::single_landmark_s landmark = map_landmarks.landmark_list[landmark_id - 1];

      double x = observation.x;
      double y = observation.y;
      double mu_x = landmark.x_f;
      double mu_y = landmark.y_f;

      double a = pow(x - mu_x, 2) / _2_sigma_x_sq;
      double b = pow(y - mu_y, 2) / _2_sigma_y_sq;
      double power = -(a + b);

      particle.weight *= 1.0 / (2.0 * M_PI * sigma_x * sigma_y) * exp(power);
    }
  }
}

void ParticleFilter::resample() {
	// Resamples particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  auto n_particles = particles.size();

  // Build a list of particle weights
  std::vector<double> weights(n_particles);
  std::transform(particles.cbegin(), particles.cend(), weights.begin(), [](const Particle& p){ return p.weight; });

  std::discrete_distribution<> distribution(weights.cbegin(), weights.cend());

  std::vector<Particle> new_particles(n_particles);
  for (auto i = 0; i < n_particles; ++i) {
    new_particles[i] = particles[distribution(gen)];
  }

  particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int> &associations,
                                     const std::vector<double> &sense_x, const std::vector<double> &sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations = associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
