/*
 * particle_filter.h
 *
 * 2D particle filter class.
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"


/**
 * @brief The Particle struct
 */
struct Particle
{
  Particle();

  void init(int initId, double initX, double initY, double initTheta);
  void setAssociations(std::vector<int> associations, std::vector<double> senseX, std::vector<double> senseY);
  std::string getAssociations() const;
  std::string getSenseX() const;
  std::string getSenseY() const;
  friend std::ostream &operator<<(std::ostream &out, const Particle &p);
  friend bool operator<(const Particle &p1, const Particle &p2);

	int id;
	double x;
	double y;
	double theta;
	double weight;
	std::vector<int> associations;
  std::vector<double> senseX;
  std::vector<double> senseY;
};


/**
 * @brief The ParticleFilter class
 */
class ParticleFilter
{
  typedef std::vector<LandmarkObs> TLandmarkObservations;

public:
	
  /**
   * Constructs a new particle filter.
   * @param num_particles Number of particles
   */
  ParticleFilter();

public:

	/**
	 * init Initializes particle filter by initializing particles to Gaussian
	 *   distribution around first position and all the weights to 1.
	 * @param x Initial x position [m] (simulated estimate from GPS)
	 * @param y Initial y position [m]
	 * @param theta Initial orientation [rad]
	 * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 */
  void init(double x, double y, double theta, double stdPos[]);

  /**
   * @brief getBestParticle
   * @return Best particle, if existing, null otherwise
   */
  const Particle *getBestParticle() const;

	/**
	 * prediction Predicts the state for the next time step
	 *   using the process model.
	 * @param delta_t Time between time step t and t+1 in measurements [s]
	 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 * @param velocity Velocity of car from t to t+1 [m/s]
	 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	 */
  void prediction(double deltaT, double stdPos[], double velocity, double yawRate);
	
	/**
	 * dataAssociation Finds which observations correspond to which landmarks (likely by using
	 *   a nearest-neighbors data association).
	 * @param predicted Vector of predicted landmark observations
	 * @param observations Vector of landmark observations
	 */
  void dataAssociation(const TLandmarkObservations &predicted, TLandmarkObservations &observations);
	
	/**
	 * updateWeights Updates the weights for each particle based on the likelihood of the 
	 *   observed measurements. 
	 * @param sensor_range Range [m] of sensor
	 * @param std_landmark[] Array of dimension 2 [Landmark measurement uncertainty [x [m], y [m]]]
	 * @param observations Vector of landmark observations
	 * @param map Map class containing map landmarks
	 */
  void updateWeights(double sensor_range, double stdLandmark[], const TLandmarkObservations &observations, const Map &mapLandmarks);
	
	/**
	 * resample Resamples from the updated set of particles to form
	 *   the new set of particles.
	 */
	void resample();

	/**
   * @return whether particle filter is initialized yet or not.
   */
  const bool isInitialized() const;

private:
  typedef std::vector<Particle> TParticles;

  /** Set of current particles */
  TParticles mParticles;

  /** Number of particles to draw */
  int mNumParticles;

  /** Flag, if filter is initialized */
  bool mIsInitialized;
};



#endif /* PARTICLE_FILTER_H_ */
