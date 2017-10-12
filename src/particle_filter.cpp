/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include "particle_filter.h"


namespace
{
  bool isZero(double f)
  {
    return fabs(f) < std::numeric_limits<double>::epsilon();
  }

  double gaussian(double mu, double sigma, double x)
  {
    return exp(- pow(mu - x, 2)  / pow(sigma, 2) / 2.0) / sqrt(2.0 * M_PI * pow(sigma, 2));
  }
}


//=====================================================================


Particle::Particle()
{

}


void Particle::init(int initId, double initX, double initY, double initTheta)
{
  id = initId;
  x = initX;
  y = initY;
  theta = initTheta;
  weight = 1.0;
  associations.clear();
  senseX.clear();
  senseY.clear();
}


void Particle::setAssociations(std::vector<int> associations, std::vector<double> senseX, std::vector<double> senseY)
{
  //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  associations= associations;
  senseX = senseX;
  senseY = senseY;
}


std::string Particle::getAssociations() const
{
  std::stringstream ss;
  std::copy(associations.begin(), associations.end(), std::ostream_iterator<int>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}


std::string Particle::getSenseX() const
{
  std::stringstream ss;
  std::copy( senseX.begin(), senseX.end(), std::ostream_iterator<float>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}


std::string Particle::getSenseY() const
{
  std::stringstream ss;
  std::copy( senseY.begin(), senseY.end(), std::ostream_iterator<float>(ss, " "));
  std::string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}


std::ostream &operator<<(std::ostream &out, const Particle &p)
{
  out << "Particle {" << p.id << ", " << p.x << ", " << p.y << ", " << p.theta << "}";
  return out;
}


bool operator<(const Particle &p1, const Particle &p2)
{
  return p1.weight < p2.weight;
}


//=====================================================================


ParticleFilter::ParticleFilter()
  : mParticles()
  , mNumParticles(500)
  , mIsInitialized(false)
{

}


void ParticleFilter::init(double x, double y, double theta, double stdPos[])
{
  std::default_random_engine gen;
  std::normal_distribution<double> distX(x, stdPos[0]);
  std::normal_distribution<double> distY(y, stdPos[1]);
  std::normal_distribution<double> distTheta(theta, stdPos[2]);
  mParticles.resize(mNumParticles);
  int i = 0;
  for (TParticles::iterator p = mParticles.begin(); p != mParticles.end(); ++p)
  {
    ++i;
    p->init(i, distX(gen), distY(gen), distTheta(gen));
    std::cout << *p << std::endl;
  }
  mIsInitialized = true;
}


const Particle *ParticleFilter::getBestParticle() const
{
  const Particle *bestParticle = 0;
  for (TParticles::const_iterator p = mParticles.begin(); p != mParticles.end(); ++p)
  {
    if (bestParticle == 0 || p->weight > bestParticle->weight)
    {
      bestParticle = &*p;
    }
  }
  return bestParticle;
}


void ParticleFilter::prediction(double deltaT, double stdPos[], double velocity, double yawRate)
{
  std::default_random_engine gen;
  for (TParticles::iterator p = mParticles.begin(); p != mParticles.end(); ++p)
  {
    if (isZero(yawRate))
    {
      // straight case
      p->x += velocity * cos(p->theta) * deltaT;
      p->y += velocity * sin(p->theta) * deltaT;
      p->theta += yawRate * deltaT;
    }
    else
    {
      p->x += velocity / yawRate * ( sin(p->theta + yawRate * deltaT) - sin(p->theta));
      p->y += velocity / yawRate * (-cos(p->theta + yawRate * deltaT) + cos(p->theta));
      p->theta += yawRate * deltaT;
    }

    // add noise
    std::normal_distribution<double> distX(p->x, stdPos[0]);
    std::normal_distribution<double> distY(p->y, stdPos[1]);
    std::normal_distribution<double> distTheta(p->theta, stdPos[2]);
    p->x = distX(gen);
    p->y = distY(gen);
    p->theta = distTheta(gen);
  }
}


void ParticleFilter::dataAssociation(const TLandmarkObservations &predicted, TLandmarkObservations &observations)
{

	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
}


void ParticleFilter::updateWeights(double sensor_range, double stdLandmark[], const TLandmarkObservations &observations, const Map &mapLandmarks)
{
  double gaussNorm= (1.0 / (2.0 * M_PI * stdLandmark[0] * stdLandmark[1]));

  for (TParticles::iterator p = mParticles.begin(); p != mParticles.end(); ++p)
  {
    p->associations.clear();
    p->senseX.clear();
    p->senseY.clear();
    for (TLandmarkObservations::const_iterator o = observations.begin(); o != observations.end(); ++o)
    {
      p->associations.push_back(o->id);
      p->senseX.push_back(p->x + (cos(p->theta) * o->x) - (sin(p->theta) * o->y));
      p->senseY.push_back(p->y + (sin(p->theta) * o->x) + (cos(p->theta) * o->y));
    }

    p->weight = 1.0;
    for(int i = 0; i < p->associations.size(); ++i)
    {
      double minDist = std::numeric_limits<double>::max();
      const Map::single_landmark_s *minLandmark = 0;
      for (std::vector<Map::single_landmark_s>::const_iterator l = mapLandmarks.landmark_list.begin(); l != mapLandmarks.landmark_list.end(); ++l)
      {
        double dist = sqrt(pow(p->senseX[i] - l->x_f, 2) + pow(p->senseY[i] - l->y_f, 2));
        if (dist < minDist)
        {
          minDist = dist;
          minLandmark = &*l;
        }
      }
      if (minLandmark)
      {
        p->associations[i] = minLandmark->id_i;
        double gaussExp = pow(p->senseX[i] - minLandmark->x_f, 2) / 2.0 / pow(stdLandmark[0], 2) + pow(p->senseY[i] - minLandmark->y_f, 2) / 2.0 / pow(stdLandmark[1], 2);
        p->weight *= gaussNorm * exp(-gaussExp);
      }
    }
  }
}


void ParticleFilter::resample()
{
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  if (mParticles.size() > 0)
  {
    double maxWeight = std::max_element(mParticles.begin(), mParticles.end())->weight;

    TParticles newParticles;
    newParticles.resize(mParticles.size());

    double beta = 0.0;
    std::default_random_engine gen;
    std::uniform_int_distribution<int> distI(0, mParticles.size() - 1);
    std::uniform_real_distribution<double> distB(0.0, 2.0 * maxWeight);
    int i = distI(gen);
    for (TParticles::iterator p = newParticles.begin(); p != newParticles.end(); ++p)
    {
      beta += distB(gen);
      while(beta > mParticles[i].weight)
      {
        beta -= mParticles[i].weight;
        i = (i + 1) % mParticles.size();
      }
      *p = mParticles[i];
    }

    mParticles = newParticles;
  }
}

const bool ParticleFilter::isInitialized() const
{
  return mIsInitialized;
}

