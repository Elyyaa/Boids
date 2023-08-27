#include "flock.hpp"
#include <cassert>
#include <cmath>
//#include <fstream>
#include <iostream>
//#include <limits>
#include <random>
#include <iomanip>

namespace bd {
void Flock::addBoid(const Boid& b) { m_flock.push_back(b); }

Boid Flock::getBoid(int i) const { return m_flock[i]; }

Boid& Flock::getBoid(int i) { return m_flock[i]; }

void Flock::updateFlock(const double delta_t) { 
  for (auto& boid : m_flock) {
    boid.update(m_flock, delta_t);
  }
}

Statistics Flock::average_distance() {
  int N = (*this).size();
  double sum_d = 0.0;
  double sum_d2 = 0.0;
  int pair_count = 0;
  assert(N >= 2); 

  for (int i = 0; i < N; i++) {
    for (int j = i + 1; j < N; j++) {
      const sf::Vector2<double>& pos1 = (*this).getBoid(i).getPosition();
      const sf::Vector2<double>& pos2 = (*this).getBoid(j).getPosition();

      double distance1 = bd::distance(pos1, pos2);
      sum_d += distance1;
      sum_d2 += pow(distance1, 2);
      pair_count++;
    }
  }
  
  if (pair_count < 2) {
    throw std::runtime_error{"Not enough entries to run a statistics"};
  }

  double average_distance = sum_d / pair_count;
  const double sigma_d =
      std::sqrt((sum_d2 - pair_count * average_distance * average_distance) /
                (pair_count - 1));

  return {average_distance, sigma_d};
}

Statistics Flock::average_speed() {
  int N = (*this).size();
  double sum_v = 0.0;
  double sum_v2 = 0.0;
  assert(N >= 2); 

  for (auto const& boid : m_flock) {
    const sf::Vector2<double>& v = boid.getVelocity();

    double speed1 = bd::magnitude(v);
    sum_v += speed1;
    sum_v2 += speed1 * speed1;
  }

  if (N < 2) {
    throw std::runtime_error{"Not enough entries to run a statistics"};
  }

  double average_speed = sum_v / N;

  const double sigma_v =
      std::sqrt((sum_v2 - N * average_speed * average_speed) / (N - 1));

  return {average_speed, sigma_v};
}

void Flock::setParameters(const Parameters& par1) { 
  for (auto& boid : m_flock) {
    boid.setPar(par1); //assert statements included in call to setPar
  };
}

void histogram(std::vector<double> entries, std::vector<double> errors,  double norm) {
  assert(entries.size() >= 1);
  if (entries.size() < 1) {
    throw std::runtime_error{"Not enough entries to draw a histogram"};
  }

  for (int i = 0, N = entries.size(); i < N ; ++i) {

    std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(0) //fixed-point notation
              << std::noshowpos << entries[i] <<"+-"<<errors[i]<< " |"; //no + for positive entries

    std::string row(std::round((entries[i]-errors[i])/norm), '-');
    std::string row2(std::round(errors[i]/norm),'-'); //rounding double to int

    std::cout << row << "σ"<<row2<<"*"<<row2<<"σ\n"; //
    
  }
}
} 