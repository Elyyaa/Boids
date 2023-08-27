#pragma once
#ifndef BOID_HPP
#define BOID_HPP

#include <SFML/System/Vector2.hpp>
#include "SFML/Window/VideoMode.hpp"
#include <vector>

namespace bd {

double distance(const sf::Vector2<double>& vec1,
                const sf::Vector2<double>& vec2);

double magnitude(const sf::Vector2<double>& vec);

double angle(const sf::Vector2<double>& v);

struct Parameters {
  double d{};
  double ds{};
  double s{};
  double a{};
  double c{};
};

class Boid {
  sf::Vector2<double> position;
  sf::Vector2<double> velocity;
  Parameters par;
  double maxspeed;

 public:
  Boid();
  Boid(double, double);

  sf::Vector2<double> getPosition() const;
  void setPosition(const sf::Vector2<double>& newPos);

  sf::Vector2<double> getVelocity() const;
  void setVelocity(const sf::Vector2<double>& newVel);

  Parameters getPar() const;
  void setPar(const Parameters& newPar);

  void setPar_d(const double new_d);
  void setPar_ds(const double new_ds);
  void setPar_s(const double new_s);
  void setPar_a(const double new_a);
  void setPar_c(const double new_c);

  double getMaxspeed() const;
  void setMaxspeed(double new_Maxspeed);

  sf::Vector2<double> separation(const std::vector<Boid>& boids);
  sf::Vector2<double> alignment(const std::vector<Boid>& boids);
  sf::Vector2<double> cohesion(const std::vector<Boid>& boids);

  void updateVelocity(const std::vector<Boid>& boids);
  void updatePosition(double const delta_t);
  void borders();

  void update(const std::vector<Boid>& boids, double const delta_t);

};

}  // namespace bd
#endif