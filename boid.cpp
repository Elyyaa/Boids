#include "boid.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace bd {

double distance(const sf::Vector2<double>& vec1,
                const sf::Vector2<double>& vec2) {
  double dX = vec2.x - vec1.x;
  double dY = vec2.y - vec1.y;
  return std::sqrt(dX * dX + dY * dY);
}

double magnitude(const sf::Vector2<double>& vec) {
  return std::sqrt(vec.x * vec.x + vec.y * vec.y);
}

double angle(const sf::Vector2<double>& v) { return std::atan2(v.y, v.x); }

Boid::Boid() : position(0, 0) {}
Boid::Boid(double pos_x, double pos_y) : position(pos_x, pos_y) {}

sf::Vector2<double> Boid::getPosition() const { return position; }
void Boid::setPosition(const sf::Vector2<double>& newPos) { position = newPos; }

sf::Vector2<double> Boid::getVelocity() const { return velocity; }
void Boid::setVelocity(const sf::Vector2<double>& newVel) { velocity = newVel; }

Parameters Boid::getPar() const { return par; }
void Boid::setPar(const Parameters& newPar) {
  par = newPar;
  assert(par.d >= 0.);
  if (par.d < 0.) {
    std::cout << "Something went wrong. Parameter d must be positive.\n";
    exit(EXIT_SUCCESS);
  }

  assert(par.ds >= 0. && par.ds < par.d);
  if (par.ds < 0. || par.ds >= par.d) {
    std::cout << "Something went wrong. Parameter ds must be positive and "
                 "smaller than d.\n";
    exit(EXIT_SUCCESS);
  }

  assert(par.s >= 0. && par.s <= 1.);
  if (par.s < 0. || par.s > 1.) {
    std::cout << "Something went wrong. Parameter s must be a number between 0 "
                 "and 1.\n";
    exit(EXIT_SUCCESS);
  }

  assert(par.a >= 0. && par.a <= 1.);
  if (par.a < 0. || par.a > 1.) {
    std::cout << "Something went wrong. Parameter a must be a number between 0 "
                 "and 1.\n";
    exit(EXIT_SUCCESS);
  }

  assert(par.c >= 0. && par.c <= 1.);
  if (par.c < 0. || par.c > 1.) {
    std::cout << "Something went wrong. Parameter c must be a number between 0 "
                 "and 1.\n";
    exit(EXIT_SUCCESS);
  }
}

void Boid::setPar_d(const double new_d) {
  par.d = new_d;
  assert(par.d >= 0.);
  if (par.d < 0.) {
    std::cout << "Something went wrong. Parameter d must be positive.\n";
    exit(EXIT_SUCCESS);
  }
}

void Boid::setPar_ds(const double new_ds) {
  par.ds = new_ds;
  assert(par.ds >= 0. && par.ds < par.d);
  if (par.ds < 0. || par.ds >= par.d) {
    std::cout << "Something went wrong. Parameter ds must be positive and "
                 "smaller than d.\n";
    exit(EXIT_SUCCESS);
  }
}

void Boid::setPar_s(const double new_s) {
  par.s = new_s;
  assert(par.s >= 0. && par.s <= 1.);
  if (par.s < 0. || par.s > 1.) {
    std::cout << "Something went wrong. Parameter s must be a number between 0 "
                 "and 1.\n";
    exit(EXIT_SUCCESS);
  }
}

void Boid::setPar_a(const double new_a) {
  par.a = new_a;
  assert(par.a >= 0. && par.a <= 1.);
  if (par.a < 0. || par.a > 1.) {
    std::cout << "Something went wrong. Parameter a must be a number between 0 "
                 "and 1.\n";
    exit(EXIT_SUCCESS);
  }
}

void Boid::setPar_c(const double new_c) {
  par.c = new_c;
  assert(par.c >= 0. && par.c <= 1.);
  if (par.c < 0. || par.c > 1.) {
    std::cout << "Something went wrong. Parameter c must be a number between 0 "
                 "and 1.\n";
    exit(EXIT_SUCCESS);
  }
}

double Boid::getMaxspeed() const { return maxspeed; }
void Boid::setMaxspeed(double new_Maxspeed) { maxspeed = new_Maxspeed; }

sf::Vector2<double> Boid::separation(const std::vector<Boid>& boids) {
  double ds = par.ds;
  double s = par.s;
  int N = boids.size();

  if (N < 2) {
    throw std::runtime_error{"Not enough boids"};
  }

  sf::Vector2<double> Displacements(0, 0);

  for (auto const& boid : boids) {
    const sf::Vector2<double>& otherPosition = boid.position;
    double distance1 = distance(position, otherPosition);
    if (distance1 < ds) {
      sf::Vector2<double> displacement = otherPosition - position;
      Displacements = Displacements + displacement;
    }
  }
  sf::Vector2<double> v1 = -s * Displacements;
  return v1;
}

sf::Vector2<double> Boid::alignment(const std::vector<Boid>& boids) {
  double a = par.a;
  double d = par.d;
  int N = boids.size();

  if (N < 2) {
    throw std::runtime_error{"Not enough boids"};
  }

  sf::Vector2<double> Velocities(0, 0);

  for (auto const& boid : boids) {
    double distance1 = distance(position, boid.position);
    if (distance1 < d) {
      sf::Vector2<double> speed = boid.velocity - velocity;
      Velocities = Velocities + speed;
    }
  }

  sf::Vector2<double> v2 = a * (1.0 / (N - 1)) * Velocities;
  return v2;
}

sf::Vector2<double> Boid::cohesion(const std::vector<Boid>& boids) {
  double c = par.c;
  double d = par.d;
  int N = boids.size();

  if (N < 2) {
    throw std::runtime_error{"Not enough boids"};
  }

  sf::Vector2<double> sum_pos(0, 0);
  sf::Vector2<double> v3(0, 0);

  for (auto const& boid : boids) {
    double distance1 = distance(position, boid.position);
    if (distance1 < d) {
      sf::Vector2<double> otherPosition = boid.position;
      sum_pos = sum_pos + otherPosition;
    }
  }
  sum_pos = sum_pos - position;
  sf::Vector2<double> xc = (1.0 / (N - 1)) * sum_pos;
  if (xc.x != 0 && xc.y != 0) {
    sf::Vector2<double> v3 = c * (xc - position);
    return v3;
  } else {
    return v3;
  }
}

void Boid::updateVelocity(const std::vector<Boid>& boids) {
  sf::Vector2<double> v1 = separation(boids);
  sf::Vector2<double> v2 = alignment(boids);
  sf::Vector2<double> v3 = cohesion(boids);

  velocity = velocity + v1 + v2 + v3;

  double mag_v = magnitude(velocity);

  if (mag_v > maxspeed) {
    velocity.x = (velocity.x / mag_v) * maxspeed;  // da vedere
    velocity.y = (velocity.y / mag_v) * maxspeed;
  };
}
void Boid::updatePosition(double const delta_t) {
  position = position + velocity * delta_t;
}

void Boid::borders() {
  double screenWidth{1280};
  double screenHeight{720};
  // passati per copy perchè altrimenti warning: "narrowing conversion from
  // unsigned int to double"

  if (position.x < 0.) {
    position.x = screenWidth;
  } else if (position.x > screenWidth) {
    position.x = 0;
  }
  if (position.y < 0.) {
    position.y =
        screenHeight;  // lo schermo va al contrario quindi bordo superiore
  } else if (position.y > screenHeight) {
    position.y = 0;
  }
}

void Boid::update(const std::vector<Boid>& boids, double const delta_t) {
  updateVelocity(boids);
  updatePosition(delta_t);
  borders();
}

}  // namespace bd