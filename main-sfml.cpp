#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Window.hpp>
#include <cassert>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

#include "boid.hpp"
#include "flock.hpp"

void ignoreLine() {
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

int main() {
  try {
    int N{};  // number of boids
    char cmd;
    double rotation{};

    bd::Flock flock1;
    int screenWidth{1280};
    int screenHeight{720};
    const float triangleSide{4};

    std::random_device r;
    std::default_random_engine eng(r());
    std::uniform_real_distribution<double> xDist(0, screenWidth);
    std::uniform_real_distribution<double> yDist(0, screenHeight);
    std::uniform_real_distribution<double> vxDist(-1, 1);
    std::uniform_real_distribution<double> vyDist(-1, 1);

    bd::Parameters params;

    std::cout << "Valid commands:\n"
              << "[g] to generate a flock\n"
              << "[b] to view the boids\n"
              << "[q] to quit.\n";

    while (std::cin >> cmd) {
      switch (cmd) {
        case 'g': {
          std::cout
              << "Please input your data. \nFirst enter the number of boids: ";
          std::cin >> N;
          ignoreLine();
          assert(N >= 2);
          if (N < 2) {
            std::cout << "Not enough data. Try generating a bigger flock.\n";
            exit(EXIT_SUCCESS);
          }

          std::cout << "Enter separation parameter s: ";
          std::cin >> params.s;
          ignoreLine();

          std::cout << "Enter alignment parameter a: ";
          std::cin >> params.a;
          ignoreLine();

          std::cout << "Enter cohesion parameter c: ";
          std::cin >> params.c;
          ignoreLine();

          std::cout << "Enter distance d and range influence parameter ds: ";
          std::cin >> params.d >> params.ds;

          if (std::cin.fail()) {
            std::cin.clear();
            ignoreLine();
          }

          // Initialize boids
          for (int i = 0; i < N; ++i) {
            double x = xDist(eng);
            double y = yDist(eng);
            double vx = vxDist(eng);
            double vy = vyDist(eng);
            bd::Boid boid(x, y);
            boid.setVelocity({vx, vy});
            boid.setMaxspeed(500);

            boid.setPar(params);

            flock1.addBoid(boid);  // or: push_back, che è dinamico
          }
          std::cout << "Data generated successfully.\n";

          /*for (bd::Boid& boid : flock1.flock()) {
            std::cout << "x " << boid.getPosition().x << "\n";
            std::cout << "y " << boid.getPosition().y << "\n";
            std::cout << "vx " << boid.getVelocity().x << "\n";
            std::cout << "vy " << boid.getVelocity().y << "\n";
          }*/

          break;
        }

        case 'b': {
          assert(N != 0);
          if (N == 0) {
            std::cout << "Not enough data. Try generating a flock with [g].\n";
            exit(EXIT_SUCCESS);
          }

          sf::RenderWindow window(sf::VideoMode(screenWidth, screenHeight),
                                  "Boids");
          window.setFramerateLimit(60);

          sf::Clock clock;

          while (window.isOpen()) {
            sf::Event event;
            sf::Time elapsed = clock.restart();
            double delta_t =
                elapsed.asSeconds();  // to make boid movement independent from
                                      // frame rate
            while (window.pollEvent(event)) {
              if ((event.type ==
                   sf::Event::Closed) ||  // press X button to quit
                  (sf::Keyboard::isKeyPressed(
                      sf::Keyboard::Escape)))  // press esc to quit
              {
                window.close();
              }
            }
            //flock1.updateFlock(delta_t);
            /*for (bd::Boid& boid : flock1.flock()) {
            sf::Vector2<double>  v1 = boid.separation(flock1.flock());
            sf::Vector2<double>  v2 = boid.alignment(flock1.flock());
            sf::Vector2<double>  v3 = boid.cohesion(flock1.flock());

            std::cout << "v1 " << v1.x << "\n";
            std::cout << "v1 " << v1.y << "\n";
            std::cout << "v2 " << v2.x << "\n";
            std::cout << "v2 " << v2.y << "\n";
            std::cout << "v3 " << v3.x << "\n";
            std::cout << "v3 " << v3.y << "\n";
          }*/

          flock1.updateFlock(delta_t);

          /*for (bd::Boid& boid : flock1.flock()) {

            std::cout << "x " << boid.getPosition().x << "\n";
            std::cout << "y " << boid.getPosition().y << "\n";
            std::cout << "vx " << boid.getVelocity().x << "\n";
            std::cout << "vy " << boid.getVelocity().y << "\n";
          }*/

            window.clear();

            

            // Draw boids
            for (const bd::Boid& boid : flock1.flock()) {
              sf::ConvexShape shape;
              shape.setPosition(boid.getPosition().x, boid.getPosition().y);
              shape.setPointCount(3);
              shape.setPoint(0, sf::Vector2f(-triangleSide, triangleSide));
              shape.setPoint(1, sf::Vector2f(triangleSide, triangleSide));
              shape.setPoint(2, sf::Vector2f(0, 5 * triangleSide));
              shape.setFillColor(sf::Color::White);
              rotation = bd::angle(boid.getVelocity());
              shape.setRotation(270 +(rotation * 180 )/
                                M_PI);  // conversione da radianti a gradi

              window.draw(shape);
            }

            window.display();
          }
          break;
        }
        case 'q': {
          return EXIT_SUCCESS;
          break;
        }
        default: {
          std::cout << "Bad format, insert a new command\n";
          std::cin.clear();
          ignoreLine();
        }
      }
    }
  } catch (std::exception const& e) {
    std::cerr << "An exception occurred: " << e.what() << "'\n";
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Caught unknown exception\n";
    return EXIT_FAILURE;
  }
}
