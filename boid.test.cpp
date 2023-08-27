#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "boid.hpp"

#include "doctest.h"
#include "flock.hpp"

TEST_CASE("Testing the vectors functions") {
  SUBCASE("Distance between vectors") {
    sf::Vector2<double> v1{1, 1};
    sf::Vector2<double> v2{4, 5};
    double distance12 = bd::distance(v1, v2);
    CHECK(distance12 == doctest::Approx(5));
  }
  SUBCASE("Magnitude of a vector") {
    sf::Vector2<double> v1{4, 3};
    double magnitude1 = bd::magnitude(v1);
    CHECK(magnitude1 == doctest::Approx(5));
  }
  SUBCASE("Angle of a vector") {
    sf::Vector2<double> v1{4, 3};
    double angle1 = bd::angle(v1);
    CHECK(angle1 == doctest::Approx(0.64).epsilon(0.01));
  }
}

TEST_CASE("Testing the class Boid") {
  SUBCASE("getPosition, setPosition, getVelocity, setVelocity") {
    bd::Boid boid1;
    sf::Vector2<double> pos1{0, 1};
    boid1.setPosition(pos1);
    sf::Vector2<double> p1 = boid1.getPosition();

    sf::Vector2<double> vel1{2, 0};
    boid1.setVelocity(vel1);
    sf::Vector2<double> v1 = boid1.getVelocity();

    CHECK(p1.x == doctest::Approx(0.0));
    CHECK(p1.y == doctest::Approx(1.0));
    CHECK(v1.x == doctest::Approx(2.0));
    CHECK(v1.y == doctest::Approx(0.0));
  }

  SUBCASE("Setting and getting Parameters") {
    bd::Boid boid1;
    bd::Parameters par1{7.0, 5.0, 1.0, 0.3, 0.5};
    boid1.setPar(par1);
    bd::Parameters getPar1 = boid1.getPar();
    CHECK(getPar1.d == doctest::Approx(7.0));
    CHECK(getPar1.ds == doctest::Approx(5.0));
    CHECK(getPar1.s == doctest::Approx(1.0));
    CHECK(getPar1.a == doctest::Approx(0.3));
    CHECK(getPar1.c == doctest::Approx(0.5));

    boid1.setPar_d(10.0);
    boid1.setPar_ds(6.0);
    boid1.setPar_s(0.5);
    boid1.setPar_a(1.0);
    boid1.setPar_c(0.2);

    bd::Parameters getPar2 = boid1.getPar();

    CHECK(getPar2.d == doctest::Approx(10.0));
    CHECK(getPar2.ds == doctest::Approx(6.0));
    CHECK(getPar2.s == doctest::Approx(0.5));
    CHECK(getPar2.a == doctest::Approx(1.0));
    CHECK(getPar2.c == doctest::Approx(0.2));
  }

  SUBCASE("Separation, Alignment and Cohesion rules, and updateVelocity") {
    bd::Boid boid1;
    sf::Vector2<double> pos1{0, 0};
    boid1.setPosition(pos1);
    sf::Vector2<double> vel1{0, 0};
    boid1.setVelocity(vel1);

    bd::Boid boid2;
    sf::Vector2<double> pos2{3, 4};
    boid2.setPosition(pos2);
    sf::Vector2<double> vel2{0, 4};
    boid2.setVelocity(vel2);

    bd::Boid testBoid;
    sf::Vector2<double> pos_test{1, 1};
    testBoid.setPosition(pos_test);
    sf::Vector2<double> vel_test{1, 1};
    testBoid.setVelocity(vel_test);

    double ds = 5;
    double s = 1;
    double c = 1;
    double a = 1;
    double d = 7;
    double maxspeed = 100;

    testBoid.setPar_s(s);
    testBoid.setPar_c(c);
    testBoid.setPar_a(a);
    testBoid.setPar_d(d);
    testBoid.setPar_ds(ds);
    testBoid.setMaxspeed(maxspeed);

    std::vector<bd::Boid> boids = {boid1, boid2, testBoid};
    sf::Vector2<double> v1 = testBoid.separation(boids);
    sf::Vector2<double> v2 = testBoid.alignment(boids);
    sf::Vector2<double> v3 = testBoid.cohesion(boids);

    testBoid.updateVelocity(boids);
    sf::Vector2<double> v = testBoid.getVelocity();

    CHECK(v1.x == doctest::Approx(-1.0));
    CHECK(v1.y == doctest::Approx(-2.0));
    CHECK(v2.x == doctest::Approx(-1.0));
    CHECK(v2.y == doctest::Approx(1.0));
    CHECK(v3.x == doctest::Approx(0.5));
    CHECK(v3.y == doctest::Approx(1.0));
    CHECK(v.x == doctest::Approx(-0.5));
    CHECK(v.y == doctest::Approx(1.0));
  }

  SUBCASE("Calling separation, alignment, cohesion with one point throws") {
    bd::Boid boid1;
    std::vector<bd::Boid> boids = {boid1};

    CHECK_THROWS(boid1.separation(boids));
    CHECK_THROWS(boid1.alignment(boids));
    CHECK_THROWS(boid1.cohesion(boids));
  }

  SUBCASE("updatePosition") {
    bd::Boid boid1;
    sf::Vector2<double> pos1{0, 0};
    boid1.setPosition(pos1);
    sf::Vector2<double> vel1{1, 1};
    boid1.setVelocity(vel1);
    double const delta_t = 1;
    boid1.updatePosition(delta_t);
    pos1 = boid1.getPosition();
    CHECK(pos1.x == doctest::Approx(1.0));
    CHECK(pos1.y == doctest::Approx(1.0));
  }
  SUBCASE("borders: the boid exceeds the superior border") {
    double screenHeight{720};
    bd::Boid boid1;
    sf::Vector2<double> pos1{5., -1.};
    boid1.setPosition(pos1);
    sf::Vector2<double> vel1{0, -1};
    boid1.setVelocity(vel1);
    boid1.borders();
    pos1 = boid1.getPosition();
    CHECK(pos1.x == doctest::Approx(5.0));
    CHECK(pos1.y == doctest::Approx(screenHeight));
  }
  SUBCASE("borders: the boid exceeds the inferior border") {
    double screenHeight{720};
    bd::Boid boid1;
    sf::Vector2<double> pos1{5, screenHeight + 1};
    boid1.setPosition(pos1);
    sf::Vector2<double> vel1{0, 1};
    boid1.setVelocity(vel1);
    boid1.borders();
    pos1 = boid1.getPosition();
    CHECK(pos1.x == doctest::Approx(5.0));
    CHECK(pos1.y == doctest::Approx(0.0));
  }

  SUBCASE("borders: the boid exceeds the right border") {
    double screenWidth{1280};
    bd::Boid boid1;
    sf::Vector2<double> pos1{screenWidth + 1, 5};
    boid1.setPosition(pos1);
    sf::Vector2<double> vel1{1, 0};
    boid1.setVelocity(vel1);
    boid1.borders();
    pos1 = boid1.getPosition();
    CHECK(pos1.x == doctest::Approx(0.0));
    CHECK(pos1.y == doctest::Approx(5.0));
  }
  SUBCASE("borders: the boid exceeds the left border") {
    double screenWidth{1280};
    bd::Boid boid1;
    sf::Vector2<double> pos1{-1., 5};
    boid1.setPosition(pos1);
    sf::Vector2<double> vel1{-1, 0};
    boid1.setVelocity(vel1);
    boid1.borders();
    pos1 = boid1.getPosition();
    CHECK(pos1.x == doctest::Approx(screenWidth));
    CHECK(pos1.y == doctest::Approx(5.0));
  }

  SUBCASE("update") {
    bd::Boid boid1;
    sf::Vector2<double> pos1{1, 1};
    boid1.setPosition(pos1);
    sf::Vector2<double> vel1{1, 0};
    boid1.setVelocity(vel1);

    bd::Boid boid2;
    sf::Vector2<double> pos2{1, 4};
    boid2.setPosition(pos2);
    sf::Vector2<double> vel2{0, 2};
    boid2.setVelocity(vel2);

    double ds = 5;
    double s = 1;
    double c = 1;
    double a = 1;
    double d = 7;
    double maxspeed = 100;

    boid2.setPar_s(s);
    boid2.setPar_c(c);
    boid2.setPar_a(a);
    boid2.setPar_d(d);
    boid2.setPar_ds(ds);
    boid2.setMaxspeed(maxspeed);

    std::vector<bd::Boid> boids = {boid1, boid2};
    double const delta_t = 1;

    boid2.update(boids, delta_t);
    sf::Vector2<double> v = boid2.getVelocity();
    sf::Vector2<double> p = boid2.getPosition();

    CHECK(v.x == doctest::Approx(1.0));
    CHECK(v.y == doctest::Approx(0.0));
    CHECK(p.x == doctest::Approx(2.0));
    CHECK(p.y == doctest::Approx(4.0));
  }

  SUBCASE("Testing getMaxspeed and the the Maxspeed in updateVelocity") {
    bd::Boid boid1;
    bd::Boid boid2;
    sf::Vector2<double> vel1{5, 12};
    sf::Vector2<double> vel2{5, 12};
    boid1.setVelocity(vel1);
    boid2.setVelocity(vel2);
    boid1.setMaxspeed(10);
    boid2.setMaxspeed(10);
    double m_s = boid1.getMaxspeed();

    std::vector<bd::Boid> boids = {boid1, boid2};
    boid1.updateVelocity(boids);
    double speed = bd::magnitude(boid1.getVelocity());

    CHECK(m_s == doctest::Approx(10));
    CHECK(speed == doctest::Approx(10));
  }
}
TEST_CASE("Testing the Flock class") {
  SUBCASE("Testing the average_distance and average_speed") {
    bd::Flock test_flock;
    bd::Boid boid1(5, 6);
    bd::Boid boid2(1, 3);
    bd::Boid boid3(2, 2);

    sf::Vector2<double> vel1{4, 3};
    boid1.setVelocity(vel1);
    sf::Vector2<double> vel2{3, 4};
    boid2.setVelocity(vel2);
    sf::Vector2<double> vel3{-1, 0};
    boid3.setVelocity(vel3);

    test_flock.addBoid(boid1);
    test_flock.addBoid(boid2);
    test_flock.addBoid(boid3);

    double average_s = test_flock.average_speed().mean;
    double average_d = test_flock.average_distance().mean;
    double sigma_s = test_flock.average_speed().sigma;
    double sigma_d = test_flock.average_distance().sigma;

    CHECK(average_s == doctest::Approx(3.7).epsilon(0.1));
    CHECK(average_d == doctest::Approx(3.8).epsilon(0.1));
    CHECK(sigma_s == doctest::Approx(2.3).epsilon(0.1));
    CHECK(sigma_d == doctest::Approx(2.1).epsilon(0.1));
  }

  SUBCASE("Testing the updateFlock method") {
    bd::Boid boid1(1, 1);
    sf::Vector2<double> vel1{1, 0};
    boid1.setVelocity(vel1);

    bd::Boid boid2(1, 2);
    sf::Vector2<double> vel2{0, 2};
    boid2.setVelocity(vel2);

    bd::Parameters par1{10.0, 7.0, 1.0, 1.0, 1.0};
    boid1.setPar(par1);
    boid2.setPar(par1);

    double maxspeed = 100;
    boid1.setMaxspeed(maxspeed);
    boid2.setMaxspeed(maxspeed);

    bd::Flock test_flock;
    test_flock.addBoid(boid1);
    test_flock.addBoid(boid2);

    double const delta_t = 1;

    test_flock.updateFlock(delta_t);

    sf::Vector2<double> v1 = test_flock.getBoid(0).getVelocity();
    sf::Vector2<double> p1 = test_flock.getBoid(0).getPosition();
    sf::Vector2<double> v2 = test_flock.getBoid(1).getVelocity();
    sf::Vector2<double> p2 = test_flock.getBoid(1).getPosition();

    CHECK(v1.x == doctest::Approx(0.0));
    CHECK(v1.y == doctest::Approx(2.0));
    CHECK(p1.x == doctest::Approx(1.0));
    CHECK(p1.y == doctest::Approx(3.0));

    CHECK(v2.x == doctest::Approx(0.0));
    CHECK(v2.y == doctest::Approx(2.0));
    CHECK(p2.x == doctest::Approx(1.0));
    CHECK(p2.y == doctest::Approx(4.0));
  }

  SUBCASE("setParameters") {
    bd::Flock test_flock;
    bd::Boid boid1(5, 6);
    bd::Boid boid2(1, 3);
    test_flock.addBoid(boid1);
    test_flock.addBoid(boid2);

    bd::Parameters par1{10, 7, 1, 1, 1};
    test_flock.setParameters(par1);

    bd::Parameters gPar = test_flock.getBoid(0).getPar();
    bd::Parameters gPar2 = test_flock.getBoid(1).getPar();

    CHECK(gPar.d == doctest::Approx(10.0));
    CHECK(gPar.ds == doctest::Approx(7.0));
    CHECK(gPar.s == doctest::Approx(1.0));
    CHECK(gPar.a == doctest::Approx(1.0));
    CHECK(gPar.c == doctest::Approx(1.0));

    CHECK(gPar2.d == doctest::Approx(10.0));
    CHECK(gPar2.ds == doctest::Approx(7.0));
    CHECK(gPar2.s == doctest::Approx(1.0));
    CHECK(gPar2.a == doctest::Approx(1.0));
    CHECK(gPar2.c == doctest::Approx(1.0));
  }

  SUBCASE("Testing getBoid ") {
    bd::Boid b1;
    bd::Boid b2(2, 3);

    bd::Flock f1;
    f1.addBoid(b1);
    f1.addBoid(b2);

    bd::Boid test_b1 = f1.getBoid(0);
    bd::Boid test_b2 = f1.getBoid(1);

    sf::Vector2<double> p1 = test_b1.getPosition();
    sf::Vector2<double> p2 = test_b2.getPosition();

    CHECK(p1.x == doctest::Approx(0));
    CHECK(p1.y == doctest::Approx(0));
    CHECK(p2.x == doctest::Approx(2));
    CHECK(p2.y == doctest::Approx(3));
  }
}