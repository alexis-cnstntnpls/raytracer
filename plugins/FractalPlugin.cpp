#include "FractalPlugin.hpp"
#include "Parser/SceneParser.hpp"
#include <cmath>
#include <iostream>

namespace Raytracer::Plugins {

std::unique_ptr<Plugin::PrimitivePlugin> FractalPlugin::create() {
  return std::make_unique<FractalPlugin>();
}

bool FractalPlugin::configure(const libconfig::Setting &config) {
  try {
    auto pos = Parser::SceneParser::parsePoint3(config.lookup("position"));
    if (pos)
      m_position = *pos;
    config.lookupValue("iterations", m_iterations);
    config.lookupValue("bailout", m_bailout);
    config.lookupValue("power", m_power);

    Parser::SceneParser::applyTransformations(config, this);

  } catch (const std::exception &e) {
    return false;
  }
  return true;
}

double FractalPlugin::mandelbulbDistanceEstimator(Math::Point<3> p) const {
  double scaleFactor = 0.05;

  Math::Vector<3> z = p - m_position;

  z = z * scaleFactor;

  double dr = 1.0;
  double r = 0.0;
  Math::Vector<3> c = z;

  for (int i = 0; i < m_iterations; i++) {
    r = sqrt(z.m_components[0] * z.m_components[0] +
             z.m_components[1] * z.m_components[1] +
             z.m_components[2] * z.m_components[2]);

    if (r > m_bailout)
      break;

    double theta = acos(z.m_components[2] / r);
    double phi = atan2(z.m_components[1], z.m_components[0]);

    dr = pow(r, m_power - 1.0) * m_power * dr + 1.0;

    double zr = pow(r, m_power);
    theta = theta * m_power;
    phi = phi * m_power;

    z.m_components[0] = zr * sin(theta) * cos(phi);
    z.m_components[1] = zr * sin(theta) * sin(phi);
    z.m_components[2] = zr * cos(theta);

    z = z + c;
  }

  return 0.25 * log(r) * r / dr / scaleFactor;
}

std::optional<Core::Intersection>
FractalPlugin::intersect(const Core::Ray &ray) const noexcept {
  try {
    Math::Point<3> origin = ray.getOrigin();
    Math::Vector<3> direction = ray.getDirection().normalize();

    double totalDistance = ray.getMinDistance();
    double epsilon = 0.01;
    int maxSteps = 256;
    double maxDistance = 1000.0;

    const double safetyFactor = 0.5;
    for (int i = 0; i < maxSteps; i++) {
      Math::Point<3> pos(
          origin.m_components[0] + direction.m_components[0] * totalDistance,
          origin.m_components[1] + direction.m_components[1] * totalDistance,
          origin.m_components[2] + direction.m_components[2] * totalDistance);

      double distance = mandelbulbDistanceEstimator(pos);

      if (distance < epsilon) {
        const double delta = epsilon * 2.0;
        Math::Vector<3> normal(
            mandelbulbDistanceEstimator({pos.m_components[0] + delta,
                                         pos.m_components[1],
                                         pos.m_components[2]}) -
                mandelbulbDistanceEstimator({pos.m_components[0] - delta,
                                             pos.m_components[1],
                                             pos.m_components[2]}),

            mandelbulbDistanceEstimator({pos.m_components[0],
                                         pos.m_components[1] + delta,
                                         pos.m_components[2]}) -
                mandelbulbDistanceEstimator({pos.m_components[0],
                                             pos.m_components[1] - delta,
                                             pos.m_components[2]}),

            mandelbulbDistanceEstimator({pos.m_components[0],
                                         pos.m_components[1],
                                         pos.m_components[2] + delta}) -
                mandelbulbDistanceEstimator({pos.m_components[0],
                                             pos.m_components[1],
                                             pos.m_components[2] - delta}));

        double normalLength = normal.length();

        if (normalLength > 0) {
          normal = normal / normalLength;
        } else {
          normal.m_components[0] = 0;
          normal.m_components[1] = 0;
          normal.m_components[2] = 1;
        }

        return Core::Intersection(pos, normal, getMaterial(), totalDistance,
                                  false, {0.0, 0.0});
      }

      totalDistance += distance * safetyFactor;

      if (totalDistance > maxDistance || totalDistance > ray.getMaxDistance()) {
        break;
      }
    }

    return std::nullopt;

  } catch (const std::exception &e) {
    return std::nullopt;
  }
}

Core::BoundingBox FractalPlugin::getBoundingBox() const noexcept {
  double size = 200.0;
  return {Math::Point<3>{m_position.m_components[0] - size,
                                          m_position.m_components[1] - size,
                                          m_position.m_components[2] - size},
                           Math::Point<3>{m_position.m_components[0] + size,
                                          m_position.m_components[1] + size,
                                          m_position.m_components[2] + size}};
}

} // namespace Raytracer::Plugins

extern "C" {
Raytracer::Plugin::IPlugin *createPlugin() {
  return new Raytracer::Plugins::FractalPlugin();
}

void destroyPlugin(Raytracer::Plugin::IPlugin *plugin) { delete plugin; }
}