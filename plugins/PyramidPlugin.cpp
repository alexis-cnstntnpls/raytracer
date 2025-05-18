#include "PyramidPlugin.hpp"
#include "Parser/SceneParser.hpp"
#include <algorithm>
#include <cmath>

namespace Raytracer::Plugins {

std::unique_ptr<Plugin::PrimitivePlugin> PyramidPlugin::create() {
  return std::make_unique<PyramidPlugin>();
}

bool PyramidPlugin::configure(const libconfig::Setting &config) {
  try {
    auto baseCenter = Parser::SceneParser::parsePoint3(config.lookup("position"));
    double baseSize = config.lookup("baseSize");
    double height = config.lookup("height");
    std::string upAxis = config.lookup("upAxis");

    if (baseCenter) {
      this->setParameters(*baseCenter, baseSize, height, upAxis);

      Parser::SceneParser::applyTransformations(config, this);
    }
  } catch (const libconfig::SettingNotFoundException &) {
    return false;
  }
  return true;
}

void PyramidPlugin::setParameters(const Math::Point<3> &baseCenter, double baseSize,
                                 double height, const std::string &upAxis) noexcept {
  m_baseCenter = baseCenter;
  m_baseSize = baseSize;
  m_height = height;

  if (upAxis == "X") {
    m_upAxis = Math::Vector<3>{1.0, 0.0, 0.0};
  } else if (upAxis == "Y") {
    m_upAxis = Math::Vector<3>{0.0, 1.0, 0.0};
  } else {
    m_upAxis = Math::Vector<3>{0.0, 0.0, 1.0};
  }

  m_apex = m_baseCenter + m_upAxis * m_height;

  int axisIndex = -1;
  if (m_upAxis.m_components[0] == 1.0) {
    axisIndex = 0;
  } else if (m_upAxis.m_components[1] == 1.0) {
    axisIndex = 1;
  } else {
    axisIndex = 2;
  }

  double halfSize = m_baseSize / 2.0;
  if (axisIndex == 0) {
    m_baseVertices[0] = Math::Point<3>{m_baseCenter.m_components[0], m_baseCenter.m_components[1] - halfSize, m_baseCenter.m_components[2] - halfSize};
    m_baseVertices[1] = Math::Point<3>{m_baseCenter.m_components[0], m_baseCenter.m_components[1] + halfSize, m_baseCenter.m_components[2] - halfSize};
    m_baseVertices[2] = Math::Point<3>{m_baseCenter.m_components[0], m_baseCenter.m_components[1] + halfSize, m_baseCenter.m_components[2] + halfSize};
    m_baseVertices[3] = Math::Point<3>{m_baseCenter.m_components[0], m_baseCenter.m_components[1] - halfSize, m_baseCenter.m_components[2] + halfSize};
  } else if (axisIndex == 1) {
    m_baseVertices[0] = Math::Point<3>{m_baseCenter.m_components[0] - halfSize, m_baseCenter.m_components[1], m_baseCenter.m_components[2] - halfSize};
    m_baseVertices[1] = Math::Point<3>{m_baseCenter.m_components[0] + halfSize, m_baseCenter.m_components[1], m_baseCenter.m_components[2] - halfSize};
    m_baseVertices[2] = Math::Point<3>{m_baseCenter.m_components[0] + halfSize, m_baseCenter.m_components[1], m_baseCenter.m_components[2] + halfSize};
    m_baseVertices[3] = Math::Point<3>{m_baseCenter.m_components[0] - halfSize, m_baseCenter.m_components[1], m_baseCenter.m_components[2] + halfSize};
  } else {
    m_baseVertices[0] = Math::Point<3>{m_baseCenter.m_components[0] - halfSize, m_baseCenter.m_components[1] - halfSize, m_baseCenter.m_components[2]};
    m_baseVertices[1] = Math::Point<3>{m_baseCenter.m_components[0] + halfSize, m_baseCenter.m_components[1] - halfSize, m_baseCenter.m_components[2]};
    m_baseVertices[2] = Math::Point<3>{m_baseCenter.m_components[0] + halfSize, m_baseCenter.m_components[1] + halfSize, m_baseCenter.m_components[2]};
    m_baseVertices[3] = Math::Point<3>{m_baseCenter.m_components[0] - halfSize, m_baseCenter.m_components[1] + halfSize, m_baseCenter.m_components[2]};
  }
}

std::optional<std::pair<double, Math::Vector<3>>>
PyramidPlugin::intersectTriangle(const Core::Ray &ray, const Math::Point<3> &v0,
                                const Math::Point<3> &v1, const Math::Point<3> &v2) const noexcept {
  const double EPSILON = 1e-6;
  Math::Vector<3> edge1 = v1 - v0;
  Math::Vector<3> edge2 = v2 - v0;
  Math::Vector<3> h = ray.getDirection().cross(edge2);
  double a = edge1.dot(h);

  if (a > -EPSILON && a < EPSILON) {
    return std::nullopt;
  }

  double f = 1.0 / a;
  Math::Vector<3> s = ray.getOrigin() - v0;
  double u = f * s.dot(h);
  if (u < 0.0 || u > 1.0) {
    return std::nullopt;
  }

  Math::Vector<3> q = s.cross(edge1);
  double v = f * ray.getDirection().dot(q);
  if (v < 0.0 || u + v > 1.0) {
    return std::nullopt;
  }

  double t = f * edge2.dot(q);
  if (t < ray.getMinDistance() || t > ray.getMaxDistance()) {
    return std::nullopt;
  }

  Math::Vector<3> normal = edge1.cross(edge2);
  normal = normal.normalize();

  return std::make_pair(t, normal);
}

std::optional<Core::Intersection>
PyramidPlugin::intersect(const Core::Ray &ray) const noexcept {
  Core::Ray localRay = getTransform().inverseTransformRay(ray);

  double closestT = std::numeric_limits<double>::infinity();
  Math::Vector<3> closestNormal;
  Math::Point<2> closestUV{0.0, 0.0};
  bool isInside = false;

  for (size_t i = 0; i < 4; i++) {
    const Math::Point<3> &v0 = m_apex;
    const Math::Point<3> &v1 = m_baseVertices[i];
    const Math::Point<3> &v2 = m_baseVertices[(i + 1) % 4];

    auto hit = intersectTriangle(localRay, v0, v1, v2);
    if (hit && hit->first < closestT) {
      closestT = hit->first;
      closestNormal = hit->second;

      if (closestNormal.dot(localRay.getDirection()) > 0) {
        closestNormal = closestNormal * -1.0;
        isInside = true;
      } else {
        isInside = false;
      }

      closestUV = Math::Point<2>{static_cast<double>(i) / 4.0, 0.5};
    }
  }

  auto hit1 = intersectTriangle(localRay, m_baseVertices[0], m_baseVertices[1], m_baseVertices[2]);
  auto hit2 = intersectTriangle(localRay, m_baseVertices[0], m_baseVertices[2], m_baseVertices[3]);

  if (hit1 && hit1->first < closestT) {
    closestT = hit1->first;
    closestNormal = hit1->second;

    if (closestNormal.dot(localRay.getDirection()) > 0) {
      closestNormal = closestNormal * -1.0;
      isInside = true;
    } else {
      isInside = false;
    }

    closestUV = Math::Point<2>{0.5, 0.0};
  }

  if (hit2 && hit2->first < closestT) {
    closestT = hit2->first;
    closestNormal = hit2->second;

    if (closestNormal.dot(localRay.getDirection()) > 0) {
      closestNormal = closestNormal * -1.0;
      isInside = true;
    } else {
      isInside = false;
    }

    closestUV = Math::Point<2>{0.5, 0.0};
  }

  if (closestT < std::numeric_limits<double>::infinity()) {
    Math::Point<3> intersectionPoint = localRay.at(closestT);
    Math::Point<3> worldPoint = getTransform().transformPoint(intersectionPoint);
    Math::Vector<3> worldNormal = getTransform().transformNormal(closestNormal);
    worldNormal = worldNormal.normalize();
    double worldDist = (worldPoint - ray.getOrigin()).length();

    return Core::Intersection(worldPoint, worldNormal, getMaterial(), worldDist, isInside, closestUV);
  }

  return std::nullopt;
}

Core::BoundingBox PyramidPlugin::getBoundingBox() const noexcept {
  Math::Point<3> min = m_apex;
  Math::Point<3> max = m_apex;

  for (const auto& vertex : m_baseVertices) {
    for (int i = 0; i < 3; i++) {
      min.m_components[i] = std::min(min.m_components[i], vertex.m_components[i]);
      max.m_components[i] = std::max(max.m_components[i], vertex.m_components[i]);
    }
  }

  return Core::BoundingBox(min, max);
}

} // namespace Raytracer::Plugins

extern "C" {
Raytracer::Plugin::IPlugin *createPlugin() {
  return new Raytracer::Plugins::PyramidPlugin();
}

void destroyPlugin(Raytracer::Plugin::IPlugin *plugin) { delete plugin; }
}