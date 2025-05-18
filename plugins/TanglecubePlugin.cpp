#include "TanglecubePlugin.hpp"
#include "Parser/SceneParser.hpp"
#include <cmath>
#include <iostream>
#include <limits>

namespace Raytracer::Plugins {

std::unique_ptr<Plugin::PrimitivePlugin> TanglecubePlugin::create() {
    return std::make_unique<TanglecubePlugin>();
}

bool TanglecubePlugin::configure(const libconfig::Setting &config) {
    try {
        Parser::SceneParser::applyTransformations(config, this);

        if (config.exists("constant")) {
            m_constant = static_cast<double>(config.lookup("constant"));
        }

        if (config.exists("epsilon")) {
            m_epsilon = static_cast<double>(config.lookup("epsilon"));
        }

        if (config.exists("maxIterations")) {
            m_maxIterations = static_cast<int>(config.lookup("maxIterations"));
        }

        if (config.exists("stepSize")) {
            m_stepSize = static_cast<double>(config.lookup("stepSize"));
        }

        return true;
    } catch (const libconfig::SettingNotFoundException &e) {
        std::cerr << "Tanglecube configuration error: " << e.what() << std::endl;
        return false;
    } catch (const libconfig::SettingTypeException &e) {
        std::cerr << "Tanglecube configuration type error: " << e.what() << std::endl;
        return false;
    }
}

double TanglecubePlugin::evaluateImplicit(const Math::Point<3> &p) const noexcept {
    double x = p.m_components[0];
    double y = p.m_components[1];
    double z = p.m_components[2];

    return std::pow(x, 4) - 5 * std::pow(x, 2) +
           std::pow(y, 4) - 5 * std::pow(y, 2) +
           std::pow(z, 4) - 5 * std::pow(z, 2) +
           m_constant;
}

bool TanglecubePlugin::intersectBoundingBox(const Core::BoundingBox &box,
                                         const Core::Ray &ray,
                                         double &tmin,
                                         double &tmax) const noexcept {
    tmin = ray.getMinDistance();
    tmax = ray.getMaxDistance();

    const auto &o = ray.getOrigin().m_components;
    const auto &d = ray.getDirection().m_components;
    const auto &mn = box.getMin().m_components;
    const auto &mx = box.getMax().m_components;

    for (int i = 0; i < 3; ++i) {
        if (std::abs(d[i]) < 1e-12) {
            if (o[i] < mn[i] || o[i] > mx[i]) {
                return false;
            }
            continue;
        }

        double invD = 1.0 / d[i];
        double t0 = (mn[i] - o[i]) * invD;
        double t1 = (mx[i] - o[i]) * invD;

        if (t0 > t1) std::swap(t0, t1);

        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);

        if (tmin > tmax) {
            return false;
        }
    }

    return true;
}

Math::Vector<3> TanglecubePlugin::getNormalAt(const Math::Point<3> &p) const noexcept {
    double x = p.m_components[0];
    double y = p.m_components[1];
    double z = p.m_components[2];

    double dx = 4.0 * std::pow(x, 3) - 10.0 * x;
    double dy = 4.0 * std::pow(y, 3) - 10.0 * y;
    double dz = 4.0 * std::pow(z, 3) - 10.0 * z;

    Math::Vector<3> normal(dx, dy, dz);

    double length = normal.length();
    if (length > m_epsilon) {
        normal = normal / length;
    } else {
        if (std::abs(x) > std::abs(y) && std::abs(x) > std::abs(z)) {
            normal = Math::Vector<3>(x > 0 ? 1.0 : -1.0, 0.0, 0.0);
        } else if (std::abs(y) > std::abs(z)) {
            normal = Math::Vector<3>(0.0, y > 0 ? 1.0 : -1.0, 0.0);
        } else {
            normal = Math::Vector<3>(0.0, 0.0, z > 0 ? 1.0 : -1.0);
        }
    }

    return normal;
}

std::optional<Core::Intersection> TanglecubePlugin::intersect(const Core::Ray &ray) const noexcept {
    Core::Ray localRay = getTransform().inverseTransformRay(ray);
    Math::Point<3> origin = localRay.getOrigin();
    Math::Vector<3> direction = localRay.getDirection();

    double tmin, tmax;
    Math::Point<3> min(-2.5, -2.5, -2.5);
    Math::Point<3> max(2.5, 2.5, 2.5);
    Core::BoundingBox localBox(min, max);

    if (!intersectBoundingBox(localBox, localRay, tmin, tmax)) {
        return std::nullopt;
    }

    double t = std::max(tmin, localRay.getMinDistance());
    double tEnd = std::min(tmax, localRay.getMaxDistance());

    Math::Point<3> currentPos;
    double currentValue;
    double lastValue = 0.0;
    bool lastValueCalculated = false;

    const int MAX_BINARY_STEPS = 10;

    for (int i = 0; i < m_maxIterations && t < tEnd; ++i) {
        currentPos = origin + direction * t;
        currentValue = evaluateImplicit(currentPos);

        double stepDistance = std::abs(currentValue) * 0.5;

        if (std::abs(currentValue) < 0.5) {
            stepDistance *= 0.5;
        }

        stepDistance = std::max(stepDistance, m_stepSize);

        if (lastValueCalculated && ((lastValue < 0 && currentValue > 0) || (lastValue > 0 && currentValue < 0))) {
            double t1 = t - stepDistance;
            double t2 = t;

            for (int j = 0; j < MAX_BINARY_STEPS; j++) {
                double tmid = (t1 + t2) * 0.5;
                Math::Point<3> pmid = origin + direction * tmid;
                double vmid = evaluateImplicit(pmid);

                if (vmid * lastValue < 0) {
                    t2 = tmid;
                    currentValue = vmid;
                } else {
                    t1 = tmid;
                    lastValue = vmid;
                }
            }

            t = (t1 + t2) * 0.5;
            currentPos = origin + direction * t;
            currentValue = evaluateImplicit(currentPos);

            if (std::abs(currentValue) < m_epsilon) {
                Math::Vector<3> localNormal = getNormalAt(currentPos);

                bool isInside = localNormal.dot(direction) > 0;
                if (isInside) {
                    localNormal = localNormal * -1.0;
                }

                double radius = std::sqrt(
                    currentPos.m_components[0] * currentPos.m_components[0] +
                    currentPos.m_components[1] * currentPos.m_components[1] +
                    currentPos.m_components[2] * currentPos.m_components[2]
                );

                if (radius < m_epsilon) radius = m_epsilon;

                double phi = std::atan2(currentPos.m_components[2], currentPos.m_components[0]);
                double theta = std::acos(std::clamp(currentPos.m_components[1] / radius, -1.0, 1.0));

                double u = (phi + M_PI) / (2 * M_PI);
                double v = theta / M_PI;

                Math::Point<3> worldPoint = getTransform().transformPoint(currentPos);
                Math::Vector<3> worldNormal = getTransform().transformNormal(localNormal);
                worldNormal = worldNormal / worldNormal.length();

                double worldDistance = (worldPoint - ray.getOrigin()).length();

                return Core::Intersection(
                    worldPoint,
                    worldNormal,
                    getMaterial(),
                    worldDistance,
                    isInside,
                    Math::Point<2>(u, v)
                );
            }
        }

        lastValue = currentValue;
        lastValueCalculated = true;

        double stepDistanceBis = std::abs(currentValue) * 0.5;

        if (std::abs(currentValue) < 0.5) {
            stepDistanceBis *= 0.5;
        }

        stepDistanceBis = std::max(stepDistanceBis, m_stepSize);

        t += stepDistanceBis;
    }

    double distToOrigin = origin.m_components[0] * origin.m_components[0] +
                          origin.m_components[1] * origin.m_components[1] +
                          origin.m_components[2] * origin.m_components[2];

    if (std::abs(evaluateImplicit(origin)) < 0.1 && distToOrigin < 10.0) {
        Math::Vector<3> localNormal = getNormalAt(origin);
        if (localNormal.dot(direction) > 0) {
            Math::Point<3> worldPoint = getTransform().transformPoint(origin);
            Math::Vector<3> worldNormal = getTransform().transformNormal(localNormal);
            worldNormal = worldNormal / worldNormal.length();

            double worldDistance = (worldPoint - ray.getOrigin()).length();

            return Core::Intersection(
                worldPoint,
                worldNormal,
                getMaterial(),
                worldDistance,
                true,
                Math::Point<2>(0.5, 0.5)
            );
        }
    }

    return std::nullopt;
}

Core::BoundingBox TanglecubePlugin::getBoundingBox() const noexcept {
    double bound = 2.5;

    Math::Point<3> min(-bound, -bound, -bound);
    Math::Point<3> max(bound, bound, bound);

    return getTransform().transformBoundingBox(Core::BoundingBox(min, max));
}

} // namespace Raytracer::Plugins

extern "C" {
Raytracer::Plugin::IPlugin *createPlugin() {
    return new Raytracer::Plugins::TanglecubePlugin();
}

void destroyPlugin(Raytracer::Plugin::IPlugin *plugin) {
    delete plugin;
}
}