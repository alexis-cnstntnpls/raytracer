#include "RainbowMaterialPlugin.hpp"
#include "Core/Color.hpp"
#include "Parser/SceneParser.hpp"
#include "Plugin/MaterialPlugin.hpp"

#include <iostream>
namespace Raytracer::Plugins {

std::shared_ptr<Plugin::MaterialPlugin> RainbowMaterialPlugin::create() {
  return std::make_shared<RainbowMaterialPlugin>();
}

bool RainbowMaterialPlugin::configure(const libconfig::Setting &config) {
  try {
    std::optional<Math::Point<3>> origin = Parser::SceneParser::parsePoint3(config.lookup("origin"));
    std::optional<Core::Color> color = Parser::SceneParser::parseColor(config, "color");

    if (!color || !origin) {
      return false;
    }

    const double ambiantCoefficient = config.lookup("ambientCoefficient");
    const double diffuseCoefficient = config.lookup("diffuseCoefficient");

    this->setAmbientCoefficient(ambiantCoefficient);
    this->setDiffuseCoefficient(diffuseCoefficient);
    this->setAmbientColor(*color);
    this->setDiffuseColor(*color);
    this->m_origin = *origin;
  } catch (const libconfig::SettingNotFoundException &) {
    return false;
  }
  return true;
}

Core::Color RainbowMaterialPlugin::computeColor(
    const Core::Intersection &intersection,
    [[maybe_unused]] const Core::Ray &ray,
    const std::vector<std::shared_ptr<Core::ILight>> &lights,
    const Core::Scene &scene) const {
  auto distance = (intersection.getPoint() - m_origin);

  double dist = std::sqrt(distance.m_components[0] * distance.m_components[0] +
                         distance.m_components[1] * distance.m_components[1]);

  double r = std::sin(0.05 * dist) * 127.5 + 127.5;
  double g = std::sin(0.05 * dist + 2.0) * 127.5 + 127.5;
  double b = std::sin(0.05 * dist + 4.0) * 127.5 + 127.5;

  Core::Color baseColor(r, g, b);
  Core::Color finalColor(0, 0, 0);

  for (const auto &light : lights) {
    const Math::Vector<3> &normal = intersection.getNormal();

    if (auto ambientLight = std::dynamic_pointer_cast<Core::IAmbientLight>(light)) {
      double ambientIntensity = ambientLight->getIntensity();
      const Core::Color &lightColor = ambientLight->getColor();
      
      Core::Color ambComp{
          baseColor.getR() * getAmbientCoefficient() * ambientIntensity * lightColor.getR() / 255.0,
          baseColor.getG() * getAmbientCoefficient() * ambientIntensity * lightColor.getG() / 255.0,
          baseColor.getB() * getAmbientCoefficient() * ambientIntensity * lightColor.getB() / 255.0
      };
      
      finalColor = finalColor.add(ambComp);
    } 
    else if (auto directionalLight = std::dynamic_pointer_cast<Core::IDirectionalLight>(light)) {
      Math::Vector<3> lightDir = directionalLight->getDirection() * -1.0;
      double dotResult = normal.dot(lightDir);

      if (dotResult > 0.0) {
        double lightIntensity = directionalLight->getIntensity();
        const Core::Color &lightColor = directionalLight->getColor();
        double diffuseFactor = getDiffuseCoefficient() * dotResult * lightIntensity;

        Core::Color diffuseComponent{
            baseColor.getR() * diffuseFactor * lightColor.getR() / 255.0,
            baseColor.getG() * diffuseFactor * lightColor.getG() / 255.0,
            baseColor.getB() * diffuseFactor * lightColor.getB() / 255.0
        };

        finalColor = finalColor.add(diffuseComponent);
      }
    } 
    else if (auto positionalLight = std::dynamic_pointer_cast<Core::IPositionalLight>(light)) {
      Math::Vector<3> lightDir = positionalLight->getDirectionFrom(intersection.getPoint());
      double dotResult = normal.dot(lightDir);

      if (dotResult > 0.0) {
        const double epsilon = 1e-4;
        Math::Point<3> shadowRayOrigin = intersection.getPoint() + normal * epsilon;
        double distanceToLight = (positionalLight->getPosition() - shadowRayOrigin).length();

        Core::Ray shadowRay(shadowRayOrigin, lightDir, epsilon, distanceToLight);
        bool inShadow = scene.hasIntersection(shadowRay);

        if (!inShadow) {
          double lightIntensity = positionalLight->getIntensity();
          const Core::Color &lightColor = positionalLight->getColor();
          double diffuseFactor = getDiffuseCoefficient() * dotResult * lightIntensity;

          Core::Color diffuseComponent{
              baseColor.getR() * diffuseFactor * lightColor.getR() / 255.0,
              baseColor.getG() * diffuseFactor * lightColor.getG() / 255.0,
              baseColor.getB() * diffuseFactor * lightColor.getB() / 255.0
          };
          
          finalColor = finalColor.add(diffuseComponent);
        }
      }
    }
  }
  finalColor = Core::Color(
    std::min(255.0, std::max(0.0, finalColor.getR())),
    std::min(255.0, std::max(0.0, finalColor.getG())),
    std::min(255.0, std::max(0.0, finalColor.getB()))
  );
  
  return finalColor;
}

} // namespace Raytracer::Plugins

extern "C" {
Raytracer::Plugin::IPlugin *createPlugin() {
  return new Raytracer::Plugins::RainbowMaterialPlugin();
}

void destroyPlugin(Raytracer::Plugin::IPlugin *plugin) { delete plugin; }
}