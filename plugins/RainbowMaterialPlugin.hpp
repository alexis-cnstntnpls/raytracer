#pragma once
#include "Plugin/MaterialPlugin.hpp"

namespace Raytracer::Plugins {

/**
 * @class RainbowMaterialPlugin
 * @brief Plugin for creating a rainbow material
 */
class RainbowMaterialPlugin : public Plugin::MaterialPlugin {
public:
  /**
   * @brief Default constructor
   */
  RainbowMaterialPlugin() noexcept = default;

  /**
   * @brief Virtual destructor
   */
  ~RainbowMaterialPlugin() noexcept override = default;

  /**
   * @brief Get the name of this plugin
   * @return The name of the plugin
   */
  [[nodiscard]]
  std::string getName() const override {
    return "RainbowMaterial";
  }

  /**
   * @brief Create a new instance of the rainbow material primitive plugin
   * @return A unique pointer to the new rainbow material primitive
   */
  std::shared_ptr<MaterialPlugin> create() override;

  /**
   * @brief Configure the rainbow material plugin with a libconfig setting
   * @param config The libconfig setting to configure the rainbow material
   * @return True if the configuration was successful, false otherwise
   */
  bool configure(const libconfig::Setting &config) override;

  /**
   * @brief Compute the color resulting from lighting at an intersection.
   * @param intersection Intersection information.
   * @param ray Incoming ray.
   * @param lights Lights affecting the intersection.
   * @param scene Scene containing all objects.
   * @return Computed color.
   */
  [[nodiscard]] Core::Color
  computeColor(const Core::Intersection &intersection, const Core::Ray &ray,
               const std::vector<std::shared_ptr<Core::ILight>> &lights,
               const Core::Scene &scene) const override;
private:
  Math::Point<3> m_origin{0.0, 0.0, 0.0};
};

} // namespace Raytracer::Plugins
