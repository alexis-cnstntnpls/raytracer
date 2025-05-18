#pragma once
#include "Plugin/PrimitivePlugin.hpp"

namespace Raytracer::Plugins {

/**
 * @class PyramidPlugin
 * @brief Plugin for creating a pyramid primitive with a square base
 */
class PyramidPlugin : public Plugin::PrimitivePlugin {
public:
  /**
   * @brief Default constructor
   */
  PyramidPlugin() noexcept = default;

  /**
   * @brief Virtual destructor
   */
  ~PyramidPlugin() noexcept override = default;

  /**
   * @brief Get the name of this plugin
   * @return The name of the plugin
   */
  [[nodiscard]]
  std::string getName() const override {
    return "Pyramid";
  }

  /**
   * @brief Create a new instance of the pyramid primitive plugin
   * @return A unique pointer to the new pyramid primitive
   */
  std::unique_ptr<PrimitivePlugin> create() override;

  /**
   * @brief Configure the pyramid plugin with a libconfig setting
   * @param config The libconfig setting to configure the pyramid
   * @return True if the configuration was successful, false otherwise
   */
  bool configure(const libconfig::Setting &config) override;

  /**
   * @brief Calculate the axis-aligned bounding box for this Pyramid
   * @return The bounding box that represents this Pyramid
   */
  [[nodiscard]] Core::BoundingBox getBoundingBox() const noexcept override;

  /**
   * @brief Calculate intersection between a ray and this Pyramid
   * @param ray The ray to test for intersection
   * @return An optional Intersection object, containing intersection data if
   * the ray hits the Pyramid
   */
  [[nodiscard]] std::optional<Core::Intersection>
  intersect(const Core::Ray &ray) const noexcept override;

private:
  /**
   * @brief Set the parameters of the Pyramid
   * @param baseCenter Center of the base
   * @param baseSize Side length of the square base
   * @param height Height of the pyramid from base to apex
   * @param upAxis The axis direction ("X", "Y", or "Z")
   */
  void setParameters(const Math::Point<3> &baseCenter, double baseSize,
                     double height, const std::string &upAxis) noexcept;

  /**
   * @brief Helper method for triangle intersection
   * @param ray The ray to test
   * @param v0 First vertex of the triangle
   * @param v1 Second vertex of the triangle
   * @param v2 Third vertex of the triangle
   * @return Optional pair of intersection distance and normal if hit
   */
  [[nodiscard]] std::optional<std::pair<double, Math::Vector<3>>>
  intersectTriangle(const Core::Ray &ray, const Math::Point<3> &v0,
                    const Math::Point<3> &v1, const Math::Point<3> &v2) const noexcept;

  Math::Point<3> m_baseCenter{0.0, 0.0, 0.0};
  double m_baseSize{2.0};
  double m_height{2.0};
  Math::Vector<3> m_upAxis{0.0, 1.0, 0.0};
  Math::Point<3> m_apex{0.0, 0.0, 0.0};
  std::array<Math::Point<3>, 4> m_baseVertices;
};

} // namespace Raytracer::Plugins