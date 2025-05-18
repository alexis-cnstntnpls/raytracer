#pragma once

#include "Math/Point.hpp"
#include "Plugin/PrimitivePlugin.hpp"

namespace Raytracer::Plugins {

/**
 * @class FractalPlugin
 * @brief A plugin representing a fractal primitive in the ray tracer.
 */
class FractalPlugin : public virtual Plugin::PrimitivePlugin {
public:
  /**
   * @brief Default constructor
   */
  FractalPlugin() noexcept = default;

  /**
   * @brief Virtual destructor
   */
  ~FractalPlugin() noexcept override = default;

  /**
   * @brief Get the name of this plugin
   * @return The name of the plugin
   */
  [[nodiscard]] std::string getName() const override { return "Fractal"; }


  /**
   * @brief Create a new instance of the Fractal primitive plugin
   * @return A unique pointer to the new Fractal primitive
   */
  std::unique_ptr<PrimitivePlugin> create() override;

  /**
   * @brief Configure the Fractal plugin with a libconfig setting
   * @param config The libconfig setting to configure the Fractal
   * @return True if the configuration was successful, false otherwise
   */
  bool configure(const libconfig::Setting &config) override;

  /**
   * @brief Calculate the intersection between a ray and this Fractal
   * @param ray The ray to test for intersection
   * @return An optional Intersection object, containing intersection data if
   * the ray hits the Fractal
   */
  [[nodiscard]] std::optional<Core::Intersection>
  intersect(const Core::Ray &ray) const noexcept override;

  /**
   * @brief Calculate the BoundingBox of the fractal object
   * @return The bounding box that represents this fractal
   */
  [[nodiscard]] Core::BoundingBox getBoundingBox() const noexcept override;

private:
  Math::Point<3> m_position = {0.0, 0.0, 0.0};
  int m_iterations = 10;
  double m_bailout = 4.0;
  double m_power = 8.0;

  /**
   * @brief Calculate the distance between the fractale object and the point
   * @param p The position of the point
   * @return The distance to the fractale
   */
  [[nodiscard]] double mandelbulbDistanceEstimator(Math::Point<3> p) const;
};

} // namespace Raytracer::Plugins
