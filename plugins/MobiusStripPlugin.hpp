#pragma once
#include "Plugin/PrimitivePlugin.hpp"
#include "Core/BoundingBox.hpp"
#include "Math/Point.hpp"
#include "Math/Vector.hpp"
#include <vector>
#include <optional>

namespace Raytracer::Plugins {

/**
 * @class MobiusStripPlugin
 * @brief Plugin for creating a Möbius strip primitive
 */
class MobiusStripPlugin : public Plugin::PrimitivePlugin {
public:
  MobiusStripPlugin() noexcept = default;
  ~MobiusStripPlugin() noexcept override = default;

  /**
   * @brief Get the name of this plugin
   * @return The name of the plugin
   */
  [[nodiscard]]
  std::string getName() const override {
    return "MobiusStrip";
  }

  /**
   * @brief Create a new instance of the Möbius strip primitive plugin
   * @return A unique pointer to the new Möbius strip primitive
   */
  std::unique_ptr<PrimitivePlugin> create() override;

  /**
   * @brief Configure the Möbius strip plugin with a libconfig setting
   * @param config The libconfig setting to configure the primitive
   * @return True if the configuration was successful, false otherwise
   */
  bool configure(const libconfig::Setting &config) override;

  /**
   * @brief Calculate intersection between a ray and this Möbius strip
   * @param ray The ray to test for intersection
   * @return An optional Intersection object, containing intersection data if
   * the ray hits the primitive
   */
  [[nodiscard]]
  std::optional<Core::Intersection> intersect(const Core::Ray &ray) const noexcept override;

  /**
   * @brief Calculate the axis-aligned bounding box for this Möbius strip
   * @return The bounding box that completely contains the primitive
   */
  [[nodiscard]]
  Core::BoundingBox getBoundingBox() const noexcept override;

private:
  struct Triangle {
    Math::Point<3> p0, p1, p2;
    Math::Vector<3> n;
  };

  struct Node {
    Core::BoundingBox bbox;
    int left{-1};
    int right{-1};
    int first{0};
    int count{0};
  };

  Math::Point<3> m_center{0.0, 0.0, 0.0};
  Math::Vector<3> m_axis{0.0, 1.0, 0.0};
  double m_radius{40.0};
  double m_halfWidth{8.0};
  double m_segU{128.0};
  double m_segV{16.0};

  std::vector<Triangle> m_tris;
  std::vector<Node> m_bvh;
  Core::BoundingBox m_bbox;

  void buildMesh();
  void buildBVH();
  int buildBVHRecursive(int first, int count);
  std::optional<std::pair<double, Math::Vector<3>>> intersectNode(int idx, const Core::Ray &ray) const noexcept;
};

} // namespace Raytracer::Plugins
