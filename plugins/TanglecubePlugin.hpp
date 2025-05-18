#pragma once
#include "Plugin/PrimitivePlugin.hpp"

namespace Raytracer::Plugins {

/**
 * @class TanglecubePlugin
 * @brief Plugin for creating a Tanglecube primitive
 *
 * A Tanglecube is a surface defined by the implicit equation:
 * x^4 - 5x^2 + y^4 - 5y^2 + z^4 - 5z^2 + D = 0
 * where D is a constant that controls the shape.
 */
class TanglecubePlugin : public Plugin::PrimitivePlugin {
public:
    /**
     * @brief Default constructor
     */
    TanglecubePlugin() noexcept = default;

    /**
     * @brief Virtual destructor
     */
    ~TanglecubePlugin() noexcept override = default;

    /**
     * @brief Get the name of this plugin
     * @return The name of the plugin
     */
    [[nodiscard]]
    std::string getName() const override {
        return "Tanglecube";
    }

    /**
     * @brief Create a new instance of the Tanglecube primitive plugin
     * @return A unique pointer to the new Tanglecube primitive
     */
    std::unique_ptr<PrimitivePlugin> create() override;

    /**
     * @brief Configure the Tanglecube plugin with a libconfig setting
     * @param config The libconfig setting to configure the Tanglecube
     * @return True if the configuration was successful, false otherwise
     */
    bool configure(const libconfig::Setting &config) override;

    /**
     * @brief Calculate intersection between a ray and this Tanglecube
     * @param ray The ray to test for intersection
     * @return An optional Intersection object, containing intersection data if
     * the ray hits the Tanglecube
     */
    [[nodiscard]] std::optional<Core::Intersection>
    intersect(const Core::Ray &ray) const noexcept override;

    /**
     * @brief Calculate the axis-aligned bounding box for this Tanglecube
     * @return The bounding box that represents this Tanglecube
     */
    [[nodiscard]] Core::BoundingBox getBoundingBox() const noexcept override;

private:
    /**
     * @brief Evaluate the Tanglecube equation at a given point
     * @param p The point to evaluate
     * @return The value of the Tanglecube equation at the point
     */
    [[nodiscard]] double evaluateImplicit(const Math::Point<3> &p) const noexcept;

    /**
     * @brief Calculate the normal of the Tanglecube at a given point
     * @param p The point to calculate the normal at
     * @return The normal vector at the point
     */
    [[nodiscard]] Math::Vector<3> getNormalAt(const Math::Point<3> &p) const noexcept;

    /**
     * @brief Check if a ray intersects a bounding box
     * @param box The bounding box to check
     * @param ray The ray to check
     * @param tmin The minimum distance along the ray
     * @param tmax The maximum distance along the ray
     * @return True if the ray intersects the bounding box
     */
    [[nodiscard]] bool intersectBoundingBox(const Core::BoundingBox &box,
                                           const Core::Ray &ray,
                                           double &tmin,
                                           double &tmax) const noexcept;

    double m_constant = 11.8;
    double m_epsilon = 0.00001;
    int m_maxIterations = 100;
    double m_stepSize = 0.01;
};

} // namespace Raytracer::Plugins