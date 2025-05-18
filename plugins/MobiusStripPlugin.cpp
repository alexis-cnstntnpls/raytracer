#include "MobiusStripPlugin.hpp"
 #include "Parser/SceneParser.hpp"
 
 #include <algorithm>
 #include <cmath>
 #include <iostream>
 #include <limits>
 
 using Raytracer::Math::Point;
 using Raytracer::Math::Vector;
 using Raytracer::Core::Ray;
 using Raytracer::Core::BoundingBox;
 using Raytracer::Core::Intersection;
 
 
 namespace {
 
 inline bool hitAABB(const BoundingBox &box, const Ray &ray, double &tmin,
  double &tmax) noexcept
 {
   tmin = 0.0;
   tmax = ray.getMaxDistance();
 
   const auto &o = ray.getOrigin().m_components;
   const auto &d = ray.getDirection().m_components;
   const auto &mn = box.getMin().m_components;
   const auto &mx = box.getMax().m_components;
 
   for (int i = 0; i < 3; ++i) {
     if (std::fabs(d[i]) < 1e-12) {
       if (o[i] < mn[i] || o[i] > mx[i]) return false;
       continue;
     }
     double invD = 1.0 / d[i];
     double t0 = (mn[i] - o[i]) * invD;
     double t1 = (mx[i] - o[i]) * invD;
     if (t0 > t1) std::swap(t0, t1);
     tmin = std::max(tmin, t0);
     tmax = std::min(tmax, t1);
     if (tmin > tmax) return false;
   }
   return true;
 }
 
 inline std::optional<std::pair<double, Vector<3>>> intersectTri(const Ray &ray, const Point<3> &v0,
  const Point<3> &v1, const Point<3> &v2, const Vector<3> &nCached) noexcept
 {
   const double eps = 1e-6;
   Vector<3> e1 = v1 - v0;
   Vector<3> e2 = v2 - v0;
   Vector<3> p = ray.getDirection().cross(e2);
   double det = e1.dot(p);
   if (std::fabs(det) < eps) return std::nullopt;
   double invDet = 1.0 / det;
 
   Vector<3> tvec = ray.getOrigin() - v0;
   double u = tvec.dot(p) * invDet;
   if (u < 0 || u > 1) return std::nullopt;
 
   Vector<3> qvec = tvec.cross(e1);
   double v = ray.getDirection().dot(qvec) * invDet;
   if (v < 0 || u + v > 1) return std::nullopt;
 
   double t = e2.dot(qvec) * invDet;
   if (t < 1e-6 || t > ray.getMaxDistance()) return std::nullopt;
 
   Vector<3> n = (det < 0) ? nCached * -1.0 : nCached;
   return std::make_pair(t, n);
 }
 
 } 
 
 
 namespace Raytracer::Plugins {
 
 
 std::unique_ptr<Plugin::PrimitivePlugin> MobiusStripPlugin::create() {
  return std::make_unique<MobiusStripPlugin>();
 }
 
 
 bool MobiusStripPlugin::configure(const libconfig::Setting &cfg)
 {
   try {
     auto pOpt = Parser::SceneParser::parsePoint3(cfg.lookup("position"));
     if (!pOpt) { std::cerr << "[MobiusStrip] position invalide\n"; return false; }
     m_center = *pOpt;
 
     m_radius = cfg.lookup("radius");
     m_halfWidth = cfg.lookup("halfWidth");
 
     std::string ax = cfg.lookup("axis");
     if (ax == "X")
      m_axis = {1.0, 0.0, 0.0};
     else if (ax == "Y")
      m_axis = {0.0, 1.0, 0.0};
     else
      m_axis = {0.0, 0.0, 1.0};
   }
   catch (const libconfig::SettingException &e) {
     std::cerr << "[MobiusStrip] " << e.what() << '\n';
     return false;
   }
 
   buildMesh();
   buildBVH();
   return true;
 }


void MobiusStripPlugin::buildMesh(unsigned segU, unsigned segV)
{
  m_tris.clear();

  Vector<3> up = m_axis; up.normalize();
  Vector<3> tmp = std::fabs(up.m_components[0]) < 0.9 ? Vector<3>{1.0,0.0,0.0} : Vector<3>{0.0,1.0,0.0};
  Vector<3> a = (up.cross(tmp)).normalize();
  Vector<3> b = up.cross(a);

  auto param = [&](double u, double v) -> Point<3>
  {
    double cu = std::cos(u), su = std::sin(u);
    double c2 = std::cos(u*0.5), s2 = std::sin(u*0.5);

    Vector<3> n = a*cu + b*su;
    Point<3> C = m_center + n * m_radius;

    return C + n * (v * c2) + up * (v * s2);
  };

  double du = 2*M_PI / segU;
  double dv = 2*m_halfWidth / segV;

  Point<3> bbMin{ std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::max(),
                  std::numeric_limits<double>::max() };
  Point<3> bbMax{ -bbMin.m_components[0],
                  -bbMin.m_components[1],
                  -bbMin.m_components[2] };

  for (unsigned iu = 0; iu < segU; ++iu) {
    double u0 = iu * du, u1 = (iu + 1) * du;
    for (unsigned iv = 0; iv < segV; ++iv) {
      double v0 = -m_halfWidth + iv * dv;
      double v1 = v0 + dv;

      Point<3> p00 = param(u0, v0);
      Point<3> p10 = param(u1, v0);
      Point<3> p01 = param(u0, v1);
      Point<3> p11 = param(u1, v1);

      Vector<3> n0 = (p10 - p00).cross(p01 - p00).normalize();
      m_tris.push_back({p00, p10, p01, n0});
      Vector<3> n1 = (p11 - p10).cross(p01 - p10).normalize();
      m_tris.push_back({p10, p11, p01, n1});

      for (const auto &p : {p00, p10, p01, p11})
        for (int k = 0; k < 3; ++k) {
          bbMin.m_components[k] = std::min(bbMin.m_components[k], p.m_components[k]);
          bbMax.m_components[k] = std::max(bbMax.m_components[k], p.m_components[k]);
        }
    }
  }
  m_bbox = BoundingBox(bbMin, bbMax);
}
 
 void MobiusStripPlugin::buildBVH()
 {
   m_bvh.clear();
   m_bvh.reserve(m_tris.size()*2);
   buildBVHRecursive(0, static_cast<int>(m_tris.size()));
 }
 
 int MobiusStripPlugin::buildBVHRecursive(int first, int count)
 {
   int idx = static_cast<int>(m_bvh.size());
   m_bvh.push_back({});
   Node &node = m_bvh.back();
   node.first = first;
   node.count = count;
 
   Point<3> mn{ std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max() };
   Point<3> mx{ -mn.m_components[0], -mn.m_components[1], -mn.m_components[2] };
   for (int i = 0; i < count; ++i) {
     const auto &t = m_tris[first + i];
     for (const auto &p : {t.p0, t.p1, t.p2}) {
       for (int k = 0; k < 3; ++k) {
         mn.m_components[k] = std::min(mn.m_components[k], p.m_components[k]);
         mx.m_components[k] = std::max(mx.m_components[k], p.m_components[k]);
       }
     }
   }
   node.bbox = BoundingBox(mn, mx);
 
   if (count <= 8) return idx;
 
   Vector<3> d = node.bbox.getMax() - node.bbox.getMin();
   int axis = (d.m_components[0] > d.m_components[1] &&
               d.m_components[0] > d.m_components[2]) ? 0 :
              (d.m_components[1] > d.m_components[2] ? 1 : 2);
 
   int mid = first + count / 2;
   std::nth_element(m_tris.begin() + first, m_tris.begin() + mid,
                    m_tris.begin() + first + count,
                    [axis](const Triangle &a, const Triangle &b) {
                      double ca = (a.p0.m_components[axis] + a.p1.m_components[axis] + a.p2.m_components[axis]) / 3.0;
                      double cb = (b.p0.m_components[axis] + b.p1.m_components[axis] + b.p2.m_components[axis]) / 3.0;
                      return ca < cb;
                    });
 
   node.left = buildBVHRecursive(first, mid - first);
   node.right = buildBVHRecursive(mid, first + count - mid);
   node.count = 0;
   return idx;
 }
 
 
 std::optional<std::pair<double, Vector<3>>>
 MobiusStripPlugin::intersectNode(int idx, const Ray &ray) const noexcept
 {
   const Node &node = m_bvh[idx];
   double tmin, tmax;
   if (!hitAABB(node.bbox, ray, tmin, tmax)) return std::nullopt;
 
   if (node.count > 0) {
     double bestT = std::numeric_limits<double>::infinity();
     Vector<3> bestN;
     for (int i = 0; i < node.count; ++i) {
       const auto &tri = m_tris[node.first + i];
       auto h = intersectTri(ray, tri.p0, tri.p1, tri.p2, tri.n);
       if (h && h->first < bestT) { bestT = h->first; bestN = h->second; }
     }
     if (bestT < std::numeric_limits<double>::infinity())
       return {{bestT, bestN}};
     return std::nullopt;
   }
 
   auto L = intersectNode(node.left, ray);
   auto R = intersectNode(node.right, ray);
   if (L && R) return (L->first < R->first) ? L : R;
   return L ? L : R;
 }
 
 
 std::optional<Intersection>
 MobiusStripPlugin::intersect(const Ray &ray) const noexcept
 {
   auto res = intersectNode(0, ray);
   if (!res) return std::nullopt;
 
   double t = res->first;
   Vector<3> n = res->second;
   Point<3> hit = ray.getOrigin() + ray.getDirection() * t;
 
   bool inside = false;
   if (n.dot(ray.getDirection()) > 0.0) { n *= -1.0; inside = true; }
 
   
   std::shared_ptr<Core::IMaterial> mat = getMaterial();
 
   return Intersection(hit, n, mat, t, inside, Raytracer::Math::Point<2>{0.0, 0.0});
 }
 
  Core::BoundingBox MobiusStripPlugin::getBoundingBox() const noexcept {
    return m_bbox;
  }

 } 
 
 
 extern "C" {
 Raytracer::Plugin::IPlugin *createPlugin() {
  return new Raytracer::Plugins::MobiusStripPlugin();
 }
 void destroyPlugin(Raytracer::Plugin::IPlugin *plugin) { delete plugin; }
 }
