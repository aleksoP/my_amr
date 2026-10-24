#include "lidar_dock_detector/model.hpp"

#include "lidar_dock_detector/geometry_utils.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>

namespace lidar_dock_detector
{

namespace
{

std::vector<Eigen::Vector2d> buildVerticesFromSegments(
  const std::vector<double> & segment_lengths,
  const std::vector<double> & turn_angles_rad)
{
  std::vector<Eigen::Vector2d> vertices;
  vertices.reserve(segment_lengths.size() + 1U);
  vertices.push_back(Eigen::Vector2d::Zero());

  double heading = 0.0;
  Eigen::Vector2d cursor = Eigen::Vector2d::Zero();
  for (std::size_t i = 0; i < segment_lengths.size(); ++i) {
    cursor += Eigen::Vector2d(std::cos(heading), std::sin(heading)) * segment_lengths[i];
    vertices.push_back(cursor);
    if (i < turn_angles_rad.size()) {
      heading += turn_angles_rad[i];
    }
  }

  return vertices;
}

std::vector<Eigen::Vector2d> buildVerticesFromArrays(
  const std::vector<double> & vertices_x,
  const std::vector<double> & vertices_y)
{
  if (vertices_x.size() != vertices_y.size()) {
    throw std::runtime_error("landmark.vertices_x and landmark.vertices_y must have the same length");
  }
  if (vertices_x.size() < 2U) {
    throw std::runtime_error("landmark.vertices_x/y must define at least two vertices");
  }

  std::vector<Eigen::Vector2d> vertices;
  vertices.reserve(vertices_x.size());
  for (std::size_t i = 0; i < vertices_x.size(); ++i) {
    vertices.emplace_back(vertices_x[i], vertices_y[i]);
  }
  return vertices;
}

Eigen::Vector2d chooseOriginPoint(
  const std::vector<Eigen::Vector2d> & raw_vertices,
  const LandmarkConfig & config)
{
  if (raw_vertices.empty()) {
    return Eigen::Vector2d::Zero();
  }

  if (config.origin_mode == "start") {
    return raw_vertices.front();
  }
  if (config.origin_mode == "end") {
    return raw_vertices.back();
  }
  if (config.origin_mode == "centroid") {
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto & vertex : raw_vertices) {
      centroid += vertex;
    }
    return centroid / static_cast<double>(raw_vertices.size());
  }
  if (config.origin_mode == "apex") {
    return *std::max_element(
      raw_vertices.begin(), raw_vertices.end(),
      [](const Eigen::Vector2d & a, const Eigen::Vector2d & b) {
        if (a.y() == b.y()) {
          return a.x() < b.x();
        }
        return a.y() < b.y();
      });
  }
  if (config.origin_mode == "vertex") {
    if (config.origin_vertex < 0 ||
      config.origin_vertex >= static_cast<int>(raw_vertices.size()))
    {
      throw std::runtime_error("landmark.origin_vertex is out of range");
    }
    return raw_vertices[static_cast<std::size_t>(config.origin_vertex)];
  }

  throw std::runtime_error(
          "landmark.origin_mode must be one of: start, end, centroid, apex, vertex");
}

std::vector<Eigen::Vector2d> canonicalizeVertices(
  const std::vector<Eigen::Vector2d> & raw_vertices,
  const LandmarkConfig & config)
{
  const Eigen::Vector2d origin = chooseOriginPoint(raw_vertices, config);
  std::vector<Eigen::Vector2d> canonical_vertices;
  canonical_vertices.reserve(raw_vertices.size());
  for (const auto & vertex : raw_vertices) {
    canonical_vertices.push_back(rotate(vertex - origin, -config.forward_axis_rad));
  }
  return canonical_vertices;
}

std::vector<double> computeSegmentLengths(const std::vector<Eigen::Vector2d> & vertices)
{
  std::vector<double> lengths;
  if (vertices.size() < 2U) {
    return lengths;
  }

  lengths.reserve(vertices.size() - 1U);
  for (std::size_t i = 0; i + 1 < vertices.size(); ++i) {
    lengths.push_back((vertices[i + 1] - vertices[i]).norm());
  }
  return lengths;
}

std::vector<double> computeTurnAngles(const std::vector<Eigen::Vector2d> & vertices)
{
  std::vector<double> turn_angles;
  if (vertices.size() < 3U) {
    return turn_angles;
  }

  turn_angles.reserve(vertices.size() - 2U);
  for (std::size_t i = 0; i + 2 < vertices.size(); ++i) {
    const Eigen::Vector2d first = (vertices[i + 1] - vertices[i]).normalized();
    const Eigen::Vector2d second = (vertices[i + 2] - vertices[i + 1]).normalized();
    turn_angles.push_back(signedTurnAngle(first, second));
  }
  return turn_angles;
}

std::vector<Eigen::Vector2d> buildModelSamples(
  const std::vector<Eigen::Vector2d> & vertices,
  const double sample_spacing)
{
  std::vector<Eigen::Vector2d> samples;
  if (vertices.size() < 2U) {
    return samples;
  }

  for (std::size_t i = 0; i + 1 < vertices.size(); ++i) {
    const Eigen::Vector2d delta = vertices[i + 1] - vertices[i];
    const double length = delta.norm();
    const Eigen::Vector2d direction = length > 1e-6 ?
      Eigen::Vector2d(delta / length) : Eigen::Vector2d(1.0, 0.0);
    const int steps = std::max(1, static_cast<int>(std::ceil(length / sample_spacing)));
    for (int step = 0; step < steps; ++step) {
      const double distance_along = std::min(length, step * sample_spacing);
      samples.push_back(vertices[i] + direction * distance_along);
    }
  }
  samples.push_back(vertices.back());
  return samples;
}

}  // namespace

LandmarkModel buildLandmarkModel(const LandmarkConfig & config)
{
  LandmarkModel model;

  const std::vector<Eigen::Vector2d> raw_vertices =
    !config.vertices_x.empty() || !config.vertices_y.empty() ?
    buildVerticesFromArrays(config.vertices_x, config.vertices_y) :
    buildVerticesFromSegments(config.segment_lengths, config.turn_angles_rad);

  model.vertices = canonicalizeVertices(raw_vertices, config);
  model.segment_lengths = computeSegmentLengths(model.vertices);
  model.turn_angles_rad = computeTurnAngles(model.vertices);
  model.total_length = std::accumulate(model.segment_lengths.begin(), model.segment_lengths.end(), 0.0);
  model.samples = buildModelSamples(model.vertices, config.sample_spacing);
  return model;
}

}  // namespace lidar_dock_detector
