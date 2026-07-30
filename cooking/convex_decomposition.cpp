/**************************************************************************/
/* convex_decomposition.cpp                                               */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/* you may not use this file except in compliance with the License.       */
/* You may obtain a copy of the License at                                */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include <SushiEngine/physics/cooking/convex_decomposition.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

#include <SushiEngine/geometry/mesh_utilities.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            namespace
            {
                /** @brief One supporting plane of a hull: an outward unit normal and an offset. */
                struct HullPlane
                {
                    Vector3 normal;
                    Scalar offset;
                };

                /** @brief The largest extent of a point set, for scaling the tolerances. */
                Scalar point_set_extent(const std::vector<Vector3>& points) noexcept
                {
                    if (points.empty())
                        return 0;
                    Vector3 low = points[0];
                    Vector3 high = points[0];
                    for (const Vector3& point : points)
                    {
                        low = Vector3{std::min(low.x, point.x), std::min(low.y, point.y),
                                      std::min(low.z, point.z)};
                        high = Vector3{std::max(high.x, point.x), std::max(high.y, point.y),
                                       std::max(high.z, point.z)};
                    }
                    return std::max(high.x - low.x, std::max(high.y - low.y, high.z - low.z));
                }

                /** @brief Whether @p plane is already in @p planes, to within @p tolerance. */
                bool plane_already_found(const std::vector<HullPlane>& planes,
                                        const HullPlane& plane, Scalar tolerance) noexcept
                {
                    for (const HullPlane& existing : planes)
                    {
                        if (std::abs(existing.offset - plane.offset) > tolerance)
                            continue;
                        // Normals are unit length, so a dot product near one is the same
                        // facing. Comparing components would fail on a plane found from a
                        // triple whose normal differs in the last bit.
                        if (dot(existing.normal, plane.normal) > Scalar(0.9999))
                            return true;
                    }
                    return false;
                }

                /** @brief Two unit vectors spanning the plane of @p normal, right-handed with it. */
                void plane_basis(const Vector3& normal, Vector3& u, Vector3& v) noexcept
                {
                    // Cross with whichever axis the normal is least aligned with, so the
                    // result is never the cross product of two parallel vectors.
                    const Vector3 axis =
                        std::abs(normal.x) <= std::abs(normal.y) &&
                                std::abs(normal.x) <= std::abs(normal.z)
                            ? Vector3{1, 0, 0}
                            : (std::abs(normal.y) <= std::abs(normal.z) ? Vector3{0, 1, 0}
                                                                        : Vector3{0, 0, 1});
                    u = normalize(cross(axis, normal));
                    // cross(u, v) == normal, so a counter-clockwise polygon in (u, v) winds
                    // outward. Getting this pair the wrong way round inverts every hull.
                    v = cross(normal, u);
                }
            } // namespace

            bool build_convex_hull_mesh(const std::vector<Vector3>& points,
                                        Geometry::TriangleMesh& out)
            {
                out.positions.clear();
                out.indices.clear();
                out.normals.clear();
                if (points.size() < 4)
                    return false;

                const Scalar extent = point_set_extent(points);
                if (!(extent > 0))
                    return false;
                const Scalar tolerance = extent * Scalar(1e-6);

                // Pass one: every triple that has all other points on one side of it names
                // a supporting plane. Triples are the *search*; the planes are the result,
                // because a coplanar face is found by many triples and must be tiled once.
                std::vector<HullPlane> planes;
                const std::size_t count = points.size();
                for (std::size_t i = 0; i + 2 < count; ++i)
                {
                    for (std::size_t j = i + 1; j + 1 < count; ++j)
                    {
                        for (std::size_t k = j + 1; k < count; ++k)
                        {
                            const Vector3 raw =
                                cross(points[j] - points[i], points[k] - points[i]);
                            const Scalar area = std::sqrt(dot(raw, raw));
                            if (!(area > extent * extent * Scalar(1e-10)))
                                continue;   // collinear triple names no plane

                            const Vector3 normal = raw * (Scalar(1) / area);
                            const Scalar offset = dot(normal, points[i]);

                            bool any_above = false;
                            bool any_below = false;
                            for (std::size_t p = 0; p < count; ++p)
                            {
                                const Scalar signed_distance = dot(normal, points[p]) - offset;
                                if (signed_distance > tolerance)
                                    any_above = true;
                                else if (signed_distance < -tolerance)
                                    any_below = true;
                                if (any_above && any_below)
                                    break;
                            }
                            if (any_above && any_below)
                                continue;   // a cutting plane, not a supporting one

                            // Orient outward: the side nothing lies on is the outside.
                            HullPlane plane{normal, offset};
                            if (any_above)
                                plane = HullPlane{normal * Scalar(-1), -offset};
                            if (!plane_already_found(planes, plane, tolerance))
                                planes.push_back(plane);
                        }
                    }
                }

                // Fewer than four planes cannot bound a volume, which is how a coplanar or
                // collinear point set reports itself.
                if (planes.size() < 4)
                    return false;

                // Pass two: tile each plane once, by fanning its own points in angular order
                // about their centroid.
                std::vector<std::size_t> on_plane;
                std::vector<std::pair<Scalar, std::size_t>> ordered;
                std::vector<std::uint32_t> remap(count, 0xFFFFFFFFu);
                for (const HullPlane& plane : planes)
                {
                    on_plane.clear();
                    for (std::size_t p = 0; p < count; ++p)
                    {
                        if (std::abs(dot(plane.normal, points[p]) - plane.offset) <= tolerance)
                            on_plane.push_back(p);
                    }
                    if (on_plane.size() < 3)
                        continue;

                    Vector3 centroid{0, 0, 0};
                    for (const std::size_t p : on_plane)
                        centroid = centroid + points[p];
                    centroid = centroid * (Scalar(1) / Scalar(on_plane.size()));

                    Vector3 u{0, 0, 0};
                    Vector3 v{0, 0, 0};
                    plane_basis(plane.normal, u, v);

                    ordered.clear();
                    for (const std::size_t p : on_plane)
                    {
                        const Vector3 offset = points[p] - centroid;
                        ordered.emplace_back(std::atan2(dot(offset, v), dot(offset, u)), p);
                    }
                    // Ties broken by index: two points at the same angle are coincident or
                    // collinear with the centroid, and either way the tiling must not depend
                    // on the sort's stability.
                    std::sort(ordered.begin(), ordered.end(),
                              [](const std::pair<Scalar, std::size_t>& a,
                                 const std::pair<Scalar, std::size_t>& b) noexcept
                              {
                                  if (a.first != b.first)
                                      return a.first < b.first;
                                  return a.second < b.second;
                              });

                    const auto emit = [&](std::size_t point_index)
                    {
                        if (remap[point_index] == 0xFFFFFFFFu)
                        {
                            remap[point_index] = std::uint32_t(out.positions.size() / 3);
                            out.positions.push_back(float(points[point_index].x));
                            out.positions.push_back(float(points[point_index].y));
                            out.positions.push_back(float(points[point_index].z));
                        }
                        out.indices.push_back(remap[point_index]);
                    };

                    for (std::size_t fan = 1; fan + 1 < ordered.size(); ++fan)
                    {
                        emit(ordered[0].second);
                        emit(ordered[fan].second);
                        emit(ordered[fan + 1].second);
                    }
                }

                if (out.indices.size() < 12)     // fewer than four triangles bounds nothing
                {
                    out.positions.clear();
                    out.indices.clear();
                    return false;
                }
                return true;
            }

            namespace
            {
                /**
                 * @brief One part of the decomposition in progress.
                 *
                 * The hull and its concavity are cached on the part, because a part that is
                 * not chosen for splitting must not be re-measured every round: the loop
                 * runs once per piece and the measurement is the expensive half.
                 */
                struct Part
                {
                    std::vector<std::uint32_t> triangles;
                    ConvexPiece piece;
                    bool measured = false;
                    /** @brief Set when no candidate plane separates it; stops the loop spinning. */
                    bool unsplittable = false;
                };

                /** @brief The distinct vertex indices the part's triangles reference, ascending. */
                std::vector<std::uint32_t> part_vertices(const Geometry::TriangleMeshView& mesh,
                                                         const std::vector<std::uint32_t>& triangles)
                {
                    std::vector<std::uint32_t> vertices;
                    vertices.reserve(triangles.size() * 3);
                    for (const std::uint32_t t : triangles)
                    {
                        for (int corner = 0; corner < 3; ++corner)
                            vertices.push_back(mesh.indices[std::size_t(t) * 3 + std::size_t(corner)]);
                    }
                    std::sort(vertices.begin(), vertices.end());
                    vertices.erase(std::unique(vertices.begin(), vertices.end()), vertices.end());
                    return vertices;
                }

                Vector3 read_vertex(const Geometry::TriangleMeshView& mesh,
                                    std::uint32_t index) noexcept
                {
                    float position[3];
                    mesh.read_position(index, position);
                    return Vector3{Scalar(position[0]), Scalar(position[1]), Scalar(position[2])};
                }

                /**
                 * @brief At most @p budget of @p candidates, spread as widely as possible.
                 *
                 * Seeded with the six axis extremes — which no reasonable selection would
                 * omit, and which fix the piece's bounding box immediately — then extended by
                 * repeatedly taking the candidate furthest from everything already chosen.
                 * Every returned point is a real mesh vertex, which is what guarantees the
                 * selected hull sits inside the true one.
                 */
                std::vector<Vector3> select_hull_points(const Geometry::TriangleMeshView& mesh,
                                                        const std::vector<std::uint32_t>& candidates,
                                                        std::size_t budget)
                {
                    std::vector<Vector3> positions;
                    positions.reserve(candidates.size());
                    for (const std::uint32_t index : candidates)
                        positions.push_back(read_vertex(mesh, index));

                    if (positions.size() <= budget)
                        return positions;

                    std::vector<bool> chosen(positions.size(), false);
                    std::vector<Vector3> selected;
                    selected.reserve(budget);

                    const auto take = [&](std::size_t index)
                    {
                        chosen[index] = true;
                        selected.push_back(positions[index]);
                    };

                    for (int axis = 0; axis < 3; ++axis)
                    {
                        for (int direction = 0; direction < 2; ++direction)
                        {
                            const Scalar sign = direction == 0 ? Scalar(1) : Scalar(-1);
                            std::size_t best = positions.size();
                            Scalar best_value = -std::numeric_limits<Scalar>::max();
                            for (std::size_t i = 0; i < positions.size(); ++i)
                            {
                                const Vector3& p = positions[i];
                                const Scalar value =
                                    sign * (axis == 0 ? p.x : (axis == 1 ? p.y : p.z));
                                if (value > best_value)
                                {
                                    best_value = value;
                                    best = i;
                                }
                            }
                            if (best < positions.size() && !chosen[best] && selected.size() < budget)
                                take(best);
                        }
                    }

                    // Farthest-point extension. The distance to the selected set is kept
                    // incrementally, so this is O(budget * candidates) rather than cubic.
                    std::vector<Scalar> distance(positions.size(),
                                                 std::numeric_limits<Scalar>::max());
                    for (std::size_t i = 0; i < positions.size(); ++i)
                    {
                        for (const Vector3& picked : selected)
                        {
                            const Vector3 offset = positions[i] - picked;
                            distance[i] = std::min(distance[i], dot(offset, offset));
                        }
                    }

                    while (selected.size() < budget)
                    {
                        std::size_t best = positions.size();
                        Scalar best_distance = -1;
                        for (std::size_t i = 0; i < positions.size(); ++i)
                        {
                            if (chosen[i])
                                continue;
                            // Strictly greater, so ties fall to the lower index and the
                            // selection is a function of the vertex numbering.
                            if (distance[i] > best_distance)
                            {
                                best_distance = distance[i];
                                best = i;
                            }
                        }
                        if (best >= positions.size())
                            break;
                        take(best);
                        for (std::size_t i = 0; i < positions.size(); ++i)
                        {
                            const Vector3 offset = positions[i] - positions[best];
                            distance[i] = std::min(distance[i], dot(offset, offset));
                        }
                    }
                    return selected;
                }

                /** @brief Builds a part's hull, its centre, its volume and its concavity. */
                void measure_part(const Geometry::TriangleMeshView& mesh,
                                  const Geometry::MeshDistanceQuery& surface,
                                  const ConvexDecompositionOptions& options, Part& part)
                {
                    part.measured = true;
                    part.piece = ConvexPiece{};
                    if (part.triangles.empty())
                    {
                        part.unsplittable = true;
                        return;
                    }

                    const std::vector<std::uint32_t> candidates =
                        part_vertices(mesh, part.triangles);
                    const std::size_t budget =
                        options.vertex_budget > 4 ? std::size_t(options.vertex_budget) : 4;
                    const std::vector<Vector3> points =
                        select_hull_points(mesh, candidates, budget);
                    if (points.empty())
                    {
                        part.unsplittable = true;
                        return;
                    }

                    Vector3 centre{0, 0, 0};
                    for (const Vector3& point : points)
                        centre = centre + point;
                    centre = centre * (Scalar(1) / Scalar(points.size()));

                    part.piece.center = centre;
                    part.piece.vertices.reserve(points.size());
                    for (const Vector3& point : points)
                        part.piece.vertices.push_back(point - centre);

                    Geometry::TriangleMesh hull;
                    if (!build_convex_hull_mesh(points, hull))
                    {
                        // A flat or collinear piece supports collisions perfectly well — a
                        // point set is all `support()` needs — it simply encloses nothing,
                        // and there is no surface to measure a protrusion on.
                        part.piece.volume = 0;
                        part.piece.concavity = 0.0f;
                        part.unsplittable = true;
                        return;
                    }

                    const Geometry::MeshTopologyReport hull_report =
                        Geometry::analyze_mesh_topology(hull.view());
                    part.piece.volume = Scalar(hull_report.signed_volume);
                    part.piece.concavity = Geometry::max_protrusion_distance(
                        hull.view(), surface, options.accuracy_lattice_order);
                }
            } // namespace

            ConvexDecompositionReport decompose_convex(const Geometry::TriangleMeshView& mesh,
                                                       const Geometry::MeshDistanceQuery& surface,
                                                       const ConvexDecompositionOptions& options,
                                                       std::vector<ConvexPiece>& out)
            {
                out.clear();
                ConvexDecompositionReport report;
                if (!mesh.has_triangles() || !surface.ready())
                    return report;

                const std::size_t max_pieces =
                    options.max_pieces > 1 ? std::size_t(options.max_pieces) : 1;

                std::vector<Part> parts(1);
                parts[0].triangles.reserve(mesh.triangle_count());
                for (std::size_t t = 0; t < mesh.triangle_count(); ++t)
                {
                    // Filtered once, here, so that every later pass may read a triangle's
                    // vertices without checking: this is the module's only entry point, and
                    // an out-of-range index reaching `read_position` is a walk off the end.
                    const std::uint32_t i0 = mesh.indices[t * 3 + 0];
                    const std::uint32_t i1 = mesh.indices[t * 3 + 1];
                    const std::uint32_t i2 = mesh.indices[t * 3 + 2];
                    if (i0 >= mesh.vertex_count || i1 >= mesh.vertex_count ||
                        i2 >= mesh.vertex_count)
                        continue;
                    parts[0].triangles.push_back(std::uint32_t(t));
                }
                if (parts[0].triangles.empty())
                    return report;
                measure_part(mesh, surface, options, parts[0]);

                while (parts.size() < max_pieces)
                {
                    // The part that departs furthest from the source surface is the one worth
                    // splitting, because that departure is exactly what the cook reports.
                    std::size_t worst = parts.size();
                    float worst_concavity = 0.0f;
                    for (std::size_t i = 0; i < parts.size(); ++i)
                    {
                        if (parts[i].unsplittable || parts[i].triangles.size() < 2)
                            continue;
                        if (parts[i].piece.concavity > worst_concavity)
                        {
                            worst_concavity = parts[i].piece.concavity;
                            worst = i;
                        }
                    }
                    if (worst >= parts.size())
                        break;
                    if (options.concavity_tolerance > 0.0f &&
                        worst_concavity <= options.concavity_tolerance)
                        break;

                    // Three axis-aligned planes through the part's triangle centroid. The
                    // winner is the one whose worse child is best, since the reported error
                    // is a maximum and improving the better child does not move it.
                    Vector3 centroid{0, 0, 0};
                    for (const std::uint32_t t : parts[worst].triangles)
                    {
                        Vector3 sum{0, 0, 0};
                        for (int corner = 0; corner < 3; ++corner)
                        {
                            sum = sum + read_vertex(mesh, mesh.indices[std::size_t(t) * 3 +
                                                                       std::size_t(corner)]);
                        }
                        centroid = centroid + sum * (Scalar(1) / Scalar(3));
                    }
                    centroid = centroid * (Scalar(1) / Scalar(parts[worst].triangles.size()));

                    Part best_low;
                    Part best_high;
                    float best_score = std::numeric_limits<float>::max();
                    bool found_split = false;

                    for (int axis = 0; axis < 3; ++axis)
                    {
                        const Scalar cut = axis == 0 ? centroid.x
                                                     : (axis == 1 ? centroid.y : centroid.z);
                        Part low;
                        Part high;
                        for (const std::uint32_t t : parts[worst].triangles)
                        {
                            Vector3 sum{0, 0, 0};
                            for (int corner = 0; corner < 3; ++corner)
                            {
                                sum = sum + read_vertex(mesh, mesh.indices[std::size_t(t) * 3 +
                                                                           std::size_t(corner)]);
                            }
                            const Vector3 triangle_centre = sum * (Scalar(1) / Scalar(3));
                            const Scalar value =
                                axis == 0 ? triangle_centre.x
                                          : (axis == 1 ? triangle_centre.y : triangle_centre.z);
                            if (value < cut)
                                low.triangles.push_back(t);
                            else
                                high.triangles.push_back(t);
                        }
                        if (low.triangles.empty() || high.triangles.empty())
                            continue;   // this axis does not separate the part at all

                        measure_part(mesh, surface, options, low);
                        measure_part(mesh, surface, options, high);
                        const float score = std::max(low.piece.concavity, high.piece.concavity);
                        if (!found_split || score < best_score)
                        {
                            best_score = score;
                            best_low = std::move(low);
                            best_high = std::move(high);
                            found_split = true;
                        }
                    }

                    if (!found_split)
                    {
                        parts[worst].unsplittable = true;
                        continue;
                    }
                    // A split that makes the worst child no better than the parent has bought
                    // nothing but a second shape to collide against, so the part is left whole.
                    if (best_score >= worst_concavity)
                    {
                        parts[worst].unsplittable = true;
                        continue;
                    }

                    parts[worst] = std::move(best_low);
                    parts.push_back(std::move(best_high));
                }

                for (Part& part : parts)
                {
                    if (!part.measured)
                        measure_part(mesh, surface, options, part);
                    if (part.piece.vertices.empty())
                        continue;
                    report.largest_piece_vertex_count =
                        std::max(report.largest_piece_vertex_count,
                                 std::uint32_t(part.piece.vertices.size()));
                    report.worst_concavity =
                        std::max(report.worst_concavity, part.piece.concavity);
                    report.summed_volume += part.piece.volume;
                    out.push_back(std::move(part.piece));
                }
                report.piece_count = std::uint32_t(out.size());
                report.exhausted_budget = parts.size() >= max_pieces;
                return report;
            }
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
