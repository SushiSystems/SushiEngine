/**************************************************************************/
/* mesh_utilities.cpp                                                     */
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

#include <SushiEngine/geometry/mesh_utilities.hpp>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <utility>
#include <vector>

namespace SushiEngine
{
    namespace Geometry
    {
        namespace
        {
            /** @brief One vertex position, packed, for the internal passes. */
            struct Point
            {
                float x;
                float y;
                float z;
            };

            Point subtract(const Point& a, const Point& b) noexcept
            {
                return Point{a.x - b.x, a.y - b.y, a.z - b.z};
            }

            Point cross(const Point& a, const Point& b) noexcept
            {
                return Point{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                             a.x * b.y - a.y * b.x};
            }

            float dot(const Point& a, const Point& b) noexcept
            {
                return a.x * b.x + a.y * b.y + a.z * b.z;
            }

            float length(const Point& a) noexcept { return std::sqrt(dot(a, a)); }

            /** @brief The mesh's positions, tightly packed, whatever stride they arrived at. */
            std::vector<Point> gather_positions(const TriangleMeshView& mesh)
            {
                std::vector<Point> points(mesh.vertex_count);
                for (std::size_t i = 0; i < mesh.vertex_count; ++i)
                {
                    float position[3];
                    mesh.read_position(i, position);
                    points[i] = Point{position[0], position[1], position[2]};
                }
                return points;
            }

            /**
             * @brief One directed edge of one triangle, ready to be sorted into runs.
             *
             * Sorting rather than hashing, because every count derived from this table
             * ends up in a cook report and a report that depends on a hash container's
             * iteration order is a report that changes between builds for no reason
             * (§12.1's rule, applied to the import side).
             */
            struct EdgeRecord
            {
                std::uint64_t key;      // low vertex in the high word, high vertex in the low
                std::uint32_t triangle;
                bool forward;           // the triangle walks the edge low -> high
            };

            std::uint64_t edge_key(std::uint32_t a, std::uint32_t b) noexcept
            {
                const std::uint32_t low = a < b ? a : b;
                const std::uint32_t high = a < b ? b : a;
                return (std::uint64_t(low) << 32) | std::uint64_t(high);
            }

            bool edge_record_less(const EdgeRecord& a, const EdgeRecord& b) noexcept
            {
                if (a.key != b.key)
                    return a.key < b.key;
                return a.triangle < b.triangle;
            }

            /**
             * @brief A triangle that survived validation, by its three current corners.
             *
             * Held separately from the index buffer because orientation flips them and
             * the edge table has to be read against their *current* winding, not the
             * one recorded when the table was built.
             */
            struct Face
            {
                std::uint32_t vertex[3];
            };

            /** @brief Twice a triangle's area — zero exactly when it is degenerate. */
            float double_area(const Point& a, const Point& b, const Point& c) noexcept
            {
                return length(cross(subtract(b, a), subtract(c, a)));
            }

            /** @brief Six times the volume of the tetrahedron from the origin to a triangle. */
            float triangle_volume_times_six(const Point& a, const Point& b,
                                            const Point& c) noexcept
            {
                return dot(a, cross(b, c));
            }

            /** @brief A triangle's three vertex indices, sorted, as one comparable key. */
            struct SortedFaceKey
            {
                std::uint32_t vertex[3];
                std::uint32_t triangle;
            };

            bool sorted_face_less(const SortedFaceKey& a, const SortedFaceKey& b) noexcept
            {
                for (int i = 0; i < 3; ++i)
                {
                    if (a.vertex[i] != b.vertex[i])
                        return a.vertex[i] < b.vertex[i];
                }
                return a.triangle < b.triangle;
            }

            SortedFaceKey make_sorted_face(const std::uint32_t vertex[3],
                                           std::uint32_t triangle) noexcept
            {
                SortedFaceKey key{{vertex[0], vertex[1], vertex[2]}, triangle};
                std::sort(key.vertex, key.vertex + 3);
                return key;
            }

            bool same_corners(const SortedFaceKey& a, const SortedFaceKey& b) noexcept
            {
                return a.vertex[0] == b.vertex[0] && a.vertex[1] == b.vertex[1] &&
                       a.vertex[2] == b.vertex[2];
            }

            /** @brief A hash-grid cell key; collisions cost a distance test, never a miss. */
            std::uint64_t cell_key(std::int64_t x, std::int64_t y, std::int64_t z) noexcept
            {
                const std::uint64_t hx = std::uint64_t(x) * 0x9E3779B97F4A7C15ull;
                const std::uint64_t hy = std::uint64_t(y) * 0xC2B2AE3D27D4EB4Full;
                const std::uint64_t hz = std::uint64_t(z) * 0x165667B19E3779F9ull;
                return hx ^ hy ^ hz;
            }

            /** @brief Union-find over triangles, for the connected-component count. */
            class DisjointSet
            {
            public:
                explicit DisjointSet(std::size_t count) : parent_(count)
                {
                    for (std::size_t i = 0; i < count; ++i)
                        parent_[i] = std::uint32_t(i);
                }

                std::uint32_t find(std::uint32_t index) noexcept
                {
                    while (parent_[index] != index)
                    {
                        parent_[index] = parent_[parent_[index]];
                        index = parent_[index];
                    }
                    return index;
                }

                void merge(std::uint32_t a, std::uint32_t b) noexcept
                {
                    const std::uint32_t root_a = find(a);
                    const std::uint32_t root_b = find(b);
                    if (root_a == root_b)
                        return;
                    // Lower root wins, so the labelling is a function of the input order.
                    if (root_a < root_b)
                        parent_[root_b] = root_a;
                    else
                        parent_[root_a] = root_b;
                }

            private:
                std::vector<std::uint32_t> parent_;
            };

            /**
             * @brief Splits a mesh into the valid faces and the counts of what was rejected.
             *
             * Shared by the analysis and the repair so the two cannot disagree about what
             * "degenerate" means — which they would, given the chance, and the symptom
             * would be a report claiming a mesh the cooker had already dropped triangles
             * from was clean.
             */
            struct FaceExtraction
            {
                std::vector<Face> faces;
                std::size_t out_of_range = 0;
                std::size_t degenerate = 0;
                std::size_t duplicate = 0;
            };

            FaceExtraction extract_faces(const std::uint32_t* indices, std::size_t index_count,
                                         const std::vector<Point>& points, bool drop_duplicates)
            {
                FaceExtraction extraction;
                const std::size_t triangle_count = index_count / 3;
                extraction.faces.reserve(triangle_count);

                const std::uint32_t vertex_count = std::uint32_t(points.size());
                for (std::size_t t = 0; t < triangle_count; ++t)
                {
                    const std::uint32_t i0 = indices[t * 3 + 0];
                    const std::uint32_t i1 = indices[t * 3 + 1];
                    const std::uint32_t i2 = indices[t * 3 + 2];
                    if (i0 >= vertex_count || i1 >= vertex_count || i2 >= vertex_count)
                    {
                        ++extraction.out_of_range;
                        continue;
                    }
                    if (i0 == i1 || i1 == i2 || i0 == i2 ||
                        double_area(points[i0], points[i1], points[i2]) <= 0.0f)
                    {
                        ++extraction.degenerate;
                        continue;
                    }
                    extraction.faces.push_back(Face{{i0, i1, i2}});
                }

                if (extraction.faces.empty())
                    return extraction;

                std::vector<SortedFaceKey> keys;
                keys.reserve(extraction.faces.size());
                for (std::size_t f = 0; f < extraction.faces.size(); ++f)
                    keys.push_back(make_sorted_face(extraction.faces[f].vertex, std::uint32_t(f)));
                std::sort(keys.begin(), keys.end(), sorted_face_less);

                std::vector<bool> duplicate(extraction.faces.size(), false);
                for (std::size_t i = 1; i < keys.size(); ++i)
                {
                    if (!same_corners(keys[i - 1], keys[i]))
                        continue;
                    ++extraction.duplicate;
                    // The lowest-numbered face of a coincident set survives, so which copy
                    // is kept does not depend on the sort's tie-breaking.
                    duplicate[keys[i].triangle] = true;
                }

                if (!drop_duplicates || extraction.duplicate == 0)
                    return extraction;

                std::vector<Face> kept;
                kept.reserve(extraction.faces.size() - extraction.duplicate);
                for (std::size_t f = 0; f < extraction.faces.size(); ++f)
                {
                    if (!duplicate[f])
                        kept.push_back(extraction.faces[f]);
                }
                extraction.faces = std::move(kept);
                return extraction;
            }

            /** @brief Every valid face's three directed edges, sorted into runs. */
            std::vector<EdgeRecord> build_edge_table(const std::vector<Face>& faces)
            {
                std::vector<EdgeRecord> edges;
                edges.reserve(faces.size() * 3);
                for (std::size_t f = 0; f < faces.size(); ++f)
                {
                    for (int e = 0; e < 3; ++e)
                    {
                        const std::uint32_t a = faces[f].vertex[e];
                        const std::uint32_t b = faces[f].vertex[(e + 1) % 3];
                        EdgeRecord record{};
                        record.key = edge_key(a, b);
                        record.triangle = std::uint32_t(f);
                        record.forward = a < b;
                        edges.push_back(record);
                    }
                }
                std::sort(edges.begin(), edges.end(), edge_record_less);
                return edges;
            }

            /** @brief Whether @p face walks the edge @p low to @p high in that direction. */
            bool face_walks_forward(const Face& face, std::uint32_t low,
                                    std::uint32_t high) noexcept
            {
                for (int e = 0; e < 3; ++e)
                {
                    const std::uint32_t a = face.vertex[e];
                    const std::uint32_t b = face.vertex[(e + 1) % 3];
                    if (a == low && b == high)
                        return true;
                    if (a == high && b == low)
                        return false;
                }
                // Unreachable for an edge the face owns; false keeps the caller total.
                return false;
            }
        } // namespace

        MeshTopologyReport analyze_mesh_topology(const TriangleMeshView& mesh)
        {
            MeshTopologyReport report;
            if (mesh.positions == nullptr || mesh.vertex_count == 0)
                return report;

            const std::vector<Point> points = gather_positions(mesh);
            report.vertex_count = mesh.vertex_count;
            report.triangle_count = mesh.indices == nullptr ? 0 : mesh.triangle_count();
            if (report.triangle_count == 0)
            {
                report.unreferenced_vertices = mesh.vertex_count;
                return report;
            }

            // Duplicates are counted but kept: a doubled surface really does have
            // non-manifold edges, and an analysis that quietly discarded one copy would
            // report the mesh as manifold when the file is not.
            const FaceExtraction extraction =
                extract_faces(mesh.indices, mesh.index_count, points, false);
            report.out_of_range_triangles = extraction.out_of_range;
            report.degenerate_triangles = extraction.degenerate;
            report.duplicate_triangles = extraction.duplicate;

            std::vector<bool> referenced(mesh.vertex_count, false);
            double area = 0.0;
            double volume = 0.0;
            for (const Face& face : extraction.faces)
            {
                const Point& a = points[face.vertex[0]];
                const Point& b = points[face.vertex[1]];
                const Point& c = points[face.vertex[2]];
                area += 0.5 * double(double_area(a, b, c));
                volume += double(triangle_volume_times_six(a, b, c)) / 6.0;
                for (int i = 0; i < 3; ++i)
                    referenced[face.vertex[i]] = true;
            }
            report.surface_area = float(area);
            report.signed_volume = float(volume);
            report.unreferenced_vertices =
                std::size_t(std::count(referenced.begin(), referenced.end(), false));

            const std::vector<EdgeRecord> edges = build_edge_table(extraction.faces);
            DisjointSet components(extraction.faces.size());
            std::size_t run_start = 0;
            while (run_start < edges.size())
            {
                std::size_t run_end = run_start + 1;
                while (run_end < edges.size() && edges[run_end].key == edges[run_start].key)
                    ++run_end;

                const std::size_t incident = run_end - run_start;
                if (incident == 1)
                    ++report.boundary_edges;
                else if (incident > 2)
                    ++report.non_manifold_edges;
                else if (edges[run_start].forward == edges[run_start + 1].forward)
                    ++report.inconsistent_edges;

                for (std::size_t i = run_start + 1; i < run_end; ++i)
                    components.merge(edges[run_start].triangle, edges[i].triangle);
                run_start = run_end;
            }

            std::vector<bool> seen_root(extraction.faces.size(), false);
            for (std::size_t f = 0; f < extraction.faces.size(); ++f)
            {
                const std::uint32_t root = components.find(std::uint32_t(f));
                if (!seen_root[root])
                {
                    seen_root[root] = true;
                    ++report.connected_components;
                }
            }
            return report;
        }

        MeshRepairReport repair_mesh(const TriangleMeshView& source,
                                     const MeshRepairOptions& options, TriangleMesh& out)
        {
            out.positions.clear();
            out.indices.clear();
            out.normals.clear();

            MeshRepairReport report;
            report.before = analyze_mesh_topology(source);
            if (source.positions == nullptr || source.vertex_count == 0)
                return report;

            const std::vector<Point> points = gather_positions(source);

            // Stage one: weld. Which triangles share an edge is undecidable until
            // coincident corners are one vertex, so nothing after this can run first.
            std::vector<std::uint32_t> remap(points.size(), 0);
            std::vector<Point> welded;
            welded.reserve(points.size());
            if (options.weld_tolerance > 0.0f)
            {
                const float cell_size = options.weld_tolerance;
                const float tolerance_squared = options.weld_tolerance * options.weld_tolerance;
                std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> grid;
                grid.reserve(points.size() * 2);

                for (std::size_t i = 0; i < points.size(); ++i)
                {
                    const Point& p = points[i];
                    const std::int64_t cx = std::int64_t(std::floor(p.x / cell_size));
                    const std::int64_t cy = std::int64_t(std::floor(p.y / cell_size));
                    const std::int64_t cz = std::int64_t(std::floor(p.z / cell_size));

                    // The lowest-numbered representative within tolerance wins, and every
                    // one of the twenty-seven neighbouring cells is searched before the
                    // decision, so the answer is the tolerance's and not a cell boundary's.
                    std::uint32_t best = 0xFFFFFFFFu;
                    for (std::int64_t dz = -1; dz <= 1; ++dz)
                    {
                        for (std::int64_t dy = -1; dy <= 1; ++dy)
                        {
                            for (std::int64_t dx = -1; dx <= 1; ++dx)
                            {
                                const auto found =
                                    grid.find(cell_key(cx + dx, cy + dy, cz + dz));
                                if (found == grid.end())
                                    continue;
                                for (const std::uint32_t candidate : found->second)
                                {
                                    if (candidate >= best)
                                        continue;
                                    const Point offset = subtract(p, welded[candidate]);
                                    if (dot(offset, offset) <= tolerance_squared)
                                        best = candidate;
                                }
                            }
                        }
                    }

                    if (best != 0xFFFFFFFFu)
                    {
                        remap[i] = best;
                        continue;
                    }
                    const std::uint32_t fresh = std::uint32_t(welded.size());
                    welded.push_back(p);
                    remap[i] = fresh;
                    grid[cell_key(cx, cy, cz)].push_back(fresh);
                }
                report.welded_vertices = points.size() - welded.size();
            }
            else
            {
                welded = points;
                for (std::size_t i = 0; i < points.size(); ++i)
                    remap[i] = std::uint32_t(i);
            }

            // Stage two: drop what welding made or the source brought. Welding *creates*
            // degenerate triangles — a sliver whose two ends weld together is a line — so
            // this cannot run before it.
            std::vector<std::uint32_t> welded_indices;
            welded_indices.reserve(source.index_count);
            const std::size_t source_triangles =
                source.indices == nullptr ? 0 : source.triangle_count();
            for (std::size_t t = 0; t < source_triangles; ++t)
            {
                bool valid = true;
                std::uint32_t corner[3];
                for (int i = 0; i < 3; ++i)
                {
                    const std::uint32_t index = source.indices[t * 3 + i];
                    if (index >= points.size())
                    {
                        valid = false;
                        break;
                    }
                    corner[i] = remap[index];
                }
                if (!valid)
                    continue;
                welded_indices.push_back(corner[0]);
                welded_indices.push_back(corner[1]);
                welded_indices.push_back(corner[2]);
            }

            FaceExtraction extraction =
                extract_faces(welded_indices.data(), welded_indices.size(), welded, true);
            std::vector<Face> faces = std::move(extraction.faces);
            report.removed_triangles = source_triangles - faces.size();

            // Stage three: orientation. Propagated across manifold edges only — an edge
            // with three incident triangles has no answer to "which side is out", and
            // guessing there would flip a whole shell on the strength of a modelling
            // mistake.
            if (options.orient_consistently && !faces.empty())
            {
                const std::vector<EdgeRecord> edges = build_edge_table(faces);
                std::vector<std::vector<std::uint32_t>> neighbours(faces.size());
                std::vector<std::vector<std::uint64_t>> shared(faces.size());

                std::size_t run_start = 0;
                std::vector<std::uint32_t> boundary_faces;
                while (run_start < edges.size())
                {
                    std::size_t run_end = run_start + 1;
                    while (run_end < edges.size() && edges[run_end].key == edges[run_start].key)
                        ++run_end;
                    if (run_end - run_start == 1)
                    {
                        boundary_faces.push_back(edges[run_start].triangle);
                    }
                    else if (run_end - run_start == 2)
                    {
                        const std::uint32_t t0 = edges[run_start].triangle;
                        const std::uint32_t t1 = edges[run_start + 1].triangle;
                        neighbours[t0].push_back(t1);
                        shared[t0].push_back(edges[run_start].key);
                        neighbours[t1].push_back(t0);
                        shared[t1].push_back(edges[run_start].key);
                    }
                    run_start = run_end;
                }

                std::vector<std::uint32_t> component(faces.size(), 0xFFFFFFFFu);
                std::vector<std::vector<std::uint32_t>> component_faces;
                std::vector<std::uint32_t> queue;
                for (std::size_t seed = 0; seed < faces.size(); ++seed)
                {
                    if (component[seed] != 0xFFFFFFFFu)
                        continue;
                    const std::uint32_t id = std::uint32_t(component_faces.size());
                    component_faces.emplace_back();
                    component[seed] = id;
                    queue.clear();
                    queue.push_back(std::uint32_t(seed));

                    for (std::size_t head = 0; head < queue.size(); ++head)
                    {
                        const std::uint32_t current = queue[head];
                        component_faces[id].push_back(current);
                        for (std::size_t n = 0; n < neighbours[current].size(); ++n)
                        {
                            const std::uint32_t next = neighbours[current][n];
                            if (component[next] != 0xFFFFFFFFu)
                                continue;
                            const std::uint64_t key = shared[current][n];
                            const std::uint32_t low = std::uint32_t(key >> 32);
                            const std::uint32_t high = std::uint32_t(key & 0xFFFFFFFFu);
                            // Two triangles agree on the outside only when one walks the
                            // shared edge low->high and the other walks it high->low.
                            if (face_walks_forward(faces[current], low, high) ==
                                face_walks_forward(faces[next], low, high))
                            {
                                std::swap(faces[next].vertex[1], faces[next].vertex[2]);
                                ++report.reoriented_triangles;
                            }
                            component[next] = id;
                            queue.push_back(next);
                        }
                    }
                }

                std::vector<bool> component_open(component_faces.size(), false);
                for (const std::uint32_t face : boundary_faces)
                    component_open[component[face]] = true;

                if (options.orient_outward)
                {
                    for (std::size_t id = 0; id < component_faces.size(); ++id)
                    {
                        // An open shell encloses nothing, so its volume's sign is not a
                        // statement about which way is out.
                        if (component_open[id])
                            continue;
                        double volume = 0.0;
                        for (const std::uint32_t f : component_faces[id])
                        {
                            volume += double(triangle_volume_times_six(welded[faces[f].vertex[0]],
                                                                       welded[faces[f].vertex[1]],
                                                                       welded[faces[f].vertex[2]]));
                        }
                        if (volume >= 0.0)
                            continue;
                        for (const std::uint32_t f : component_faces[id])
                            std::swap(faces[f].vertex[1], faces[f].vertex[2]);
                        ++report.reversed_components;
                    }
                }
            }

            // Stage four: compaction, last because it is the only stage that renumbers.
            if (options.drop_unreferenced_vertices)
            {
                std::vector<std::uint32_t> compact(welded.size(), 0xFFFFFFFFu);
                out.positions.reserve(welded.size() * 3);
                for (const Face& face : faces)
                {
                    for (int i = 0; i < 3; ++i)
                    {
                        const std::uint32_t index = face.vertex[i];
                        if (compact[index] != 0xFFFFFFFFu)
                            continue;
                        compact[index] = std::uint32_t(out.positions.size() / 3);
                        out.positions.push_back(welded[index].x);
                        out.positions.push_back(welded[index].y);
                        out.positions.push_back(welded[index].z);
                    }
                }
                out.indices.reserve(faces.size() * 3);
                for (const Face& face : faces)
                {
                    for (int i = 0; i < 3; ++i)
                        out.indices.push_back(compact[face.vertex[i]]);
                }
                report.removed_vertices = welded.size() - out.positions.size() / 3;
            }
            else
            {
                out.positions.reserve(welded.size() * 3);
                for (const Point& p : welded)
                {
                    out.positions.push_back(p.x);
                    out.positions.push_back(p.y);
                    out.positions.push_back(p.z);
                }
                out.indices.reserve(faces.size() * 3);
                for (const Face& face : faces)
                {
                    for (int i = 0; i < 3; ++i)
                        out.indices.push_back(face.vertex[i]);
                }
            }

            report.after = analyze_mesh_topology(out.view());
            return report;
        }

        void closest_point_on_triangle(const float point[3], const float a[3], const float b[3],
                                       const float c[3], float out[3]) noexcept
        {
            const Point pa{a[0], a[1], a[2]};
            const Point pb{b[0], b[1], b[2]};
            const Point pc{c[0], c[1], c[2]};
            const Point pp{point[0], point[1], point[2]};

            const Point ab = subtract(pb, pa);
            const Point ac = subtract(pc, pa);
            const Point ap = subtract(pp, pa);

            const float d1 = dot(ab, ap);
            const float d2 = dot(ac, ap);
            if (d1 <= 0.0f && d2 <= 0.0f)
            {
                out[0] = pa.x;
                out[1] = pa.y;
                out[2] = pa.z;
                return;     // vertex region A
            }

            const Point bp = subtract(pp, pb);
            const float d3 = dot(ab, bp);
            const float d4 = dot(ac, bp);
            if (d3 >= 0.0f && d4 <= d3)
            {
                out[0] = pb.x;
                out[1] = pb.y;
                out[2] = pb.z;
                return;     // vertex region B
            }

            // Each edge region divides by the length the region is parameterized along,
            // and a triangle with two coincident corners makes that length zero while
            // still satisfying the region test. The guards are not defensive padding:
            // a collapsed triangle is the normal output of welding a sliver, and a
            // not-a-number leaving here would be carried silently through a whole cook.
            const float vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
            {
                const float span = d1 - d3;
                const float v = span > 0.0f ? d1 / span : 0.0f;
                out[0] = pa.x + ab.x * v;
                out[1] = pa.y + ab.y * v;
                out[2] = pa.z + ab.z * v;
                return;     // edge region AB
            }

            const Point cp = subtract(pp, pc);
            const float d5 = dot(ab, cp);
            const float d6 = dot(ac, cp);
            if (d6 >= 0.0f && d5 <= d6)
            {
                out[0] = pc.x;
                out[1] = pc.y;
                out[2] = pc.z;
                return;     // vertex region C
            }

            const float vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
            {
                const float span = d2 - d6;
                const float w = span > 0.0f ? d2 / span : 0.0f;
                out[0] = pa.x + ac.x * w;
                out[1] = pa.y + ac.y * w;
                out[2] = pa.z + ac.z * w;
                return;     // edge region AC
            }

            const float va = d3 * d6 - d5 * d4;
            if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
            {
                const float span = (d4 - d3) + (d5 - d6);
                const float w = span > 0.0f ? (d4 - d3) / span : 0.0f;
                const Point bc = subtract(pc, pb);
                out[0] = pb.x + bc.x * w;
                out[1] = pb.y + bc.y * w;
                out[2] = pb.z + bc.z * w;
                return;     // edge region BC
            }

            const float denominator = va + vb + vc;
            // A denominator of zero means a degenerate triangle, which the four region
            // tests above cannot reach for a triangle with area; guard it anyway, because
            // a caller reading a not-a-number here would carry it into a whole cook.
            if (!(denominator > 0.0f))
            {
                out[0] = pa.x;
                out[1] = pa.y;
                out[2] = pa.z;
                return;
            }
            const float v = vb / denominator;
            const float w = vc / denominator;
            out[0] = pa.x + ab.x * v + ac.x * w;
            out[1] = pa.y + ab.y * v + ac.y * w;
            out[2] = pa.z + ab.z * v + ac.z * w;
        }

        float tetrahedron_signed_volume_times_six(const float a[3], const float b[3],
                                                  const float c[3], const float d[3]) noexcept
        {
            const Point pa{a[0], a[1], a[2]};
            const Point ab = subtract(Point{b[0], b[1], b[2]}, pa);
            const Point ac = subtract(Point{c[0], c[1], c[2]}, pa);
            const Point ad = subtract(Point{d[0], d[1], d[2]}, pa);
            return dot(cross(ab, ac), ad);
        }

        bool tetrahedron_barycentric(const float point[3], const float a[3], const float b[3],
                                     const float c[3], const float d[3],
                                     float weights[4]) noexcept
        {
            const float total = tetrahedron_signed_volume_times_six(a, b, c, d);
            if (total == 0.0f)
            {
                weights[0] = 0.25f;
                weights[1] = 0.25f;
                weights[2] = 0.25f;
                weights[3] = 0.25f;
                return false;
            }

            // Each coordinate is the signed volume of the tetrahedron formed by replacing
            // that vertex with the query point, over the whole. Nothing clamps: a
            // negative coordinate is the extrapolation §8.3 stage 6 relies on to keep a
            // thin feature bound to its nearest element.
            const float inverse = 1.0f / total;
            weights[0] = tetrahedron_signed_volume_times_six(point, b, c, d) * inverse;
            weights[1] = tetrahedron_signed_volume_times_six(a, point, c, d) * inverse;
            weights[2] = tetrahedron_signed_volume_times_six(a, b, point, d) * inverse;
            weights[3] = tetrahedron_signed_volume_times_six(a, b, c, point) * inverse;
            return true;
        }
    } // namespace Geometry
} // namespace SushiEngine
