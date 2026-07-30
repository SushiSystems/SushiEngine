/**************************************************************************/
/* tetrahedral_mesh.cpp                                                   */
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

#include <SushiEngine/physics/cooking/tetrahedral_mesh.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

#include <SushiEngine/geometry/mesh_utilities.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            namespace
            {
                /** @brief Marks a grid cell whose state is not yet decided. */
                constexpr std::uint8_t CELL_UNKNOWN = 0;

                /** @brief The surface passes through this cell. */
                constexpr std::uint8_t CELL_SURFACE = 1;

                /** @brief The exterior flood fill reached this cell. */
                constexpr std::uint8_t CELL_OUTSIDE = 2;

                /** @brief No vertex assigned to this lattice corner yet. */
                constexpr std::uint32_t NO_VERTEX = 0xFFFFFFFFu;

                /**
                 * @brief Kuhn's six tetrahedra of a unit cell, by corner index.
                 *
                 * Corner `c` is `i + 2j + 4k`. Each tetrahedron is the monotone path from
                 * corner 0 to corner 7 following one of the six axis orderings, so all six
                 * share the main diagonal and together tile the cell exactly once.
                 *
                 * Conforming across a shared face falls out of that construction rather than
                 * out of a parity rule: on the cell's `+x` face the split diagonal is 1-7,
                 * and the neighbour computes 0-6 on its own `-x` face, which is the same two
                 * vertices. The same holds on `+y` and `+z`. That is the property body-centred
                 * cubic needs alternation to obtain, and the reason this lattice is used here.
                 */
                constexpr int KUHN_TETRAHEDRA[6][4] = {{0, 1, 3, 7}, {0, 1, 5, 7}, {0, 2, 3, 7},
                                                       {0, 2, 6, 7}, {0, 4, 5, 7}, {0, 4, 6, 7}};

                /** @brief Six times the signed volume of a tetrahedron. */
                Scalar signed_volume_times_six(const Vector3& a, const Vector3& b,
                                               const Vector3& c, const Vector3& d) noexcept
                {
                    return dot(cross(b - a, c - a), d - a);
                }

                /** @brief A lattice corner's index into the corner grid, which is one larger. */
                std::size_t corner_index(const std::int32_t corner_grid[3], std::int32_t x,
                                         std::int32_t y, std::int32_t z) noexcept
                {
                    return std::size_t(x) +
                           std::size_t(corner_grid[0]) *
                               (std::size_t(y) + std::size_t(corner_grid[1]) * std::size_t(z));
                }

                /** @brief A cell's index into the cell grid. */
                std::size_t cell_index(const std::int32_t grid[3], std::int32_t x, std::int32_t y,
                                       std::int32_t z) noexcept
                {
                    return std::size_t(x) +
                           std::size_t(grid[0]) *
                               (std::size_t(y) + std::size_t(grid[1]) * std::size_t(z));
                }

                /**
                 * @brief Marks every cell the surface passes through.
                 *
                 * Per triangle over the cells its own bounds touch, rather than per cell over
                 * every triangle: the surface occupies a vanishing fraction of the grid, and
                 * sweeping the whole grid against a hierarchy would be a query per voxel —
                 * sixteen million of them at the fidelity dial's top resolution.
                 *
                 * The test is per axis and not a distance: the closest point of the triangle
                 * to the cell centre lies inside the cell exactly when the triangle meets the
                 * cell, so comparing each component against half the cell is an *exact*
                 * intersection test. A circumradius test instead marks a shell of cells the
                 * surface never enters, and since a marked cell is treated as part of the
                 * body, that shell makes the simulation mesh a full cell fatter than the
                 * source in every direction — measured at nearly twice the volume for a box.
                 *
                 * The tie is included **deliberately and with a tolerance**, which matters for
                 * the common case of a mesh modelled on round numbers. A face lying on a cell
                 * boundary is equidistant from the centres either side of it, so an exact
                 * comparison decides by whichever way the grid arithmetic happened to round —
                 * and it rounds differently for different faces of the same box, which showed
                 * up as one wall of a cube marking both its layers and the opposite wall
                 * marking one. Widening by a hair makes both layers marked for every face, and
                 * the outer layer is then removed by @ref classify_surface_cells rather than by
                 * luck.
                 */
                void rasterize_surface(const Geometry::TriangleMeshView& mesh,
                                       const std::int32_t grid[3], const Vector3& origin,
                                       Scalar cell_size, std::vector<std::uint8_t>& cells)
                {
                    const Scalar half = cell_size * Scalar(0.5) * Scalar(1.0 + 1e-6);

                    for (std::size_t t = 0; t < mesh.triangle_count(); ++t)
                    {
                        const std::uint32_t i0 = mesh.indices[t * 3 + 0];
                        const std::uint32_t i1 = mesh.indices[t * 3 + 1];
                        const std::uint32_t i2 = mesh.indices[t * 3 + 2];
                        if (i0 >= mesh.vertex_count || i1 >= mesh.vertex_count ||
                            i2 >= mesh.vertex_count)
                            continue;

                        float corner[3][3];
                        mesh.read_position(i0, corner[0]);
                        mesh.read_position(i1, corner[1]);
                        mesh.read_position(i2, corner[2]);

                        std::int32_t low[3];
                        std::int32_t high[3];
                        for (int axis = 0; axis < 3; ++axis)
                        {
                            const Scalar base = axis == 0 ? origin.x
                                                          : (axis == 1 ? origin.y : origin.z);
                            Scalar minimum = Scalar(corner[0][axis]);
                            Scalar maximum = minimum;
                            for (int v = 1; v < 3; ++v)
                            {
                                minimum = std::min(minimum, Scalar(corner[v][axis]));
                                maximum = std::max(maximum, Scalar(corner[v][axis]));
                            }
                            low[axis] = std::int32_t(std::floor((minimum - base) / cell_size)) - 1;
                            high[axis] = std::int32_t(std::floor((maximum - base) / cell_size)) + 1;
                            low[axis] = std::max(low[axis], 0);
                            high[axis] = std::min(high[axis], grid[axis] - 1);
                        }

                        for (std::int32_t z = low[2]; z <= high[2]; ++z)
                        {
                            for (std::int32_t y = low[1]; y <= high[1]; ++y)
                            {
                                for (std::int32_t x = low[0]; x <= high[0]; ++x)
                                {
                                    const float centre[3] = {
                                        float(origin.x + (Scalar(x) + Scalar(0.5)) * cell_size),
                                        float(origin.y + (Scalar(y) + Scalar(0.5)) * cell_size),
                                        float(origin.z + (Scalar(z) + Scalar(0.5)) * cell_size)};
                                    float nearest[3];
                                    Geometry::closest_point_on_triangle(centre, corner[0],
                                                                        corner[1], corner[2],
                                                                        nearest);
                                    const Scalar dx = std::abs(Scalar(centre[0] - nearest[0]));
                                    const Scalar dy = std::abs(Scalar(centre[1] - nearest[1]));
                                    const Scalar dz = std::abs(Scalar(centre[2] - nearest[2]));
                                    if (dx <= half && dy <= half && dz <= half)
                                        cells[cell_index(grid, x, y, z)] = CELL_SURFACE;
                                }
                            }
                        }
                    }
                }

                /**
                 * @brief Floods the exterior inward from the grid's corner.
                 *
                 * Six-connected, and it cannot pass through a surface cell. Everything it does
                 * not reach is interior — which is what makes this robust to the meshes real
                 * projects contain: a hole in the surface smaller than a cell does not let the
                 * fill through, so an open-shelled model still gets a solid interior.
                 */
                void flood_exterior(const std::int32_t grid[3], std::vector<std::uint8_t>& cells)
                {
                    // The grid is padded, so cell zero is guaranteed to be outside; starting
                    // anywhere else would be starting from an assumption.
                    if (cells[0] == CELL_SURFACE)
                        return;

                    std::vector<std::size_t> stack;
                    stack.push_back(0);
                    cells[0] = CELL_OUTSIDE;
                    while (!stack.empty())
                    {
                        const std::size_t current = stack.back();
                        stack.pop_back();

                        const std::int32_t x = std::int32_t(current % std::size_t(grid[0]));
                        const std::int32_t y = std::int32_t(
                            (current / std::size_t(grid[0])) % std::size_t(grid[1]));
                        const std::int32_t z = std::int32_t(current / (std::size_t(grid[0]) *
                                                                      std::size_t(grid[1])));

                        const std::int32_t steps[6][3] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                                                          {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};
                        for (const auto& step : steps)
                        {
                            const std::int32_t nx = x + step[0];
                            const std::int32_t ny = y + step[1];
                            const std::int32_t nz = z + step[2];
                            if (nx < 0 || ny < 0 || nz < 0 || nx >= grid[0] || ny >= grid[1] ||
                                nz >= grid[2])
                                continue;
                            const std::size_t neighbour = cell_index(grid, nx, ny, nz);
                            if (cells[neighbour] != CELL_UNKNOWN)
                                continue;
                            cells[neighbour] = CELL_OUTSIDE;
                            stack.push_back(neighbour);
                        }
                    }
                }

                /**
                 * @brief Decides which of the marked boundary cells are actually inside.
                 *
                 * The division of labour here is the whole robustness argument. The flood fill
                 * answers the *global* question — is this region enclosed — topologically, with
                 * no reliance on a surface normal, which is why a self-intersecting or
                 * open-shelled mesh still produces a solid body. But the fill cannot pass
                 * through the marked band, so every marked cell comes out "not reached" and
                 * a body inflated by the whole band's thickness.
                 *
                 * So the band, and only the band, is decided by the *local* question: is this
                 * cell's centre behind the surface. That is the one place a sign is trusted,
                 * it is the best information available there, and an error costs a single cell
                 * at a crease — which the cook report's accuracy number then measures. The
                 * alternative, believing the sign everywhere, is what §8.3 stage 2 rejects.
                 */
                void classify_surface_cells(const Geometry::MeshDistanceQuery& surface,
                                           const std::int32_t grid[3], const Vector3& origin,
                                           Scalar cell_size, std::vector<std::uint8_t>& cells)
                {
                    for (std::int32_t z = 0; z < grid[2]; ++z)
                    {
                        for (std::int32_t y = 0; y < grid[1]; ++y)
                        {
                            for (std::int32_t x = 0; x < grid[0]; ++x)
                            {
                                const std::size_t cell = cell_index(grid, x, y, z);
                                if (cells[cell] != CELL_SURFACE)
                                    continue;
                                const float centre[3] = {
                                    float(origin.x + (Scalar(x) + Scalar(0.5)) * cell_size),
                                    float(origin.y + (Scalar(y) + Scalar(0.5)) * cell_size),
                                    float(origin.z + (Scalar(z) + Scalar(0.5)) * cell_size)};
                                if (surface.signed_distance(centre) > 0.0f)
                                    cells[cell] = CELL_OUTSIDE;
                            }
                        }
                    }
                }

                /** @brief Inverts a 3x3 given as three rows; false when singular. */
                bool invert_3x3(const Vector3 rows[3], Vector3 out[3]) noexcept
                {
                    const Scalar determinant =
                        rows[0].x * (rows[1].y * rows[2].z - rows[1].z * rows[2].y) -
                        rows[0].y * (rows[1].x * rows[2].z - rows[1].z * rows[2].x) +
                        rows[0].z * (rows[1].x * rows[2].y - rows[1].y * rows[2].x);
                    if (determinant == Scalar(0))
                        return false;

                    const Scalar inverse = Scalar(1) / determinant;
                    out[0] = Vector3{(rows[1].y * rows[2].z - rows[1].z * rows[2].y) * inverse,
                                     (rows[0].z * rows[2].y - rows[0].y * rows[2].z) * inverse,
                                     (rows[0].y * rows[1].z - rows[0].z * rows[1].y) * inverse};
                    out[1] = Vector3{(rows[1].z * rows[2].x - rows[1].x * rows[2].z) * inverse,
                                     (rows[0].x * rows[2].z - rows[0].z * rows[2].x) * inverse,
                                     (rows[0].z * rows[1].x - rows[0].x * rows[1].z) * inverse};
                    out[2] = Vector3{(rows[1].x * rows[2].y - rows[1].y * rows[2].x) * inverse,
                                     (rows[0].y * rows[2].x - rows[0].x * rows[2].y) * inverse,
                                     (rows[0].x * rows[1].y - rows[0].y * rows[1].x) * inverse};
                    return true;
                }

                /** @brief The worst quality among the elements touching @p vertex. */
                float worst_incident_quality(const TetrahedralMesh& mesh,
                                            const std::vector<std::uint32_t>& incident_offsets,
                                            const std::vector<std::uint32_t>& incident,
                                            std::uint32_t vertex) noexcept
                {
                    float worst = 1.0f;
                    for (std::uint32_t i = incident_offsets[vertex];
                         i < incident_offsets[vertex + 1]; ++i)
                    {
                        const std::uint32_t t = incident[i];
                        const float quality = tetrahedron_quality(
                            mesh.vertices[mesh.tetrahedra[t * 4 + 0]],
                            mesh.vertices[mesh.tetrahedra[t * 4 + 1]],
                            mesh.vertices[mesh.tetrahedra[t * 4 + 2]],
                            mesh.vertices[mesh.tetrahedra[t * 4 + 3]]);
                        const Scalar volume = signed_volume_times_six(
                            mesh.vertices[mesh.tetrahedra[t * 4 + 0]],
                            mesh.vertices[mesh.tetrahedra[t * 4 + 1]],
                            mesh.vertices[mesh.tetrahedra[t * 4 + 2]],
                            mesh.vertices[mesh.tetrahedra[t * 4 + 3]]);
                        // A negative volume is an inverted element, which is worse than any
                        // quality value can express, so it reports as zero.
                        worst = std::min(worst, volume > Scalar(0) ? quality : 0.0f);
                    }
                    return worst;
                }

                /** @brief Builds a vertex-to-incident-tetrahedra table in compressed form. */
                void build_incidence(const TetrahedralMesh& mesh,
                                    std::vector<std::uint32_t>& offsets,
                                    std::vector<std::uint32_t>& incident)
                {
                    const std::size_t vertices = mesh.vertex_count();
                    const std::size_t tetrahedra = mesh.tetrahedron_count();
                    offsets.assign(vertices + 1, 0);
                    for (const std::uint32_t index : mesh.tetrahedra)
                        ++offsets[index + 1];
                    for (std::size_t i = 0; i < vertices; ++i)
                        offsets[i + 1] += offsets[i];

                    std::vector<std::uint32_t> cursor(offsets.begin(), offsets.end() - 1);
                    incident.assign(tetrahedra * 4, 0);
                    for (std::size_t t = 0; t < tetrahedra; ++t)
                    {
                        for (int corner = 0; corner < 4; ++corner)
                        {
                            const std::uint32_t vertex = mesh.tetrahedra[t * 4 + std::size_t(corner)];
                            incident[cursor[vertex]++] = std::uint32_t(t);
                        }
                    }
                }
            } // namespace

            float tetrahedron_quality(const Vector3& a, const Vector3& b, const Vector3& c,
                                      const Vector3& d) noexcept
            {
                const Scalar volume = std::abs(signed_volume_times_six(a, b, c, d)) / Scalar(6);
                if (!(volume > Scalar(0)))
                    return 0.0f;

                const Vector3 edges[6] = {b - a, c - a, d - a, c - b, d - b, d - c};
                Scalar sum = 0;
                for (const Vector3& edge : edges)
                    sum += dot(edge, edge);
                const Scalar mean_square = sum / Scalar(6);
                if (!(mean_square > Scalar(0)))
                    return 0.0f;

                // 6 sqrt(2) V / L_rms^3, which is exactly one for a regular tetrahedron: with
                // edge a, V = a^3 / (6 sqrt 2) and L_rms = a.
                const Scalar root_mean_square = std::sqrt(mean_square);
                const Scalar quality = Scalar(8.485281374238570) * volume /
                                       (root_mean_square * root_mean_square * root_mean_square);
                if (!(quality > Scalar(0)))
                    return 0.0f;
                return float(quality < Scalar(1) ? quality : Scalar(1));
            }

            Geometry::TriangleMesh TetrahedralMesh::surface_mesh() const
            {
                Geometry::TriangleMesh mesh;
                // Compacted rather than carrying every simulation vertex: the boundary is a
                // fraction of the lattice, and a hierarchy built over unreferenced vertices
                // measures a bounding box that is not the surface's.
                std::vector<std::uint32_t> remap(vertices.size(), NO_VERTEX);
                mesh.indices.reserve(surface_indices.size());
                for (const std::uint32_t index : surface_indices)
                {
                    if (index >= vertices.size())
                        continue;
                    if (remap[index] == NO_VERTEX)
                    {
                        remap[index] = std::uint32_t(mesh.positions.size() / 3);
                        mesh.positions.push_back(float(vertices[index].x));
                        mesh.positions.push_back(float(vertices[index].y));
                        mesh.positions.push_back(float(vertices[index].z));
                    }
                    mesh.indices.push_back(remap[index]);
                }
                return mesh;
            }

            TetrahedralizationReport build_tetrahedral_mesh(
                const Geometry::TriangleMeshView& mesh,
                const Geometry::MeshDistanceQuery& surface,
                const TetrahedralizationOptions& options, TetrahedralMesh& out)
            {
                out = TetrahedralMesh{};
                TetrahedralizationReport report;
                if (!mesh.has_triangles() || !surface.ready())
                    return report;

                float bounds_min[3];
                float bounds_max[3];
                if (!Geometry::compute_bounds(mesh, bounds_min, bounds_max))
                    return report;

                std::int32_t resolution = std::max(options.voxel_resolution, 2);
                const Scalar longest = std::max(
                    Scalar(bounds_max[0] - bounds_min[0]),
                    std::max(Scalar(bounds_max[1] - bounds_min[1]),
                             Scalar(bounds_max[2] - bounds_min[2])));
                if (!(longest > Scalar(0)))
                    return report;

                // The dial authors a tetrahedron *count*; six elements fill a cell, so the
                // resolution that would produce it scales as the cube root. Applied once and
                // then reported, rather than iterated: the artist chose an order of magnitude.
                if (options.target_tetrahedron_count > 0)
                {
                    const Scalar current =
                        Scalar(resolution) * Scalar(resolution) * Scalar(resolution) * Scalar(6);
                    const Scalar ratio = Scalar(options.target_tetrahedron_count) / current;
                    if (ratio > Scalar(0))
                    {
                        const Scalar scaled = Scalar(resolution) * std::cbrt(ratio);
                        resolution = std::max(std::int32_t(scaled + Scalar(0.5)), 2);
                    }
                }

                const Scalar cell_size = longest / Scalar(resolution);
                report.cell_size = cell_size;

                // Two cells of padding on every side, so the exterior fill starts in a cell
                // that is certainly outside and a snapped vertex has room to move.
                std::int32_t grid[3];
                Vector3 origin;
                for (int axis = 0; axis < 3; ++axis)
                {
                    const Scalar span = Scalar(bounds_max[axis] - bounds_min[axis]);
                    const std::int32_t needed =
                        std::int32_t(std::ceil(span / cell_size)) + 4;
                    grid[axis] = std::max(needed, 5);
                    const Scalar low = Scalar(bounds_min[axis]) - cell_size * Scalar(2);
                    if (axis == 0)
                        origin.x = low;
                    else if (axis == 1)
                        origin.y = low;
                    else
                        origin.z = low;
                }

                const std::size_t cell_total = std::size_t(grid[0]) * std::size_t(grid[1]) *
                                               std::size_t(grid[2]);
                std::vector<std::uint8_t> cells(cell_total, CELL_UNKNOWN);
                rasterize_surface(mesh, grid, origin, cell_size, cells);
                report.surface_cell_count =
                    std::uint32_t(std::count(cells.begin(), cells.end(), CELL_SURFACE));
                flood_exterior(grid, cells);
                classify_surface_cells(surface, grid, origin, cell_size, cells);
                report.resolution = resolution;

                // Stage 3: one Kuhn cell per interior cell. Corner vertices are created on
                // first touch, in cell order, so the numbering is a function of the grid.
                const std::int32_t corner_grid[3] = {grid[0] + 1, grid[1] + 1, grid[2] + 1};
                std::vector<std::uint32_t> corner_vertex(
                    std::size_t(corner_grid[0]) * std::size_t(corner_grid[1]) *
                        std::size_t(corner_grid[2]),
                    NO_VERTEX);
                std::vector<bool> vertex_on_boundary;

                for (std::int32_t z = 0; z < grid[2]; ++z)
                {
                    for (std::int32_t y = 0; y < grid[1]; ++y)
                    {
                        for (std::int32_t x = 0; x < grid[0]; ++x)
                        {
                            const std::size_t cell = cell_index(grid, x, y, z);
                            if (cells[cell] == CELL_OUTSIDE)
                                continue;
                            ++report.interior_cell_count;

                            std::uint32_t corner[8];
                            for (int c = 0; c < 8; ++c)
                            {
                                const std::int32_t cx = x + (c & 1);
                                const std::int32_t cy = y + ((c >> 1) & 1);
                                const std::int32_t cz = z + ((c >> 2) & 1);
                                const std::size_t slot =
                                    corner_index(corner_grid, cx, cy, cz);
                                if (corner_vertex[slot] == NO_VERTEX)
                                {
                                    corner_vertex[slot] = std::uint32_t(out.vertices.size());
                                    out.vertices.push_back(
                                        Vector3{origin.x + Scalar(cx) * cell_size,
                                                origin.y + Scalar(cy) * cell_size,
                                                origin.z + Scalar(cz) * cell_size});
                                    vertex_on_boundary.push_back(false);
                                }
                                corner[c] = corner_vertex[slot];
                            }

                            for (const auto& pattern : KUHN_TETRAHEDRA)
                            {
                                std::uint32_t element[4] = {corner[pattern[0]], corner[pattern[1]],
                                                            corner[pattern[2]], corner[pattern[3]]};
                                // Wound to a positive volume once, here, so every later pass
                                // may read the sign as a statement about deformation rather
                                // than about how the lattice happened to be listed.
                                if (signed_volume_times_six(out.vertices[element[0]],
                                                            out.vertices[element[1]],
                                                            out.vertices[element[2]],
                                                            out.vertices[element[3]]) < Scalar(0))
                                    std::swap(element[2], element[3]);
                                for (int c = 0; c < 4; ++c)
                                    out.tetrahedra.push_back(element[c]);
                                out.tetrahedron_cell.push_back(std::uint32_t(cell));
                            }
                        }
                    }
                }

                if (out.tetrahedra.empty())
                    return report;

                out.cell_size = cell_size;
                out.grid_origin = origin;
                for (int axis = 0; axis < 3; ++axis)
                    out.grid[axis] = grid[axis];

                // A vertex is on the boundary when some incident cell was not interior. Only
                // those are candidates for snapping: moving an interior vertex onto the
                // surface would fold the lattice through itself.
                for (std::int32_t z = 0; z <= grid[2]; ++z)
                {
                    for (std::int32_t y = 0; y <= grid[1]; ++y)
                    {
                        for (std::int32_t x = 0; x <= grid[0]; ++x)
                        {
                            const std::uint32_t vertex =
                                corner_vertex[corner_index(corner_grid, x, y, z)];
                            if (vertex == NO_VERTEX)
                                continue;
                            bool exposed = false;
                            for (std::int32_t dz = -1; dz <= 0 && !exposed; ++dz)
                            {
                                for (std::int32_t dy = -1; dy <= 0 && !exposed; ++dy)
                                {
                                    for (std::int32_t dx = -1; dx <= 0 && !exposed; ++dx)
                                    {
                                        const std::int32_t cx = x + dx;
                                        const std::int32_t cy = y + dy;
                                        const std::int32_t cz = z + dz;
                                        if (cx < 0 || cy < 0 || cz < 0 || cx >= grid[0] ||
                                            cy >= grid[1] || cz >= grid[2])
                                        {
                                            exposed = true;
                                            break;
                                        }
                                        if (cells[cell_index(grid, cx, cy, cz)] == CELL_OUTSIDE)
                                            exposed = true;
                                    }
                                }
                            }
                            vertex_on_boundary[vertex] = exposed;
                        }
                    }
                }

                // Stages 3b and 4: snap the boundary onto the surface, smooth the interior,
                // and revert any move that made an incident element worse. The guard is what
                // makes this safe to iterate — without it, snapping a vertex across a thin
                // region inverts the elements behind it and the solve diverges on frame one.
                std::vector<std::uint32_t> incident_offsets;
                std::vector<std::uint32_t> incident;
                build_incidence(out, incident_offsets, incident);

                const std::int32_t passes = std::max(options.conforming_passes, 0);
                for (std::int32_t pass = 0; pass < passes; ++pass)
                {
                    for (std::size_t v = 0; v < out.vertices.size(); ++v)
                    {
                        const Vector3 original = out.vertices[v];
                        const float before = worst_incident_quality(out, incident_offsets,
                                                                    incident, std::uint32_t(v));

                        Vector3 candidate = original;
                        if (vertex_on_boundary[v])
                        {
                            const float point[3] = {float(original.x), float(original.y),
                                                    float(original.z)};
                            const Geometry::MeshClosestPoint nearest =
                                surface.closest_point(point);
                            if (!nearest.valid)
                                continue;
                            // Only within a cell: a vertex further than that from the surface
                            // is not describing the same feature, and dragging it there would
                            // be inventing geometry rather than conforming to it.
                            if (Scalar(nearest.distance) > cell_size)
                                continue;
                            candidate = Vector3{Scalar(nearest.point[0]), Scalar(nearest.point[1]),
                                                Scalar(nearest.point[2])};
                        }
                        else
                        {
                            Vector3 centroid{0, 0, 0};
                            std::uint32_t neighbours = 0;
                            for (std::uint32_t i = incident_offsets[v];
                                 i < incident_offsets[v + 1]; ++i)
                            {
                                const std::uint32_t t = incident[i];
                                for (int corner = 0; corner < 4; ++corner)
                                {
                                    const std::uint32_t other =
                                        out.tetrahedra[std::size_t(t) * 4 + std::size_t(corner)];
                                    if (other == v)
                                        continue;
                                    centroid = centroid + out.vertices[other];
                                    ++neighbours;
                                }
                            }
                            if (neighbours == 0)
                                continue;
                            centroid = centroid * (Scalar(1) / Scalar(neighbours));
                            // Halfway, not all the way: full Laplacian smoothing shrinks a
                            // body toward its own centre over repeated passes.
                            candidate = original + (centroid - original) * Scalar(0.5);
                        }

                        out.vertices[v] = candidate;
                        const float after = worst_incident_quality(out, incident_offsets, incident,
                                                                   std::uint32_t(v));
                        if (after < before)
                        {
                            out.vertices[v] = original;
                            continue;
                        }
                        if (vertex_on_boundary[v])
                            ++report.snapped_vertex_count;
                    }
                }

                // Stage 4's removal, after the moves rather than before: a sliver the snapping
                // is about to fix is not a sliver worth deleting, and deleting it first would
                // punch a hole in the lattice the smoothing then closes around.
                {
                    std::vector<std::uint32_t> kept;
                    std::vector<std::uint32_t> kept_cell;
                    kept.reserve(out.tetrahedra.size());
                    kept_cell.reserve(out.tetrahedron_cell.size());
                    const std::size_t count = out.tetrahedron_count();
                    for (std::size_t t = 0; t < count; ++t)
                    {
                        const std::uint32_t* element = out.tetrahedra.data() + t * 4;
                        const Vector3& a = out.vertices[element[0]];
                        const Vector3& b = out.vertices[element[1]];
                        const Vector3& c = out.vertices[element[2]];
                        const Vector3& d = out.vertices[element[3]];
                        const Scalar volume = signed_volume_times_six(a, b, c, d) / Scalar(6);
                        const float quality = tetrahedron_quality(a, b, c, d);
                        if (volume <= Scalar(0))
                        {
                            // Reported, not silently dropped: an inverted element means the
                            // conforming guard let a move through that it should not have, and
                            // that is a defect in this function rather than in the mesh.
                            ++report.inverted_element_count;
                            ++report.removed_tetrahedron_count;
                            continue;
                        }
                        if (quality < options.min_element_quality)
                        {
                            ++report.removed_tetrahedron_count;
                            continue;
                        }
                        for (int corner = 0; corner < 4; ++corner)
                            kept.push_back(element[corner]);
                        kept_cell.push_back(out.tetrahedron_cell[t]);
                    }
                    out.tetrahedra = std::move(kept);
                    out.tetrahedron_cell = std::move(kept_cell);
                }

                if (out.tetrahedra.empty())
                    return report;

                // Stage 5: the rest state. One pass produces the inverse rest matrix, the rest
                // volume, and the vertex masses the solve reads as inverse masses.
                const std::size_t count = out.tetrahedron_count();
                out.rest_inverse.assign(count * 3, Vector3{0, 0, 0});
                out.rest_volume.assign(count, 0);
                out.vertex_mass.assign(out.vertices.size(), 0);
                report.worst_element_quality = 1.0f;

                for (std::size_t t = 0; t < count; ++t)
                {
                    const std::uint32_t* element = out.tetrahedra.data() + t * 4;
                    const Vector3& a = out.vertices[element[0]];
                    const Vector3 rows[3] = {out.vertices[element[1]] - a,
                                             out.vertices[element[2]] - a,
                                             out.vertices[element[3]] - a};
                    Vector3 inverse[3];
                    if (!invert_3x3(rows, inverse))
                    {
                        // Unreachable for an element that survived the volume test above; left
                        // total rather than assumed away, since a singular rest matrix would
                        // make every deformation gradient a not-a-number.
                        inverse[0] = Vector3{0, 0, 0};
                        inverse[1] = Vector3{0, 0, 0};
                        inverse[2] = Vector3{0, 0, 0};
                    }
                    for (int row = 0; row < 3; ++row)
                        out.rest_inverse[t * 3 + std::size_t(row)] = inverse[row];

                    const Scalar volume =
                        signed_volume_times_six(a, out.vertices[element[1]],
                                                out.vertices[element[2]],
                                                out.vertices[element[3]]) /
                        Scalar(6);
                    out.rest_volume[t] = volume;
                    report.total_volume += volume;

                    const Scalar quarter = volume * Scalar(options.density) / Scalar(4);
                    for (int corner = 0; corner < 4; ++corner)
                        out.vertex_mass[element[corner]] += quarter;

                    report.worst_element_quality =
                        std::min(report.worst_element_quality,
                                 tetrahedron_quality(a, out.vertices[element[1]],
                                                     out.vertices[element[2]],
                                                     out.vertices[element[3]]));
                }

                // Stage 7: the boundary. A face shared by two elements is interior; one that
                // appears once is the surface, and the winding follows from the element's
                // positive orientation.
                {
                    struct FaceRecord
                    {
                        std::uint64_t key;
                        std::uint32_t a;
                        std::uint32_t b;
                        std::uint32_t c;
                    };
                    const int FACES[4][3] = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};

                    std::vector<FaceRecord> faces;
                    faces.reserve(count * 4);
                    for (std::size_t t = 0; t < count; ++t)
                    {
                        const std::uint32_t* element = out.tetrahedra.data() + t * 4;
                        for (const auto& face : FACES)
                        {
                            FaceRecord record{};
                            record.a = element[face[0]];
                            record.b = element[face[1]];
                            record.c = element[face[2]];
                            std::uint32_t sorted[3] = {record.a, record.b, record.c};
                            std::sort(sorted, sorted + 3);
                            // Twenty-one bits per index, which the lattice cannot exceed: two
                            // million vertices is four hundred times the top of the dial.
                            record.key = (std::uint64_t(sorted[0]) << 42) |
                                         (std::uint64_t(sorted[1]) << 21) |
                                         std::uint64_t(sorted[2]);
                            faces.push_back(record);
                        }
                    }
                    std::sort(faces.begin(), faces.end(),
                              [](const FaceRecord& left, const FaceRecord& right) noexcept
                              {
                                  if (left.key != right.key)
                                      return left.key < right.key;
                                  if (left.a != right.a)
                                      return left.a < right.a;
                                  if (left.b != right.b)
                                      return left.b < right.b;
                                  return left.c < right.c;
                              });

                    std::size_t run = 0;
                    while (run < faces.size())
                    {
                        std::size_t end = run + 1;
                        while (end < faces.size() && faces[end].key == faces[run].key)
                            ++end;
                        if (end - run == 1)
                        {
                            out.surface_indices.push_back(faces[run].a);
                            out.surface_indices.push_back(faces[run].b);
                            out.surface_indices.push_back(faces[run].c);
                        }
                        run = end;
                    }
                }

                report.vertex_count = std::uint32_t(out.vertices.size());
                report.tetrahedron_count = std::uint32_t(count);
                return report;
            }

            Vector3 evaluate_binding(const TetrahedralMesh& mesh,
                                     const TetrahedronBinding& binding) noexcept
            {
                if (std::size_t(binding.tetrahedron) >= mesh.tetrahedron_count())
                    return Vector3{0, 0, 0};
                const std::uint32_t* element =
                    mesh.tetrahedra.data() + std::size_t(binding.tetrahedron) * 4;
                Vector3 position{0, 0, 0};
                for (int corner = 0; corner < 4; ++corner)
                {
                    position = position + mesh.vertices[element[corner]] *
                                              Scalar(binding.weights[corner]);
                }
                return position;
            }

            std::uint32_t embed_points(const TetrahedralMesh& mesh, const Vector3* points,
                                       std::size_t count, std::vector<TetrahedronBinding>& out)
            {
                out.clear();
                if (points == nullptr || count == 0)
                    return 0;
                out.assign(count, TetrahedronBinding{});
                const std::size_t tetrahedra = mesh.tetrahedron_count();
                if (tetrahedra == 0 || !(mesh.cell_size > Scalar(0)))
                    return std::uint32_t(count);

                // A compressed cell-to-elements table, so the search is arithmetic rather
                // than a sweep over every element per point. Built here and not cached on the
                // mesh: the mesh is the asset's content, and an acceleration structure that
                // travels with it is a structure that has to be kept in step with it.
                const std::size_t cell_total = std::size_t(mesh.grid[0]) *
                                               std::size_t(mesh.grid[1]) *
                                               std::size_t(mesh.grid[2]);
                std::vector<std::uint32_t> offsets(cell_total + 1, 0);
                for (const std::uint32_t cell : mesh.tetrahedron_cell)
                {
                    if (std::size_t(cell) < cell_total)
                        ++offsets[std::size_t(cell) + 1];
                }
                for (std::size_t i = 0; i < cell_total; ++i)
                    offsets[i + 1] += offsets[i];
                std::vector<std::uint32_t> cursor(offsets.begin(), offsets.end() - 1);
                std::vector<std::uint32_t> by_cell(mesh.tetrahedron_cell.size(), 0);
                for (std::size_t t = 0; t < mesh.tetrahedron_cell.size(); ++t)
                {
                    const std::uint32_t cell = mesh.tetrahedron_cell[t];
                    if (std::size_t(cell) < cell_total)
                        by_cell[cursor[cell]++] = std::uint32_t(t);
                }

                std::uint32_t extrapolated = 0;
                for (std::size_t p = 0; p < count; ++p)
                {
                    const Vector3& point = points[p];
                    const float query[3] = {float(point.x), float(point.y), float(point.z)};

                    std::int32_t home[3];
                    const Scalar base[3] = {mesh.grid_origin.x, mesh.grid_origin.y,
                                            mesh.grid_origin.z};
                    const Scalar component[3] = {point.x, point.y, point.z};
                    for (int axis = 0; axis < 3; ++axis)
                    {
                        home[axis] = std::int32_t(
                            std::floor((component[axis] - base[axis]) / mesh.cell_size));
                    }

                    bool bound = false;
                    std::uint32_t nearest_element = 0;
                    Scalar nearest_distance = std::numeric_limits<Scalar>::max();

                    // Rings outward from the point's own cell. Two are enough for a point
                    // inside the lattice; the widening rings are what catch a render vertex
                    // sitting outside a snapped boundary.
                    for (std::int32_t radius = 1; radius <= 4 && !bound; ++radius)
                    {
                        for (std::int32_t dz = -radius; dz <= radius && !bound; ++dz)
                        {
                            for (std::int32_t dy = -radius; dy <= radius && !bound; ++dy)
                            {
                                for (std::int32_t dx = -radius; dx <= radius && !bound; ++dx)
                                {
                                    const std::int32_t cx = home[0] + dx;
                                    const std::int32_t cy = home[1] + dy;
                                    const std::int32_t cz = home[2] + dz;
                                    if (cx < 0 || cy < 0 || cz < 0 || cx >= mesh.grid[0] ||
                                        cy >= mesh.grid[1] || cz >= mesh.grid[2])
                                        continue;
                                    const std::size_t cell =
                                        std::size_t(cx) +
                                        std::size_t(mesh.grid[0]) *
                                            (std::size_t(cy) +
                                             std::size_t(mesh.grid[1]) * std::size_t(cz));
                                    for (std::uint32_t i = offsets[cell]; i < offsets[cell + 1];
                                         ++i)
                                    {
                                        const std::uint32_t t = by_cell[i];
                                        const std::uint32_t* element =
                                            mesh.tetrahedra.data() + std::size_t(t) * 4;
                                        float corner[4][3];
                                        for (int c = 0; c < 4; ++c)
                                        {
                                            const Vector3& v = mesh.vertices[element[c]];
                                            corner[c][0] = float(v.x);
                                            corner[c][1] = float(v.y);
                                            corner[c][2] = float(v.z);
                                        }
                                        float weights[4];
                                        if (!Geometry::tetrahedron_barycentric(
                                                query, corner[0], corner[1], corner[2], corner[3],
                                                weights))
                                            continue;

                                        const bool inside =
                                            weights[0] >= -1.0e-5f && weights[1] >= -1.0e-5f &&
                                            weights[2] >= -1.0e-5f && weights[3] >= -1.0e-5f;
                                        if (inside)
                                        {
                                            TetrahedronBinding binding;
                                            binding.tetrahedron = t;
                                            for (int c = 0; c < 4; ++c)
                                                binding.weights[c] = weights[c];
                                            binding.inside = true;
                                            out[p] = binding;
                                            bound = true;
                                            break;
                                        }

                                        // How far outside, measured by the most negative
                                        // coordinate: the nearest element by that measure is
                                        // the one whose extrapolation distorts the least.
                                        Scalar outside = 0;
                                        for (int c = 0; c < 4; ++c)
                                        {
                                            if (weights[c] < 0.0f)
                                                outside -= Scalar(weights[c]);
                                        }
                                        // Strictly less, so a tie falls to the lower element
                                        // index and the binding is a function of the lattice.
                                        if (outside < nearest_distance)
                                        {
                                            nearest_distance = outside;
                                            nearest_element = t;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    if (bound)
                        continue;

                    // Nothing contained it. Bind by extrapolated coordinates against the
                    // closest element found, which keeps a thin feature attached and moving
                    // correctly rather than leaving a hole in the render mesh (§8.6 item 1).
                    if (nearest_distance == std::numeric_limits<Scalar>::max())
                    {
                        // Not even a neighbouring cell held an element. Fall back to the whole
                        // lattice rather than reporting the point unbound: a sweep once for a
                        // handful of stragglers is cheaper than a torn mesh.
                        for (std::size_t t = 0; t < tetrahedra; ++t)
                        {
                            const std::uint32_t* element = mesh.tetrahedra.data() + t * 4;
                            Vector3 centroid{0, 0, 0};
                            for (int c = 0; c < 4; ++c)
                                centroid = centroid + mesh.vertices[element[c]];
                            centroid = centroid * Scalar(0.25);
                            const Vector3 offset = point - centroid;
                            const Scalar distance = dot(offset, offset);
                            if (distance < nearest_distance)
                            {
                                nearest_distance = distance;
                                nearest_element = std::uint32_t(t);
                            }
                        }
                    }

                    const std::uint32_t* element =
                        mesh.tetrahedra.data() + std::size_t(nearest_element) * 4;
                    float corner[4][3];
                    for (int c = 0; c < 4; ++c)
                    {
                        const Vector3& v = mesh.vertices[element[c]];
                        corner[c][0] = float(v.x);
                        corner[c][1] = float(v.y);
                        corner[c][2] = float(v.z);
                    }
                    TetrahedronBinding binding;
                    binding.tetrahedron = nearest_element;
                    Geometry::tetrahedron_barycentric(query, corner[0], corner[1], corner[2],
                                                      corner[3], binding.weights);
                    binding.inside = false;
                    out[p] = binding;
                    ++extrapolated;
                }
                return extrapolated;
            }
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
