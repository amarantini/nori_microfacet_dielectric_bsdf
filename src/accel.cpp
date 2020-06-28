/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/accel.h>
#include <Eigen/Geometry>
#include <chrono>

using namespace std::chrono;

NORI_NAMESPACE_BEGIN

void Accel::addMesh(Mesh *mesh) {
    if (m_mesh)
        throw NoriException("Accel: only a single mesh is supported!");
    m_mesh = mesh;
    m_bbox = m_mesh->getBoundingBox();
}

void Accel::build() {
    if (!m_mesh)
        throw NoriException("No mesh found, could not build acceleration structure");

    auto start = high_resolution_clock::now();
    delete m_root;

    uint32_t num_triangles = m_mesh->getTriangleCount();
    std::vector<uint32_t> triangles;
    for(uint32_t i = 0; i < num_triangles; i++) {
        triangles.emplace_back(i);
    }

    m_root = buildRecursive(m_bbox, triangles, 0);
    printf("Octree build time: %ldms \n", duration_cast<milliseconds>(high_resolution_clock::now() - start).count());
    printf("Num nodes: %d \n", m_num_nodes);
    printf("Num leaf nodes: %d \n", m_num_leaf_nodes);
    printf("Num non-empty leaf nodes: %d \n", m_num_nonempty_leaf_nodes);
    printf("Total number of saved triangles: %d \n", m_num_triangles_saved);
    printf("Avg triangles per node: %f \n", (float)m_num_triangles_saved / (float)m_num_nodes);
    printf("Recursion depth: %d \n", m_recursion_depth);
}

bool Accel::rayIntersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const {
    bool foundIntersection = false;  // Was an intersection found so far?
    uint32_t f = (uint32_t) -1;      // Triangle index of the closest intersection

    Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)

    /* Brute force search through all triangles */
    for (uint32_t idx = 0; idx < m_mesh->getTriangleCount(); ++idx) {
        float u, v, t;
        if (m_mesh->rayIntersect(idx, ray, u, v, t)) {
            /* An intersection was found! Can terminate
               immediately if this is a shadow ray query */
            if (shadowRay)
                return true;
            ray.maxt = its.t = t;
            its.uv = Point2f(u, v);
            its.mesh = m_mesh;
            f = idx;
            foundIntersection = true;
        }
    }

    if (foundIntersection) {
        /* At this point, we now know that there is an intersection,
           and we know the triangle index of the closest such intersection.

           The following computes a number of additional properties which
           characterize the intersection (normals, texture coordinates, etc..)
        */

        /* Find the barycentric coordinates */
        Vector3f bary;
        bary << 1-its.uv.sum(), its.uv;

        /* References to all relevant mesh buffers */
        const Mesh *mesh   = its.mesh;
        const MatrixXf &V  = mesh->getVertexPositions();
        const MatrixXf &N  = mesh->getVertexNormals();
        const MatrixXf &UV = mesh->getVertexTexCoords();
        const MatrixXu &F  = mesh->getIndices();

        /* Vertex indices of the triangle */
        uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);

        Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

        /* Compute the intersection positon accurately
           using barycentric coordinates */
        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

        /* Compute proper texture coordinates if provided by the mesh */
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) +
                bary.y() * UV.col(idx1) +
                bary.z() * UV.col(idx2);

        /* Compute the geometry frame */
        its.geoFrame = Frame((p1-p0).cross(p2-p0).normalized());

        if (N.size() > 0) {
            /* Compute the shading frame. Note that for simplicity,
               the current implementation doesn't attempt to provide
               tangents that are continuous across the surface. That
               means that this code will need to be modified to be able
               use anisotropic BRDFs, which need tangent continuity */

            its.shFrame = Frame(
                (bary.x() * N.col(idx0) +
                 bary.y() * N.col(idx1) +
                 bary.z() * N.col(idx2)).normalized());
        } else {
            its.shFrame = its.geoFrame;
        }
    }

    return foundIntersection;
}

Accel::Node* Accel::buildRecursive(const BoundingBox3f& bbox, std::vector<uint32_t>& triangle_indices, uint32_t recursion_depth) {
    // a node is created in any case
    m_num_nodes++;

    uint32_t num_triangles = triangle_indices.size();

    // return empty node if no triangles are left
    if (num_triangles == 0) {
        Node* node = new Node();
        node->bbox = BoundingBox3f(bbox);

        // add to statistics
        m_num_leaf_nodes++;
        return node;
    }

    // create leaf node if 10 or less triangles are left or if the max recursion depth is reached.
    if (num_triangles <= MAX_TRIANGLES_PER_NODE || recursion_depth >= MAX_RECURSION_DEPTH) {
        Node* node = new Node();
        node->num_triangles = num_triangles;
        node->triangle_indices = new uint32_t[num_triangles];

        for (uint32_t i = 0; i < num_triangles; i++) {
            node->triangle_indices[i] = triangle_indices[i];
        }
        node->bbox = BoundingBox3f(bbox);

        // add to statistics
        m_num_leaf_nodes++;
        m_num_nonempty_leaf_nodes++;
        m_num_triangles_saved += num_triangles;
        return node;
    }

    Node* node = new Node();

    BoundingBox3f bboxes[8] = {};
    subdivideBBox(bbox, bboxes);

    std::vector<std::vector<uint32_t>> bbox_triangle_indices(8);

    uint32_t bbox_num_triangles[8] = {};

    // for every child bbox
    for (uint32_t i = 0; i < 8; i++) {
        // for every triangle inside of the parent create triangle bounding box
        for (uint32_t j = 0; j < num_triangles; j++) {
            BoundingBox3f triangle_bbox;

            // for every triangle vertex expand triangle bbox
            for (uint32_t k = 0; k < 3; k++) {
                uint32_t idx = m_mesh->getIndices()(k, triangle_indices[j]);
                const Point3f p = m_mesh->getVertexPositions().col(idx);
                triangle_bbox.expandBy(p);
            }

            // check if triangle is in bbox
            if (bboxes[i].overlaps(triangle_bbox)) {
                bbox_triangle_indices[i].emplace_back(triangle_indices[j]);
                bbox_num_triangles[i]++;
            }
        }
    }

    // release memory to avoid stack overflow
    triangle_indices = std::vector<uint32_t>();

    // for every child bbox
    for (uint32_t i = 0; i < 8; i++) {
        Node* last_child;
        // first child
        if (i == 0) {
            node->child = buildRecursive(bboxes[i], bbox_triangle_indices[i], recursion_depth + 1);
            last_child = node->child;
        // neighbour children
        } else {
            last_child->next = buildRecursive(bboxes[i], bbox_triangle_indices[i], recursion_depth + 1);
            last_child = last_child->next;
        }
        m_recursion_depth = std::max(m_recursion_depth, recursion_depth + 1);
    }
    return node;
}

void Accel::subdivideBBox(const nori::BoundingBox3f &parent, nori::BoundingBox3f *bboxes) {
    Point3f extents = parent.getExtents();

    Point3f x0_y0_z0 = parent.min;
    Point3f x1_y0_z0 = Point3f(parent.min.x() + extents.x() / 2.f, parent.min.y(), parent.min.z());
    Point3f x0_y1_z0 = Point3f(parent.min.x(), parent.min.y() + extents.y() / 2.f, parent.min.z());
    Point3f x1_y1_z0 = Point3f(parent.min.x() + extents.x() / 2.f, parent.min.y() + extents.y() / 2.f, parent.min.z());

    Point3f x0_y0_z1 = Point3f(parent.min.x(), parent.min.y(), parent.min.z() + extents.z() / 2.f);
    Point3f x1_y0_z1 = Point3f(parent.min.x() + extents.x() / 2.f, parent.min.y(), parent.min.z() + extents.z() / 2.f);
    Point3f x0_y1_z1 = Point3f(parent.min.x(), parent.min.y() + extents.y() / 2.f, parent.min.z() + extents.z() / 2.f);
    Point3f x1_y1_z1 = Point3f(parent.min.x() + extents.x() / 2.f, parent.min.y() + extents.y() / 2.f, parent.min.z() + extents.z() / 2.f);
    Point3f x2_y1_z1 = Point3f(parent.max.x(), parent.min.y() + extents.y() / 2.f, parent.min.z() + extents.z() / 2.f);
    Point3f x1_y2_z1 = Point3f(parent.min.x() + extents.x() / 2.f, parent.max.y(), parent.min.z() + extents.z() / 2.f);
    Point3f x2_y2_z1 = Point3f(parent.max.x(), parent.max.y(), parent.min.z() + extents.z() / 2.f);

    Point3f x1_y1_z2 = Point3f(parent.min.x() + extents.x() / 2.f, parent.min.y() + extents.y() / 2.f, parent.max.z());
    Point3f x2_y1_z2 = Point3f(parent.max.x(), parent.min.y() + extents.y() / 2.f, parent.max.z());
    Point3f x1_y2_z2 = Point3f(parent.min.x() + extents.x() / 2.f, parent.max.y(), parent.max.z());
    Point3f x2_y2_z2 = Point3f(parent.max.x(), parent.max.y(), parent.max.z());

    bboxes[0] = BoundingBox3f(x0_y0_z0, x1_y1_z1);
    bboxes[1] = BoundingBox3f(x1_y0_z0, x2_y1_z1);
    bboxes[2] = BoundingBox3f(x0_y1_z0, x1_y2_z1);
    bboxes[3] = BoundingBox3f(x1_y1_z0, x2_y2_z1);
    bboxes[4] = BoundingBox3f(x0_y0_z1, x1_y1_z2);
    bboxes[5] = BoundingBox3f(x1_y0_z1, x2_y1_z2);
    bboxes[6] = BoundingBox3f(x0_y1_z1, x1_y2_z2);
    bboxes[7] = BoundingBox3f(x1_y1_z1, x2_y2_z2);
}

NORI_NAMESPACE_END

