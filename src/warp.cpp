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

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

float sign (const Point2f& p1, const Point2f& p2, const Point2f& p3)
{
    return (p1.x() - p3.x()) * (p2.y() - p3.y()) - (p2.x() - p3.x()) * (p1.y() - p3.y());
}

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample) {
    float s = 1.f - sqrt(1.f - sample.x());
    float t = (1.f - s) * sample.y();
    return Point2f(-1.f, 0.f) + s * Point2f(2.f, 0.f) + t * Point2f(1.f, 1.f);
}

float Warp::squareToTentPdf(const Point2f &p) {
    float d1, d2, d3;
    bool has_neg, has_pos;
    Point2f v1 = Point2f(-1.f, 0.f);
    Point2f v2 = Point2f(1.f, 0.f);
    Point2f v3 = Point2f(0.f, 1.f);

    d1 = sign(p, v1, v2);
    d2 = sign(p, v2, v3);
    d3 = sign(p, v3, v1);

    has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    return !(has_neg && has_pos) ? 1.f : 0.f;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float phi = 2.f * M_PI * sample.x();
    float r = sqrt(sample.y());
    return {r * cos(phi), r * sin(phi)};
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    return (p.x() * p.x() + p.y() * p.y() <= 1) ? M_1_PI : 0.f;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float theta = acos(1 - 2 * sample.x());
    float phi = 2 * M_PI * sample.y();
    return {sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)};
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    return (std::abs((v.x() * v.x() + v.y() * v.y() + v.z() * v.z()) - 1.f) < Epsilon) ? 0.25f * M_1_PI : 0.f;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    float theta = acos(1 - sample.x());
    float phi = 2 * M_PI * sample.y();
    return {sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)};
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    return (std::abs((v.x() * v.x() + v.y() * v.y() + v.z() * v.z()) - 1.f) < Epsilon
        && v.z() > 0) ? 0.5f * M_1_PI : 0.f;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    float theta = acos(sqrt(1 - sample.x()));
    float phi = 2 * M_PI * sample.y();
    return {sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)};
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    return (std::abs((v.x() * v.x() + v.y() * v.y() + v.z() * v.z()) - 1.f) < Epsilon
            && v.z() > 0) ? (float)M_1_PI * v.z() : 0.f;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    float phi = 2 * M_PI * sample.x();
    float theta = atan(sqrt(-alpha * alpha * log(1 - sample.y())));
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)).normalized();
}

Vector3f Warp::squareToGXX(const Point2f &sample, float alpha) {
    float phi = 2 * M_PI * sample.x();
    float theta = atan(sqrt(sample.y()) * alpha / sqrt(1-sample.y()));
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)).normalized();
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    if (m.z() <= 0) return 0.f;
    float a_2 = alpha * alpha;
    float tan_theta = tan(acos(m.z()));

    float azimuthal = 0.5f * M_1_PI;
    float longitudinal = 2 * exp((-tan_theta * tan_theta) / a_2) / (a_2 * m.z() * m.z() * m.z());

    return azimuthal * longitudinal;
}

NORI_NAMESPACE_END
