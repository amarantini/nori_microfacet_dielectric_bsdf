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

#pragma once

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

struct EmitterQueryRecord {
    Point3f shading_point;
    Normal3f shading_normal;
    Point3f light_point;
    Normal3f light_normal;

    EmitterQueryRecord(Point3f shading_point, Normal3f shading_normal, Point3f light_point, Normal3f light_normal)
        : shading_point(std::move(shading_point)),
        shading_normal(std::move(shading_normal)),
        light_point(std::move(light_point)),
        light_normal(std::move(light_normal))
    {}
};

/**
 * \brief Superclass of all emitters
 */
class Emitter : public NoriObject {
public:

    /**
     * \brief Return the type of object (i.e. Mesh/Emitter/etc.) 
     * provided by this instance
     * */
    EClassType getClassType() const { return EEmitter; }

    /**
     * Evaluates a query record (calculates how much light is emitted from a light point to a shading point)
     * @param record EmitterQueryRecord containing shading point/normal and light point/normal
     * @return Emitted light as Color3f
     */
    virtual Color3f evalQueryRecord(const EmitterQueryRecord& record) const = 0;
    virtual Color3f getRadiance() const = 0;
};

NORI_NAMESPACE_END
