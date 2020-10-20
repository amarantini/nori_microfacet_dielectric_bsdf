//
// Created by jallmenroeder on 19.10.20.
//

#pragma once

#include <nori/common.h>
#include <nori/vector.h>
#include <nori/color.h>

NORI_NAMESPACE_BEGIN

class SceneUtils {
public:
    /**
     * Samples a point on a scene light and returns the query record, the emitter object of the scene light and the
     * probability of the sample.
     */
    static EmitterQueryRecord sampleLightSource(const Scene* scene, Sampler* sampler, const Point3f& shading_point,
                                                const Normal3f& shading_normal, Emitter*& emitter, float& light_pdf);
    /**
     * Calculates the incoming radiance from an emitter to a shading point in a given scene.
     */
    static Color3f getIncomingLightRadiance(const EmitterQueryRecord& emitter_record, const Emitter* emitter,
                                            const Scene* scene);

    /**
     * Returns the probability of a light sample wrt solid angles. Since only uniform light sampling is supported, no
     * specific sample is needed.
     */
    static float getLightPdf(const Scene* scene, const Mesh* emitter_mesh, const EmitterQueryRecord& record);
};

NORI_NAMESPACE_END
