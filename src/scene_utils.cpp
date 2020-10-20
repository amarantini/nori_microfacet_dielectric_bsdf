
#include <nori/scene_utils.h>
#include <nori/emitter.h>
#include <nori/scene.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

EmitterQueryRecord SceneUtils::sampleLightSource(const Scene* scene, Sampler* sampler, const Point3f& shading_point,
                                            const Normal3f& shading_normal, Emitter*& emitter, float& light_pdf) {
    Mesh *emitter_mesh;
    const auto &emitter_meshes = scene->getEmitter();
    float emitter_sample = sampler->next1D() * emitter_meshes.size();
    emitter_mesh = emitter_meshes[(int) emitter_sample];

    emitter = emitter_mesh->getEmitter();

    // sample point on emitter
    float surface_pdf;
    Normal3f light_normal;
    Point3f light_point = emitter_mesh->sampleSurfaceUniform(sampler, light_normal, surface_pdf);
    EmitterQueryRecord emitter_record(shading_point, shading_normal, light_point, light_normal);
    light_pdf = getLightPdf(scene, emitter_mesh, emitter_record);

    return emitter_record;
}

Color3f SceneUtils::getIncomingLightRadiance(const EmitterQueryRecord& emitter_record, const Emitter* emitter,
                                        const Scene* scene) {
    // shadow ray query, corresponds to V(x, p)
    Vector3f wo = (emitter_record.light_point - emitter_record.shading_point);
    float dist = wo.norm();
    wo.normalize();
    if ((-wo).dot(emitter_record.light_normal) <= 0) {
        return {0.f};
    }
    Ray3f shadow_ray = Ray3f(emitter_record.shading_point, wo);
    shadow_ray.maxt = dist - Epsilon;
    if (scene->rayIntersect(shadow_ray)) {
        return {0.f};
    }
    return emitter->evalQueryRecord(emitter_record);
}

float SceneUtils::getLightPdf(const Scene* scene, const Mesh* emitter_mesh, const EmitterQueryRecord& record) {
    Vector3f light_dir = record.shading_point - record.light_point;
    float length_squared = light_dir.squaredNorm();
    light_dir.normalize();
    float cos_theta = abs(light_dir.dot(record.light_normal));
    return clamp(1.f / scene->getEmitter().size() * emitter_mesh->getPdf() * length_squared / cos_theta, 0.f, 100000000000.f);
}

NORI_NAMESPACE_END
