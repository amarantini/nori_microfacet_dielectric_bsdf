#include <nori/sampler.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene_utils.h>

NORI_NAMESPACE_BEGIN

class WhittedIntegrator : public Integrator {
public:
    WhittedIntegrator(const PropertyList &props) {};

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        Color3f light_eval(0.f);
        Color3f emitted_light(0.f);

        if (!scene->rayIntersect(ray, its))
            return {0.0f};

        if (its.mesh->getBSDF()->isDiffuse()) {
            // diffuse shading
            Mesh *emitter_mesh;
            const auto &emitter_meshes = scene->getEmitter();
            float emitter_sample = sampler->next1D() * emitter_meshes.size();
            float emitter_pdf = 1.f / (float) emitter_meshes.size();
            emitter_mesh = emitter_meshes[(int) emitter_sample];

            Emitter *emitter = emitter_mesh->getEmitter();

            float light_pdf;
            Normal3f light_normal;
            Point3f light_point = emitter_mesh->sampleSurfaceUniform(sampler, light_normal, light_pdf);
            EmitterQueryRecord emitter_record(its.p, its.shFrame.n, light_point, light_normal);

            if (its.mesh == emitter_mesh)
                emitted_light += emitter_mesh->getEmitter()->getRadiance();

            light_eval += SceneUtils::getIncomingLightRadiance(emitter_record, emitter, scene);

            Vector3f l_i = light_point - its.p;
            float dist_2 = l_i.dot(l_i);
            l_i = l_i.normalized();
            float cos_theta_light = (-l_i).dot(light_normal);
            BSDFQueryRecord bsdf_record(its.toLocal(l_i), its.toLocal(-ray.d), ESolidAngle);
            Color3f bsdf_eval = its.mesh->getBSDF()->eval(bsdf_record);
            // convert probability measure
            light_pdf *= dist_2 / cos_theta_light;
            return emitted_light + light_eval / light_pdf / emitter_pdf * bsdf_eval;
        } else {
            // specular shading, reflect /  refract ray, make recursive function call and weight light path
            BSDFQueryRecord bRec(its.toLocal(-ray.d));
            Color3f ref_color = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            if (sampler->next1D() < 0.95 && ref_color.x() > 0.f) {
                return Li(scene, sampler, Ray3f(its.p, its.toWorld(bRec.wo))) / 0.95 * ref_color;
            } else {
                return {0};
            }
        }
    }

    std::string toString() const {
        return "WhittedIntegrator[]";
    }
};

    NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END