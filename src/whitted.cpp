#include <nori/sampler.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class WhittedIntegrator : public Integrator {
public:
    WhittedIntegrator(const PropertyList &props) {};

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        Color3f light_eval(0.f);

        if (!scene->rayIntersect(ray, its))
            return {0.0f};


        Mesh *emitter_mesh;
        const auto& emitter_meshes = scene->getEmitter();
        if (emitter_meshes.size() != 1)
            throw NoriException("Multiple/no mesh emitter not supported");
        emitter_mesh = emitter_meshes.front();

        Emitter *emitter = emitter_mesh->getEmitter();

        float pdf;
        Normal3f light_normal;
        Point3f light_point = emitter_mesh->sampleSurfaceUniform(sampler->next1D(), sampler, light_normal, pdf);
        EmitterQueryRecord emitter_record(its.p, its.shFrame.n, light_point, light_normal);

        if (its.mesh == emitter_mesh)
            light_eval += emitter_mesh->getEmitter()->getRadiance();


        // shadow ray query, corresponds to V(x, p)
        Vector3f wi = (emitter_record.light_point - emitter_record.shading_point);
        float dist = wi.norm();
        wi.normalize();
        Ray3f shadow_ray = Ray3f(its.p, wi);
        shadow_ray.maxt = dist;
        if (scene->rayIntersect(shadow_ray))
            return light_eval;
        light_eval += emitter->evalQueryRecord(emitter_record);

        BSDFQueryRecord bsdf_record(its.toLocal(wi), its.toLocal(-ray.d), ESolidAngle);
        Color3f bsdf_eval = its.mesh->getBSDF()->eval(bsdf_record);
        return  light_eval / pdf * bsdf_eval;
    }

    std::string toString() const {
        return "WhittedIntegrator[]";
    }
};

    NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END