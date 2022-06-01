#include <nori/sampler.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/weighting.h>
#include <nori/scene_utils.h>

NORI_NAMESPACE_BEGIN

    class PathMis : public Integrator {
    public:
        PathMis(const PropertyList &props) {};

        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const override {
            /* Find the surface that is visible in the requested direction */
            Intersection its;
            Color3f path_contribution(0.f);
            Color3f path_throughput(1.f);
            Ray3f ray(_ray);
            int path_length = 0;
            bool is_hit = scene->rayIntersect(ray, its);

            // special case if the first hit is an emitter, all other emitters are considered via sampling
            if (is_hit && its.mesh->isEmitter() && Frame::cosTheta(its.shFrame.toLocal(-ray.d)) > 0) {
                path_contribution += path_throughput * its.mesh->getEmitter()->getRadiance();
            }

            while (is_hit) {
                Vector3f wi = -ray.d;
                // Light importance sampling
                if (its.mesh->getBSDF()->isDiffuse()) {
                    float light_pdf;
                    Emitter *emitter;
                    EmitterQueryRecord emitter_record =
                            SceneUtils::sampleLightSource(scene, sampler, its.p, its.shFrame.n, emitter,light_pdf);
                    Color3f incoming_radiance = {0.f};
                    if (light_pdf > Epsilon) {
                        incoming_radiance = SceneUtils::getIncomingLightRadiance(emitter_record, emitter, scene) / light_pdf;
                    } 

                    Vector3f wo = (emitter_record.light_point - its.p).normalized();
                    BSDFQueryRecord bsdf_record(its.shFrame.toLocal(wi), its.shFrame.toLocal(wo), ESolidAngle);
                    Color3f bsdf_throughput = its.mesh->getBSDF()->eval(bsdf_record);
                    float bsdf_pdf = its.mesh->getBSDF()->pdf(bsdf_record);
                    float weight = Weighting::power2Heuristic(1, light_pdf, 1, bsdf_pdf);
                    path_contribution += path_throughput * incoming_radiance * bsdf_throughput * weight;
                }

                // BSDF importance sampling
                BSDFQueryRecord bsdf_record = BSDFQueryRecord(its.toLocal(wi));
                path_throughput *= its.mesh->getBSDF()->sample(bsdf_record, sampler->next2D());
                ray = Ray3f(its.p, its.toWorld(bsdf_record.wo));
                Intersection shading_its = its;
                is_hit = scene->rayIntersect(ray, its);

                if (is_hit && its.mesh->isEmitter()) {
                    EmitterQueryRecord emitter_record = EmitterQueryRecord(shading_its.p, shading_its.shFrame.n, its.p, its.shFrame.n);
                    float light_pdf = SceneUtils::getLightPdf(scene, its.mesh, emitter_record);
                    Color3f incoming_radiance = SceneUtils::getIncomingLightRadiance(emitter_record, its.mesh->getEmitter(), scene);
                    float bsdf_pdf = shading_its.mesh->getBSDF()->pdf(bsdf_record);
                    float weight = 1.f;
                    if (shading_its.mesh->getBSDF()->isDiffuse()) {
                        weight = Weighting::power2Heuristic(1, bsdf_pdf, 1, light_pdf);
                    }
                    path_contribution += path_throughput * incoming_radiance * weight;
                }

                // start russian roulette for paths with more than 3 segments
                if (path_length > 3) {
                    // calculate probability of next segment and adjust throughput weight
                    float continuation = fmin(path_throughput.maxCoeff(), 0.99f);
                    path_throughput /= continuation;
                    if (sampler->next1D() > continuation) break;
                }
                path_length++;
            }

            return path_contribution;
        }

        std::string toString() const {
            return "PathMis[]";
        }
    };

    NORI_REGISTER_CLASS(PathMis, "path_mis");
NORI_NAMESPACE_END