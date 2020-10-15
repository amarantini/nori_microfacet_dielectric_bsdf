#include <nori/sampler.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

    class PathMis : public Integrator {
    public:
        PathMis(const PropertyList &props) {};

        float convertUnitAreaToSolidAngle(const EmitterQueryRecord& queryRecord, float pdf) const {
            Vector3f light_dir = queryRecord.shading_point - queryRecord.light_point;
            float length_squared = light_dir.dot(light_dir);
            light_dir.normalize();
            float cos_theta = abs(light_dir.dot(queryRecord.light_normal));
            return pdf * length_squared / cos_theta;
        }

        Color3f directLight(const Scene* scene, Sampler* sampler, const Intersection& its, Vector3f& wo, float &pdf) const {
            // sample emitter
            Mesh *emitter_mesh;
            const auto &emitter_meshes = scene->getEmitter();
            float emitter_sample = sampler->next1D() * emitter_meshes.size();
            float emitter_pdf = 1.f / (float) emitter_meshes.size();
            emitter_mesh = emitter_meshes[(int) emitter_sample];

            Emitter *emitter = emitter_mesh->getEmitter();

            // sample point on emitter
            float light_pdf;
            Normal3f light_normal;
            Point3f light_point = emitter_mesh->sampleSurfaceUniform(sampler, light_normal, light_pdf);
            EmitterQueryRecord emitter_record(its.p, its.shFrame.n, light_point, light_normal);


            // shadow ray query, corresponds to V(x, p)
            wo = (emitter_record.light_point - emitter_record.shading_point);
            float dist = wo.norm();
            wo.normalize();
            pdf = convertUnitAreaToSolidAngle(emitter_record, emitter_pdf * light_pdf);
            Ray3f shadow_ray = Ray3f(its.p, wo);
            shadow_ray.maxt = dist - Epsilon;
            if (scene->rayIntersect(shadow_ray))
                return {0.f};
            return emitter->evalQueryRecord(emitter_record);
        }

        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const override {
            /* Find the surface that is visible in the requested direction */
            Intersection its;
            Color3f path_contribution(0.f);
            Color3f throughput(1.f);
            Ray3f ray(_ray);
            int path_length = 0;
            // consider emission only for first ray and (completely) specular reflections to avoid double counting
            bool consider_emission = true;
            bool is_hit = scene->rayIntersect(ray, its);

            // construct path while ray intersects scene
            while(is_hit) {
                // if mesh is emitter, add its radiance * path contribution to evaluation sum
                if (consider_emission && its.mesh->isEmitter() && Frame::cosTheta(its.shFrame.toLocal(-ray.d)) > 0) {
                    path_contribution += its.mesh->getEmitter()->getRadiance() * throughput;
                }

                if (its.mesh->getBSDF()->isDiffuse()) {
                    // only compute direct light for non-specular (non-mirror, non-dielectric) materials
                    Vector3f l_wo;
                    float light_pdf;
                    Color3f direct_li = directLight(scene, sampler, its, l_wo, light_pdf);
                    BSDFQueryRecord l_bRec(its.shFrame.toLocal(-ray.d), its.shFrame.toLocal(l_wo), ESolidAngle);
                    // balance heuristic
                    direct_li *= light_pdf / (light_pdf + its.mesh->getBSDF()->pdf(l_bRec));
                    path_contribution += direct_li * its.mesh->getBSDF()->eval(l_bRec) * throughput;
                    consider_emission = false;
                } else {
                    // consider emissions in next bounce for specular reflections, because we did no NEE
                    consider_emission = true;
                }

                // sample new direction for next path segment according to BSDF
                BSDFQueryRecord bRec(its.toLocal(-ray.d));
                Color3f throughput_new_sample = its.mesh->getBSDF()->sample(bRec, sampler->next2D());

                // create new ray
                ray = Ray3f(its.p, its.toWorld(bRec.wo));

                // save old ray data
                Point3f old_p = its.p;
                Normal3f old_normal = its.shFrame.n;

                is_hit = scene->rayIntersect(ray, its);
                // balance heuristic
                if (its.mesh->isEmitter()) {
                    float bsdf_pdf = its.mesh->getBSDF()->pdf(bRec);
                    throughput_new_sample *= bsdf_pdf;
                    float light_pdf = 1.f / scene->getEmitter().size() * its.mesh->getPdf();

                    EmitterQueryRecord emitter_record(old_p, old_normal, its.p, its.shFrame.n);
                    light_pdf = convertUnitAreaToSolidAngle(emitter_record, light_pdf);
                    throughput_new_sample *= bsdf_pdf / (bsdf_pdf + light_pdf);
                }

                throughput *= throughput_new_sample;

                // start russian roulette for paths with more than 3 segments
                if (path_length > 3) {
                    // calculate probability of next segment and adjust throughput weight
                    float continuation = fmin(throughput.maxCoeff(), 0.99f);
                    throughput /= continuation;
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