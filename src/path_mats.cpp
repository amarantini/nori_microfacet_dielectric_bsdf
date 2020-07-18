#include <nori/sampler.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

    class PathMats : public Integrator {
    public:
        PathMats(const PropertyList &props) {};

        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const override {
            /* Find the surface that is visible in the requested direction */
            Intersection its;
            Color3f light_eval(0.f);
            Color3f throughput(1.f);
            Ray3f ray(_ray);
            float eta = 1.f;
            int path_length = 0;

            // construct path while ray intersects scene
            while(scene->rayIntersect(ray, its)) {
                // if mesh is emitter, add its radiance * path contribution to evaluation sum
                if (its.mesh->isEmitter() && Frame::cosTheta(its.shFrame.toLocal(-ray.d)) > 0) {
                    light_eval += its.mesh->getEmitter()->getRadiance() * throughput;
                }

                // sample new direction for next path segment according to BSDF
                BSDFQueryRecord bRec(its.toLocal(-ray.d));
                throughput *= its.mesh->getBSDF()->sample(bRec, sampler->next2D());

                eta *= bRec.eta;

                // start russian roulette for paths with more than 3 segments
                if (path_length > 3) {
                    // calculate probability of next segment and adjust throughput weight
                    float continuation = fmin(throughput.maxCoeff() * eta * eta, 0.99f);
                    throughput /= continuation;
                    if (sampler->next1D() > continuation) break;
                }
                // create new ray
                ray = Ray3f(its.p, its.toWorld(bRec.wo));
                path_length++;
            }
            return light_eval;
        }

        std::string toString() const {
            return "PathMats[]";
        }
    };

    NORI_REGISTER_CLASS(PathMats, "path_mats");
NORI_NAMESPACE_END