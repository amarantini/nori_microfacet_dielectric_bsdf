#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AoIntegrator : public Integrator {
public:
    AoIntegrator(const PropertyList &props) {}

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {
        /* Find the surface that is visible in the requested direction */
        Intersection its;

        if (!scene->rayIntersect(ray, its))
            return {0.f};

        auto sampled_dir = Warp::squareToCosineHemisphere(sampler->next2D());
        auto sampled_dir_world = its.shFrame.toWorld(sampled_dir);

        if (scene->rayIntersect(Ray3f(its.p, sampled_dir_world)))
            return {0.f};
        return Warp::squareToCosineHemispherePdf(sampled_dir);
    }

    std::string toString() const {
        return "AoIntegrator[]";
    }

protected:
};

NORI_REGISTER_CLASS(AoIntegrator, "ao");
NORI_NAMESPACE_END
