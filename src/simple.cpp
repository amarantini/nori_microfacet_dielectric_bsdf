#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class SimpleIntegrator : public Integrator {
public:
    SimpleIntegrator(const PropertyList &props) {
        m_position = props.getPoint("position");
        m_energy = props.getColor("energy");
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {
        /* Find the surface that is visible in the requested direction */
        Intersection its;

        if (!scene->rayIntersect(ray, its))
            return {0.0f};


        Vector3f l = m_position - its.p;
        float dist = l.squaredNorm();
        l.normalize();

        // shadow ray query, corresponds to V(x, p)
        if (scene->rayIntersect(Ray3f(its.p, l)))
            return {0.0f};

        float cos_theta = l.dot(its.shFrame.n);
        return m_energy / (4 * M_PI * M_PI) * fmax(0, cos_theta) / dist;
    }

    std::string toString() const {
        return "SimpleIntegrator[ position: " + m_position.toString() + ", energy: " + m_energy.toString() + " ]";
    }

protected:
    Point3f m_position;
    Color3f m_energy;
};

    NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END
