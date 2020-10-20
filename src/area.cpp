#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class AreaLight : public Emitter {

public:
    explicit AreaLight(const PropertyList &props) {
        m_radiance = props.getColor("radiance");
    }

    Color3f evalQueryRecord(const EmitterQueryRecord &record) const override {
        Vector3f x_y = record.light_point - record.shading_point;
        Vector3f l_i = (x_y).normalized();
        float cos_theta_i = l_i.dot(record.shading_normal);
        if (cos_theta_i <= 0)
            return {0.f};
        return m_radiance * cos_theta_i;
    }

    Color3f getRadiance() const override {
        return m_radiance;
    }

    std::string toString() const override {
        return "AreaLight[ radiance: " + m_radiance.toString() + "]";
    }

private:
    Color3f m_radiance;
};

    NORI_REGISTER_CLASS(AreaLight, "area");
NORI_NAMESPACE_END
