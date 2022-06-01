/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Microfacet : public BSDF {
public:
    Microfacet(const PropertyList &propList) {
        /* RMS surface roughness */
        m_alpha = propList.getFloat("alpha", 0.1f);

        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        /* Albedo of the transmission base material (a.k.a "kt") */
        m_kt = propList.getColor("kt", Color3f(1.0f));

        /* Albedo of the reflection base material (a.k.a "kr") */
        m_kr = propList.getColor("kr", Color3f(1.0f));

        /* Albedo of the diffuse base material (a.k.a "kd") */
        // m_kd = propList.getColor("kd", Color3f(1.0f));

        /* To ensure energy conservation, we must scale the 
           specular component by 1-kd. 

           While that is not a particularly realistic model of what 
           happens in reality, this will greatly simplify the 
           implementation. Please see the course staff if you're 
           interested in implementing a more realistic version 
           of this BRDF. */
        // m_ks = 1 - m_kd.maxCoeff();
    }

    // float evalBeckmann(const Normal3f& wh) const {
    //     float temp = Frame::tanTheta(wh) / m_alpha;
    //     float ct = Frame::cosTheta(wh);
    //     float ct2 = ct*ct;

    //     return std::exp(-temp*temp)
    //            / (M_PI * m_alpha * m_alpha * ct2 * ct2);
    // }

    // float G1(const Vector3f& wv, const Vector3f& wh) const {
    //     float c = wv.dot(wh) / Frame::cosTheta(wv);
    //     if (c <= 0) return 0;
    //     float b = 1 / (m_alpha * Frame::tanTheta(wv));
    //     float b_2 = b * b;
    //     return (b < 1.6f) ? (3.535f * b + 2.181f * b_2) / (1.f + 2.276f * b + 2.577f * b_2) : 1.f;
    // }

    // /// Evaluate the BRDF for the given pair of directions
    // Color3f eval(const BSDFQueryRecord &bRec) const {
    //     Vector3f wh = (bRec.wi + bRec.wo).normalized();

    //     float D = evalBeckmann(wh);
    //     float G = G1(bRec.wi, wh) * G1(bRec.wo, wh);
    //     float F = fresnel(wh.dot(bRec.wi), m_extIOR, m_intIOR);
    //     return m_kd / M_PI + (m_ks * F * D * G) / 4.f / bRec.wi.z() / bRec.wo.z();
    // }

    // /// Evaluate the sampling density of \ref sample() wrt. solid angles
    // float pdf(const BSDFQueryRecord &bRec) const {
    //     if (bRec.wo.z() <= 0) return 0;
    //     Vector3f wh = (bRec.wi + bRec.wo).normalized();

    //     float D = evalBeckmann(wh);
    //     float jacobian = 1 / (4.f * abs(wh.dot(bRec.wo)));
    // 	return m_ks * D * Frame::cosTheta(wh) * jacobian + (1 - m_ks) * Frame::cosTheta(bRec.wo) * INV_PI;
    // }

    // /// Sample the BRDF
    // Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
    //     if (Frame::cosTheta(bRec.wi) <= 0)
    //         return {0.f};
    //     if (_sample.x() > m_ks) {
    //         // diffuse
    //         // reuse sample
    //         Point2f sample((_sample.x() - m_ks) / (1.f - m_ks), _sample.y());
    //         bRec.wo = Warp::squareToCosineHemisphere(sample);
    //     } else {
    //         // specular
    //         // reuse sample
    //         Point2f sample(_sample.x() / m_ks, _sample.y());
    //         Vector3f wh = Warp::squareToBeckmann(sample, m_alpha);
    //         bRec.wo = ((2.f * wh.dot(bRec.wi) * wh) - bRec.wi).normalized();
    //     }
    //     if (bRec.wo.z() < 0.f) return {0.f};

    //     return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
    // }

    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return false;
    }

    std::string toString() const {
        return tfm::format(
            "Microfacet[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  kt = %s,\n"
            "  kr = %f\n"
            "]",
            m_alpha,
            m_intIOR,
            m_extIOR,
            m_kt.toString(),
            m_kr.toString()
        );
    }

    //GGX implementation=====================================
    Vector3f squareToGXX(const Point2f &sample, float alpha) {
        float phi = 2 * M_PI * sample.x();
        float theta = atan(sqrt(sample.y()) * alpha / sqrt(1-sample.y()));
        return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)).normalized();
    }
    double random_uniform() const {
        return ((double)std::rand()) / RAND_MAX;
    }

    bool SameHemisphere(const Vector3f &w, const Vector3f &wp) const {
        return w.z() * wp.z() > 0;
    } 
    
    float getTheta(const Vector3f& w) const{
      return acos(fabs(clamp(w.z(), -1.0 + 1e-5, 1.0 - 1e-5)));
    }
    double Lambda (const Vector3f& w) const{
      double theta = getTheta(w);
      return (-1.0+sqrt(1.0+1.0/(m_alpha * m_alpha * pow(tan(theta),2))))/2.0;
    }
    double G1(const Vector3f &wo,const Vector3f &h) const{
        if(wo.dot(h)*wo.z()<=0) return 0.0;
        // double theta = getTheta(wo);
        float tanTheta = std::abs(Frame::tanTheta(wo));
        if (tanTheta == 0.0f)
            return 1.0f;
        double g1 = 2.0f / (1.0f + sqrt(1.0f + m_alpha * m_alpha * pow(tanTheta/*fabs(tan(theta))*/,2)));
        return g1;
        // return 1.0/(1.0+Lambda(wo));
    }
    double G(const Vector3f &wo, const Vector3f &wi, const Vector3f wh) const
    {
        // Shadowing-masking term
        // if(wh.dot(wo) * Frame::cosTheta(wo) <= 0 || wh.dot(wi) *Frame::cosTheta(wi)<=0){
        //     return 0.0;
        // }
        // return 1.0 / (1.0 + Lambda(wi) + Lambda(wo)); //Height-Correlated Masking and Shadowing
        return G1(wo,wh)*G1(wi,wh);
    }


    double D(const Normal3f &h) const
    {
        //GXX
        double cos_theta = Frame::cosTheta(h);
        if (Frame::cosTheta(h) <= 0) {
            return 0;
        }
        double theta_h = acos(cos_theta);
        return m_alpha * m_alpha /
            (M_PI * pow(cos_theta, 4) * pow(m_alpha * m_alpha + tan(theta_h) * tan(theta_h), 2));
 
    }

    float F(const Vector3f &wi, const Vector3f &wh) const
    {
        float c = wi.dot(wh);  
        float etaI = m_extIOR;
        float etaT = m_intIOR;
        if (c < 0.0f) {
            std::swap(etaI, etaT);
            c = -c;
        }
        float temp = etaT * etaT / etaI / etaI - 1 + c * c;
        if(temp<0) 
            return 1;
        float g = sqrt(temp);
        float result = 0.5 * pow((g - c), 2) / pow(g + c, 2) * (1 + pow(c*(g + c) - 1, 2) / pow(c*(g - c) + 1, 2));
        return result;
    }


    

    // Evaluate the BRDF for the given pair of directions
    Color3f eval(const BSDFQueryRecord &bRec) const {
        if(Frame::cosTheta(bRec.wi) == 0)
            return Color3f();
        bool reflect = Frame::cosTheta(bRec.wi)
            * Frame::cosTheta(bRec.wo) > 0;
        float eta = Frame::cosTheta(bRec.wi) > 0
                 ? (m_extIOR / m_intIOR) : (m_intIOR / m_extIOR);
        Vector3f wh;
        if (reflect) {
            /* Calculate the reflection half-vector */
            wh = (bRec.wo+bRec.wi).normalized();
        } else {
            /* Calculate the transmission half-vector */
            wh = (bRec.wi + bRec.wo*eta).normalized();
        }
        wh *= copysignf(1.f,Frame::cosTheta(wh));
        float d = D(wh);
        if (d==0) 
            return Color3f();
        float f = fresnel(wh.dot(bRec.wi), m_extIOR, m_intIOR);//F(bRec.wi,wh);
        float g = G(bRec.wo, bRec.wi,wh);
        
        if (reflect) {
            /* Calculate the total amount of reflection */
            float value = f * d * g /
                (4.0f * std::fabs(Frame::cosTheta(bRec.wi))* std::fabs(Frame::cosTheta(bRec.wo)));
            return m_kr * Color3f(fabs(value));
        } else {
            /* Calculate the total amount of transmission */
            float sqrtDenom = bRec.wi.dot(wh) + eta * bRec.wo.dot(wh);
            float value = ((1 - f) * d * g //* eta * eta
                * fabs(bRec.wi.dot(wh)) * fabs(bRec.wo.dot(wh))) /
                (fabs(Frame::cosTheta(bRec.wi)) * fabs(Frame::cosTheta(bRec.wo)) * sqrtDenom * sqrtDenom);

            /* Missing term in the original paper: account for the solid angle
               compression when tracing radiance -- this is necessary for
               bidirectional methods */
            float factor = eta * eta;
            return m_kt * Color3f(std::fabs(value * factor));
        }
        
    }


    /// Sample the BRDF
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
        // specular
        float cos_theta_i = Frame::cosTheta(bRec.wi);
        Vector3f wh = Warp::squareToGXX(_sample, m_alpha);
        wh.normalize();
        
        float f = fresnel(wh.dot(bRec.wi), m_extIOR, m_intIOR);//F(bRec.wi,wh);
        
        float factor;
        float p = random_uniform();
        Color3f color;
        if(p < f){ //Reflection
            wh *= copysignf(1.f,Frame::cosTheta(bRec.wi));
            bRec.wo = ((2.f * fabs(wh.dot(bRec.wi)) * wh) - bRec.wi).normalized();
            bRec.eta =  cos_theta_i >= 0 ?  m_extIOR : m_intIOR;
            factor  = 1.0;
            color = m_kr;
        } else { //Transmission
            wh *= copysignf(1.f,Frame::cosTheta(wh));
            bRec.eta = cos_theta_i >= 0 ?  m_extIOR / m_intIOR : m_intIOR / m_extIOR;

            float c = bRec.wi.dot(wh);
            float eta = cos_theta_i >= 0 ?  m_extIOR / m_intIOR : m_intIOR / m_extIOR;
            float temp = 1.0 + eta*eta * (c*c-1.0);
            if(temp<=0.0){
                bRec.wo = Vector3f();
                return 0.0;
            }
            bRec.wo = (eta * c - copysignf(1.f,Frame::cosTheta(bRec.wi)) * sqrt(temp)) * wh - eta * bRec.wi;
            bRec.wo.normalize();
            if (Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo) >= 0)
                return Color3f();

            factor = eta*eta;
            color = m_kt;
        }
        // float pdf_ = pdf(bRec);
        // if(pdf_==0) return Color3f();
        // return eval(bRec) * fabs(Frame::cosTheta(bRec.wo)) / pdf_;
        
        float g = G(bRec.wo, bRec.wi,wh);
        return color * factor * g * fabs(bRec.wi.dot(wh) /fabs(Frame::cosTheta(bRec.wi))/fabs(Frame::cosTheta(bRec.wo)));
    }

    
    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec) const { 
        bool reflect = Frame::cosTheta(bRec.wi)
            * Frame::cosTheta(bRec.wo) > 0;
        float eta = Frame::cosTheta(bRec.wi) > 0
                ? (m_extIOR / m_intIOR) : (m_intIOR / m_extIOR);
        Vector3f wh;
        float dwh_dwo;
        if (reflect) {
            /* Calculate the reflection half-vector */
            wh = (bRec.wo+bRec.wi).normalized();
            dwh_dwo = 1.0f / (4.0f * bRec.wo.dot(wh));
        } else {
            /* Calculate the transmission half-vector */
            wh = (bRec.wi + bRec.wo*eta).normalized();
            float sqrtDenom = bRec.wi.dot(wh) + eta * bRec.wo.dot(wh);
            dwh_dwo = (eta*eta * bRec.wo.dot(wh)) / (sqrtDenom*sqrtDenom);
        }
        wh *= copysignf(1.f,Frame::cosTheta(wh));
        float d = D(wh);
        float jacobian = 1 / (4.f * fabs(wh.dot(bRec.wo)) );
    	float p = d * fabs(Frame::cosTheta(wh)) * jacobian;
        float f = fresnel(wh.dot(bRec.wi), m_extIOR, m_intIOR);//F(bRec.wi,wh);
        if (reflect){
            p *= f;
        } else {
            p *= (1.0-f);
        } 
        return fabs(p * dwh_dwo);
    }

    


private:
    float m_alpha;
    float m_intIOR, m_extIOR;
    // float m_ks;
    // Color3f m_kd;
    Color3f m_kt;
    Color3f m_kr;
};

NORI_REGISTER_CLASS(Microfacet, "microfacet");
NORI_NAMESPACE_END
