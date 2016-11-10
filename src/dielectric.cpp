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

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dielectric : public BSDF {
public:
    Dielectric(const PropertyList &propList) {
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);
    }

    virtual Color3f eval(const BSDFQueryRecord &) const override {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    virtual float pdf(const BSDFQueryRecord &) const override {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

    virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {
		if (Frame::cosTheta(bRec.wi) <= 0)
			return Color3f(0.0f);

		// Check if reflection or refraction is sampled
		float frCoeff = fresnel(Frame::cosTheta(bRec.wi), m_extIOR, m_intIOR);
		bool bRfrac = (sample.x() <= frCoeff);

		if (bRfrac) {
			// Sampling refraction

			// Compute exitant vector according to Snell's law
			float relIOR = m_extIOR / m_intIOR; // TODO: Ensure, that intIOR is on side of exitant vector (new medium)
			Vector3f n = Vector3f(0.f, 0.f, 1.f);

			Vector3f tmp1 = -relIOR * bRec.wi - bRec.wi.z() * n;
			Vector3f tmp2 = n * sqrt(1 - relIOR * relIOR * (1 - bRec.wi.z()) * (1 - bRec.wi.z()));

			bRec.wo = tmp1 - tmp2;
			bRec.measure = EDiscrete;

			bRec.eta = m_intIOR;

			return Color3f(1.0f);

		}
		else {
			// Sampling reflection

			// Reflection in local coordinates (same as in mirror.cpp)
			bRec.wo = Vector3f(-bRec.wi.x(), -bRec.wi.y(), bRec.wi.z());
			bRec.measure = EDiscrete;

			bRec.eta = 1.0f;

			return Color3f(1.0f);

		}


    }

    virtual std::string toString() const override {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }
private:
    float m_intIOR, m_extIOR;
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END
