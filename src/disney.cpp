#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class DisneyBRDF : public BSDF {
public:
	DisneyBRDF(const PropertyList &propList) {
		/* Interior IOR (default: BK7 borosilicate optical glass) */
		m_intIOR = propList.getFloat("intIOR", 1.5046f);

		/* Exterior IOR (default: air) */
		m_extIOR = propList.getFloat("extIOR", 1.000277f);
	}

	virtual Color3f eval(const BSDFQueryRecord &) const override {
		throw NoriException("TO IMPLEMENT!");
	}

	virtual float pdf(const BSDFQueryRecord &) const override {
		throw NoriException("TO IMPLEMENT!");
	}

	virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {

		throw NoriException("TO IMPLEMENT!");

	}

	virtual std::string toString() const override {
		return tfm::format(
			"DisneyBRDF[\n"
			"  intIOR = %f,\n"
			"  extIOR = %f\n"
			"]",
			m_intIOR, m_extIOR);
	}
private:
	float m_intIOR, m_extIOR;
};

NORI_REGISTER_CLASS(DisneyBRDF, "disney");
NORI_NAMESPACE_END