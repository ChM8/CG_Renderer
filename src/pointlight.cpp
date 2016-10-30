#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class PointLight : public Emitter {
public:

	PointLight(const PropertyList &propList) {
		
		m_lightPos = propList.getPoint3("position", Point3f());
		m_power = propList.getColor("power", Color3f(1.0f));

	}

	virtual std::string toString() const override {
		return tfm::format(
			"PointLight[\n"
			"  power = %s,\n"
			"]",
			m_power.toString());
	}

	virtual Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample) const override {

		// Compute distance and (normalized) vector from light to sampler position.
		Vector3f diff = (m_lightPos - lRec.ref);
		float dis = diff.norm();
		diff.normalize();
		// Normal on point-light (imagine small sphere)
		Vector3f nEm = -diff;
		// ShadowRay (vector from ref to emitter)
		Ray3f sRay = Ray3f(lRec.ref, diff, 0.0001f, dis);

		// Set some fields of the EmitterQueryRecord
		lRec.p = m_lightPos;
		lRec.n = Normal3f(nEm);
		lRec.pdf = pdf(lRec);
		lRec.shadowRay = sRay;
		lRec.wi = diff;

		return eval(lRec)/pdf(lRec);

	}

	virtual Color3f eval(const EmitterQueryRecord &lRec) const override {
		
		// Compute the distance between the light and the sampling position
		// Result is 'Radiant Intensity/Distance^2'
		// With 'Radiant Intensity = Power/(4*pi)'

		float c = 4 * M_PI;
		// Compute randiant intensity
		Color3f radI = Color3f(m_power.x() / c, m_power.y() / c, m_power.z() / c);

		// Calculate illumination at sampler point
		Color3f illum = radI / (lRec.shadowRay.maxt * lRec.shadowRay.maxt);
		
		// return result
		return illum;

	}

	virtual float pdf(const EmitterQueryRecord &lRec) const override {
		return 1.0f;
	}



protected:

	Point3f m_lightPos;
	Color3f m_power;


};

NORI_REGISTER_CLASS(PointLight, "point")
NORI_NAMESPACE_END