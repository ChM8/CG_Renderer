#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class DisneyBRDF : public BSDF {
public:
	DisneyBRDF(const PropertyList &propList) {
		/* baseColor - default white */
		m_baseColor = propList.getColor("baseColor", Color3f(0.0f));

		/* subsurface - default 0 */
		m_subsurface = propList.getFloat("subsurface", 0.0f);

		/* metallic - default 0 */
		m_metallic = propList.getFloat("metallic", 0.0f);

		/* specular - default 0 */
		m_specular = propList.getFloat("specular", 0.0f);

		/* specularTint - default 0 */
		m_specularTint = propList.getFloat("specularTint", 0.0f);

		/* m_roughness - default 0 */
		m_roughness = propList.getFloat("m_roughness", 0.0f);

		/* anisotropic - default 0 */
		m_anisotropic = propList.getFloat("anisotropic", 0.0f);

		/* sheen - default 0 */
		m_sheen = propList.getFloat("sheen", 0.0f);

		/* sheenTint - default 0 */
		m_sheenTint = propList.getFloat("sheenTint", 0.0f);

		/* clearcoat - default 0 */
		m_clearcoat = propList.getFloat("clearcoat", 0.0f);

		/* clearcoatGloss - default 0 */
		m_clearcoatGloss = propList.getFloat("clearcoatGloss", 0.0f);
	}

	virtual Color3f eval(const BSDFQueryRecord &bRec) const override {
		if (Frame::cosTheta(bRec.wi) <= 0
		|| Frame::cosTheta(bRec.wo) <= 0)
			return Color3f(0.0f);

		// Normal is 0,0,1 in the local coordinates
		Vector3f n = Vector3f(0.0f, 0.0f, 1.0f);
		Vector3f h = (bRec.wi + bRec.wo).normalized();
		// cos(Theta_NH) - of angle between half-vector and normal
		float cosThNH = n.dot(h);
		// cos(Theta_NL) - of angle between light-vector and normal
		float cosThNL = Frame::cosTheta(bRec.wo);
		// cos(Theta_NV) - of angle between view-vector and normal
		float cosThNV = Frame::cosTheta(bRec.wi);
		// cos(Theta_D) - of angle between half-vector and light/view-vector
		float cosThDH = bRec.wi.dot(h) / (bRec.wi.norm());

		// Diffuse part
		float FD90 = 0.5 + 2 * cosThDH * cosThDH * m_roughness;
		float FD = (1 + (FD90 - 1) * pow((1 - cosThNL), 5)) * (1 + (FD90 - 1) * pow((1 - cosThNV), 5));

		// m_subsurface model of disney brdf - based on Hanrahan-Kruger
		float FSS90 = cosThDH * cosThDH * m_roughness;
		float FSs = (1 + (FSS90 - 1) * pow((1 - cosThNL), 5)) * (1 + (FSS90 - 1) * pow((1 - cosThNV), 5)); // TODO: Correct implementation?

		Color3f diffuse = INV_PI * m_baseColor * (FD * (1 - m_subsurface) + FSs * m_subsurface);

		throw NoriException("TO IMPLEMENT!");
	}

	virtual float pdf(const BSDFQueryRecord &bRec) const override {
		throw NoriException("TO IMPLEMENT!");
	}

	virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {

		throw NoriException("TO IMPLEMENT!");

	}

	virtual std::string toString() const override {
		return tfm::format(
			"DisneyBRDF[\n"
			"  baseColor = %f %f %f,\n"
			"  subsurface = %f\n"
			"  metallic = %f\n"
			"  specular = %f\n"
			"  specularTint = %f\n"
			"  roughness = %f\n"
			"  anisotropic = %f\n"
			"  sheen = %f\n"
			"  sheenTint = %f\n"
			"  clearcoat = %f\n"
			"  clearcoatGloss = %f\n"
			"]",
			m_baseColor.x(), m_baseColor.y(), m_baseColor.z(), m_subsurface, m_metallic, m_specular, m_specularTint, m_roughness, m_anisotropic, m_sheen, m_sheenTint, m_clearcoat, m_clearcoatGloss);
	}
private:
	Color3f m_baseColor;
	float m_subsurface, m_metallic, m_specular, m_specularTint, m_roughness, m_anisotropic, m_sheen, m_sheenTint, m_clearcoat, m_clearcoatGloss;
};

NORI_REGISTER_CLASS(DisneyBRDF, "disney");
NORI_NAMESPACE_END