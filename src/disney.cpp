#include <nori/bsdf.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

class DisneyBRDF : public BSDF {
public:
	DisneyBRDF(const PropertyList &propList) {
		/* baseColor - default white */
		m_baseColor = propList.getColor("baseColor", Color3f(0.5f));

		/* subsurface - default 0 */
		m_subsurface = propList.getFloat("subsurface", 0.0f);

		/* metallic - default 0 */
		m_metallic = propList.getFloat("metallic", 0.0f);

		/* specular - default 0 */
		m_specular = propList.getFloat("specular", 0.5f);

		/* specularTint - default 0 */
		m_specularTint = propList.getFloat("specularTint", 0.0f);

		/* m_roughness - default 0 */
		m_roughness = propList.getFloat("m_roughness", 0.5f);

		/* anisotropic - default 0 */
		m_anisotropic = propList.getFloat("anisotropic", 0.0f);

		/* sheen - default 0 */
		m_sheen = propList.getFloat("sheen", 0.0f);

		/* sheenTint - default 0 */
		m_sheenTint = propList.getFloat("sheenTint", 0.5f);

		/* clearcoat - default 0 */
		m_clearcoat = propList.getFloat("clearcoat", 0.0f);

		/* clearcoatGloss - default 0 */
		m_clearcoatGloss = propList.getFloat("clearcoatGloss", 0.1f);
	}

	virtual Color3f eval(const BSDFQueryRecord &bRec) const override {
		if (Frame::cosTheta(bRec.wi) <= 0 || Frame::cosTheta(bRec.wo) <= 0)
			return Color3f(0.0f);

		// Implemented according to the paper "Physically-Based Shading at Disney" from Brent Burley

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
		float FD = (1.0f + (FD90 - 1.0f) * pow((1.0f - cosThNL), 5)) * (1.0f + (FD90 - 1.0f) * pow((1.0f - cosThNV), 5));

		// m_subsurface model of disney brdf - based on Hanrahan-Kruger
		float FSS90 = cosThDH * cosThDH * m_roughness;
		float FSS = (1.0f + (FSS90 - 1.0f) * pow((1.0f - cosThNL), 5)) * (1.0f + (FSS90 - 1.0f) * pow((1.0f - cosThNV), 5)); // TODO: Correct implementation?
		float SS = 1.25 * (FSS * (1.0f / (cosThNL + cosThNV) - 0.5f) + 0.5f);

		Color3f colTint = m_baseColor;
		float sFresL = pow((1.0f - cosThDH), 5);
		// Ensure that the Schlick-term is in [0,1]
		sFresL = (sFresL > 1.0f) ? 1.0f : ((sFresL < 0.0f) ? 0.0f : sFresL);
		
		// Sheen
		Color3f colSheen = m_sheen * sFresL * (Color3f(1.0f) * (1.0f - m_sheenTint) + (colTint * m_sheenTint));

		// Linear blending of subsurface approx. (note: metallic has no diffuse) and adding sheen
		Color3f colDiff = ((INV_PI * m_baseColor * (FD * (1.0f - m_subsurface) + SS * m_subsurface)) + colSheen) * (1.0f - m_metallic);

		// Specular D
		float ratSpec = 0.08 * m_specular;
		Color3f colSpec = ((Color3f(1.0f) * (m_specularTint - 1.0f) + colTint * m_specularTint) * (1.0f - m_metallic)) + (m_baseColor * m_metallic);
		// Primary lobe (metallic, anisotropic, GTR with g=2)
		// TODO: Anisotropy
		float anisoAsp = sqrt(1.0f - 0.9f * m_anisotropic);
		float ax = std::max(0.001f, (m_roughness * m_roughness) / anisoAsp);
		float ay = std::max(0.001f, (m_roughness * m_roughness) * anisoAsp);
		/*float dt1 = 1;
		float DAnisoGTR2 = 1.0f / (M_PI * ax * ay * dt1);*/
		float r2 = m_roughness * m_roughness;
		float tGTR2 = 1.0f + (r2 - 1) * cosThNH * cosThNH;
		float dGTR2 = r2 / (M_PI * tGTR2 * tGTR2);
		// Secondary lobe (clearcoat, isotropic, GTR with g=1)
		float factCC = 0.1f * (1.0f - m_clearcoatGloss) + 0.001f;
		float f2 = factCC * factCC;
		float tGTR1 = 1.0f + (f2 - 1.0f) * cosThNH * cosThNH;
		float dGTR1 = (f2 - 1.0f) / (M_PI * log(f2) * tGTR1);

		// Specular F
		Color3f specF = (colSpec * (1.0f - sFresL)) + (Color3f(1.0f) * sFresL);
		float ccF = (0.04f * (1.0f - sFresL)) + (1.0f * sFresL);

		// Specular G
		float gR2 = (m_roughness * 0.5f + 0.5f) * (m_roughness * 0.5f + 0.5f);
		float tGGX1 = 1.0f / (cosThNL + sqrt(gR2 + (cosThNL * cosThNL) - gR2 * (cosThNL * cosThNL)));
		float tGGX2 = 1.0f / (cosThNV + sqrt(gR2 + (cosThNV * cosThNV) - gR2 * (cosThNV * cosThNV)));
		float specG = tGGX1 * tGGX2;
		float a2 = 0.25f * 0.25f;
		tGGX1 = 1.0f / (cosThNL + sqrt(a2 + (cosThNL * cosThNL) - a2 * (cosThNL * cosThNL)));
		tGGX2 = 1.0f / (cosThNV + sqrt(a2 + (cosThNV * cosThNV) - a2 * (cosThNV * cosThNV)));
		float ccG = tGGX1 * tGGX2;

		// Combine the results of the different aspects (diffuse, specular, clearcoat)
		return colDiff + (dGTR2 * specF * specG) + 0.25f * m_clearcoat * (dGTR1 * ccF * ccG);
	}

	virtual float pdf(const BSDFQueryRecord &bRec) const override {
		if (Frame::cosTheta(bRec.wi) <= 0 || Frame::cosTheta(bRec.wo) <= 0)
			return 0.0f;

		// The chosen pdf is (as recommended in the paper) pdf_h = D(theta_h) * cos(theta_h)
		


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