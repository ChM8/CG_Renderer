#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>
#include <nori/texture.h>

NORI_NAMESPACE_BEGIN

class DisneyBRDF : public BSDF {
public:
	DisneyBRDF(const PropertyList &propList) {
		/* baseColor - default white */
		m_baseColor = propList.getColor("baseColor", Color3f(0.25f));

		/* subsurface - default 0 */
		m_subsurface = propList.getFloat("subsurface", 0.0f);

		/* metallic - default 0 */
		m_metallic = propList.getFloat("metallic", 0.0f);

		/* specular - default 0 */
		m_specular = propList.getFloat("specular", 0.5f);

		/* specularTint - default 0 */
		m_specularTint = propList.getFloat("specularTint", 0.0f);

		/* m_roughness - default 0 */
		m_roughness = propList.getFloat("roughness", 0.5f);

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
		if (Frame::cosTheta(bRec.wi) <= 0 || Frame::cosTheta(bRec.wo) <= 0) {
			//printf("Disney: Returned zero: wi.z()=%.2f - wo.z()=%.2f\n", Frame::cosTheta(bRec.wi), Frame::cosTheta(bRec.wo));
			return Color3f(0.0f);
		}
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
		float fd90 = 0.5 + 2 * cosThDH * cosThDH * m_roughness;
		float shNL = pow((1.0f - cosThNL), 5);
		float shNV = pow((1.0f - cosThNV), 5);
		//float fd = (1.0f + (fd90 - 1.0f) * pow((1.0f - cosThNL), 5)) * (1.0f + (fd90 - 1.0f) * pow((1.0f - cosThNV), 5));
		float fd = ((1.0f - shNL) + fd90 * shNL) + ((1.0f - shNV) + fd90 * shNV);

		// m_subsurface model of disney brdf - based on Hanrahan-Kruger
		float fss90 = cosThDH * cosThDH * m_roughness;
		//float fss = (1.0f + (fss90 - 1.0f) * pow((1.0f - cosThNL), 5)) * (1.0f + (fss90 - 1.0f) * pow((1.0f - cosThNV), 5)); // TODO: Correct implementation?
		float fss = ((1.0f - shNL) + fss90 * shNL) + ((1.0f - shNV) + fss90 * shNV);
		float ss = 1.25 * (fss * (1.0f / (cosThNL + cosThNV) - 0.5f) + 0.5f);

		float lum = (Color3f(0.2126f, 0.7152f, 0.0722f) * m_baseColor).sum();
		Color3f colTint = (lum > 0.0f) ? Color3f(m_baseColor.x()/lum, m_baseColor.y()/lum, m_baseColor.z()/lum) : Color3f(1.0f);
		float sFresL = pow((1.0f - cosThDH), 5);
		// Ensure that the Schlick-term is in [0,1]
		sFresL = (sFresL > 1.0f) ? 1.0f : ((sFresL < 0.0f) ? 0.0f : sFresL);
		
		// Sheen color
		Color3f colSheen = m_sheen * sFresL * linInt(Color3f(1.0f), colTint, m_sheenTint);

		// Diffuse: Linear blending of subsurface approx. (note: metallic has no diffuse) and adding sheen
		Color3f colDiff = INV_PI * m_baseColor * linInt(fd, ss, m_subsurface);
		colDiff = colDiff + colSheen;
		colDiff = colDiff * (1.0f - m_metallic); // black if full metallic

		// Specular D
		float ratSpec = 0.08 * m_specular;
		Color3f tCS = ratSpec * linInt(Color3f(1.0f), colTint, m_specularTint);
		Color3f colSpec = linInt(tCS, m_baseColor, m_metallic);
		// Primary lobe (metallic, anisotropic, GTR with g=2)
		// TODO: Anisotropy (where to compute tangent, bitangent?)
		/*float anisoAsp = sqrt(1.0f - 0.9f * m_anisotropic);
		float ax = std::max(0.001f, (m_roughness * m_roughness) / anisoAsp);
		float ay = std::max(0.001f, (m_roughness * m_roughness) * anisoAsp);*/
		/*float dt1 = 1;
		float DAnisoGTR2 = 1.0f / (M_PI * ax * ay * dt1);*/
		float tGTR2 = 1.0f + ((m_roughness * m_roughness) - 1.0f) * cosThNH * cosThNH;
		float dGTR2 = (tGTR2 != 0.0f) ? (m_roughness * m_roughness) / (M_PI * tGTR2 * tGTR2) : 0.0f;
		// Secondary lobe, clearcoat (isotropic, GTR with g=1)
		float factCC = linInt(0.1f, 0.001f, m_clearcoatGloss);
		float tGTR1 = 1.0f + ((factCC * factCC) - 1.0f) * cosThNH * cosThNH;
		float dGTR1 = ((factCC * factCC) - 1.0f) / (M_PI * log((factCC * factCC)) * tGTR1);

		// Specular F
		Color3f specF = linInt(colSpec, Color3f(1.0f), sFresL);
		float ccF = linInt(0.04f, 1.0f, sFresL);

		// Specular G
		// Specular
		float gR2 = (m_roughness * 0.5f + 0.5f) * (m_roughness * 0.5f + 0.5f);
		float tGGX1 = 1.0f / (cosThNL + sqrt(gR2 + (cosThNL * cosThNL) - gR2 * (cosThNL * cosThNL)));
		float tGGX2 = 1.0f / (cosThNV + sqrt(gR2 + (cosThNV * cosThNV) - gR2 * (cosThNV * cosThNV)));
		float specG = tGGX1 * tGGX2;
		// Clearcoat
		float a2 = 0.25f * 0.25f;
		tGGX1 = 1.0f / (cosThNL + sqrt(a2 + (cosThNL * cosThNL) - a2 * (cosThNL * cosThNL)));
		tGGX2 = 1.0f / (cosThNV + sqrt(a2 + (cosThNV * cosThNV) - a2 * (cosThNV * cosThNV)));
		float ccG = tGGX1 * tGGX2;

		// Combine the results of the different aspects (diffuse, specular, clearcoat)
		Color3f res = colDiff + (dGTR2 * specF * specG) + 0.25f * m_clearcoat * (dGTR1 * ccF * ccG);
		//printf("Disney: Returned color: %.2f, %.2f, %.2f\n", res.x(), res.y(), res.z());
		if (res.x() != res.x())
			printf("Result of DISNEY is NAN! GTR2:%.2f with %.2f\n", dGTR2, tGTR2);
		return res;
		return 0.0f;
	}

	// Linearly interpolate from float value1 to value2, as defined by factor in [0,1]
	virtual float linInt(float value1, float value2, float factor) const {
		return (value1 * (1.0f - factor) + value2 * factor);
	}
	// Linearly interpolate from color value1 to value2, as defined by factor in [0,1]
	virtual Color3f linInt(const Color3f value1, const Color3f value2, const float factor) const {
		return (value1 * (1.0f - factor) + value2 * factor);
	}

	virtual float pdf(const BSDFQueryRecord &bRec) const override {
		if (Frame::cosTheta(bRec.wi) <= 0 || Frame::cosTheta(bRec.wo) <= 0)
			return 0.0f;

		// compute half-vector (sampled it, not wi/wo)
		Vector3f vh = (bRec.wi + bRec.wo) / (bRec.wi + bRec.wo).norm();
		float cosT = Frame::cosTheta(vh);

		// The chosen pdf is (as recommended in the paper) pdf_h = D(theta_h) * cos(theta_h) with different parameters
		// which describe the different layers (diffuse, specular, clearcoat)

		// The total probability of the given sample is the pdf of the specific layer weighted by the probability of sampling it (\ref sample())
		// The weights sum up to 1.0f
		float wCC = 0.25f * m_clearcoat;
		float wS = m_metallic / (1.0f - 0.25f * m_clearcoat);
		float wD = 1.0f - (wCC + wS);

		float alpha = m_roughness * m_roughness;
		float f1 = alpha * alpha * INV_PI;
		float f2 = (1.0f + (alpha * alpha - 1.0f) * cosT * cosT);
		// f2 is only 0 when roughness is 0 and half vector equals the normal
		float GTR2pdf = (f2 != 0.0f) ? (f1 / (f2 * f2)) * cosT : 1.0f;
		GTR2pdf = (GTR2pdf > 1.0f) ? 1.0f : GTR2pdf;

		alpha = linInt(0.1f, 0.001f, m_clearcoatGloss);
		f1 = (alpha * alpha - 1.0f) / (M_PI * std::log(alpha * alpha));
		f2 = (1.0f + (alpha * alpha - 1.0f) * cosT * cosT);
		float GTR1pdf = (f1 / f2) * cosT;

		float res = wD * (INV_PI * Frame::cosTheta(bRec.wo)) + wS * GTR2pdf + wCC * GTR1pdf;
		//printf("Disney: Returned pdf: %.4f\n", res);
		if (res != res)
			printf("DISNEY pdf is NAN! %.2f - %.2f\n", f1, f2);
		return res;
	}

	virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {
		if (Frame::cosTheta(bRec.wi) <= 0)
			return Color3f(0.0f);

		bRec.measure = ESolidAngle;

		Point2f rnd = Point2f(sample.x(), sample.y());

		// Depending on the different parameters describing the material, the direction is sampled from
		// either the diffuse, specular or clearcoat layer according to the distributions of each.

		// Test if sampling clearcoat (following the influence of the parameter on the brdf value)
		if (rnd.x() < m_clearcoat * 0.25f) {
			// Readjust the sample.x() (is in [0, 0.25f * m_clearcoat]
			rnd.x() = (rnd.x() / (0.25f * m_clearcoat));

			// Sampling clearcoat layer
			// (Secondary lobe with GTR1, specific alpha)
			Vector3f wh = Warp::squareToGTR1(rnd, linInt(0.1f, 0.001f, m_clearcoatGloss));
			// Get the light-vector by mirroring the view-vector on the received half-vector
			bRec.wo = ((2.f * wh.dot(bRec.wi) * wh) - bRec.wi).normalized();

			// Relative index of refraction: no change
			bRec.eta = 1.0f;

			// Check if direction above/below surface
			if (Frame::cosTheta(bRec.wo) <= 0) {
				// Below surface - reject sample by returning 0
				return Color3f(0.0f);
			}

		}
		else {
			// readjust rnd.x()
			rnd.x() = ((rnd.x() - (0.25f * m_clearcoat)) / (1.0f - (0.25f * m_clearcoat)));


			// Sampling diffuse or specular - check which one
			if (rnd.x() < m_metallic) {
				// Sampling metallic
				// Adjust sample
				rnd.x() = (rnd.x() / (m_metallic));

				// Sample from the distribution described by the brdf's parameters
				// (GTR2 distribution)
				// If roughness is 0, halfvector is normal for proper reflection
				Vector3f wh = Vector3f(0.0f, 0.0f, 0.1);
				if (m_roughness != 0.0f) {
					wh = Warp::squareToGTR2(rnd, m_roughness * m_roughness);
				}
				// Get the light-vector by mirroring the view-vector on the received half-vector
				bRec.wo = ((2.f * wh.dot(bRec.wi) * wh) - bRec.wi).normalized();

				// Relative index of refraction: no change
				bRec.eta = 1.0f;

				// Check if direction above/below surface
				if (Frame::cosTheta(bRec.wo) <= 0) {
					// Below surface - reject sample by returning 0
					return Color3f(0.0f);
				}

			}
			else {
				// Sampling diffuse
				// Adjust rnd.x()
				rnd.x() = ((rnd.x() - m_metallic) / (1.0f - m_metallic));

				// Sample from a uniform hemisphere
				bRec.wo = Warp::squareToCosineHemisphere(sample);

				// Relative index of refraction: no change
				bRec.eta = 1.0f;

			}
		}

		// Return brdf
		float pdfS = pdf(bRec);
		float cT = Frame::cosTheta(bRec.wo);

		if (pdfS > 0.0f && cT >= 0.0f) {
			return (eval(bRec) * cT) / pdfS;
		}
		else {
			return Color3f(0.0f);
		}

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