#include <nori/object.h>
#include <nori/phasefunction.h>
#include <nori/henyey_greenstein.h>

NORI_NAMESPACE_BEGIN

HenyeyGreenstein::HenyeyGreenstein(float g)
{
	m_g = g;
}

float HenyeyGreenstein::eval(float cosTh)
{
	// The same
	return pdf(cosTh);
}

Vector3f HenyeyGreenstein::sample(Point2f sample) {
	float th = 0.0f;
	if (m_g != 0.0f) {
		float g2 = m_g*m_g;
		float t1 = 1.0f / (2.0f * m_g);
		float t2 = (1.0f - g2) / (1.0f - m_g + 2.0f * m_g * sample.x());
		// Clamp value (if there was some error and cosTh >1/<-1)
		float cosTh = clamp(t1 * (1.0f + g2 - t2 * t2), -1.0f, 1.0f);
		th = acos(cosTh);
		/*if (th != th)
			printf("Dir is nan! g=%.2f, x=%.2f, t1=%.2f, t2=%.2f, cosTh=%.2f, th=%.2f\n", m_g, sample.x(), t1, t2, cosTh, th);*/
	}
	else {
		// isotropic
		th = 2.0f * M_PI * sample.x();
	}
	float phi = 2.0f * M_PI * sample.y();

	float x = sin(th) * sin(phi);
	float y = sin(th) * cos(phi);
	float z = cos(th);
	return Vector3f(x, y, z);
}

float HenyeyGreenstein::pdf(float cosTh) {
	float g2 = m_g * m_g;
	float t1 = 1.0f + g2 - 2.0f * m_g * cosTh;
	return INV_FOURPI * (1.0f - g2) / (sqrt(t1 * t1 * t1));
}


NORI_NAMESPACE_END