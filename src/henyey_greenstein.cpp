#include <nori/object.h>
#include <nori/phasefunction.h>
#include <nori/henyey_greenstein.h>

NORI_NAMESPACE_BEGIN

HenyeyGreenstein::HenyeyGreenstein(float g)
{
	m_g = g;
}

Vector3f HenyeyGreenstein::sample(Point2f sample) {
	float g = (m_g == 0.0f) ? 0.0001 : m_g;
	printf("m_g=%.2f", m_g);
	float g2 = g*g;
	float t1 = 1.0f / (2.0f * g);
	float t2 = (1.0f - g2) / (1.0f - g + 2.0f * g * sample.x());
	float th = acos(t1 * (1.0f + g2 - t2 * t2));
	float phi = 2.0f * M_PI * sample.y();

	float x = sin(th) * sin(phi);
	float y = sin(th) * cos(phi);
	float z = cos(th);
	if (x != x || y != y)
		printf("Dir is nana! g=%.2f, t1=%.2f, t2=%.2f, th=%.2f\n", g, t1, t2, th);
	return Vector3f(x, y, z);
}

float HenyeyGreenstein::pdf(float cosTh) {
	float g2 = m_g * m_g;
	float t1 = 1.0f + g2 - 2.0f * m_g * cosTh;
	return INV_FOURPI * (1.0f - g2) / (sqrt(t1 * t1 * t1));
}


NORI_NAMESPACE_END