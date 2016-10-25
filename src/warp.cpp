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

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Vector3f Warp::sampleUniformHemisphere(Sampler *sampler, const Normal3f &pole) {
    // Naive implementation using rejection sampling
    Vector3f v;
    do {
        v.x() = 1.f - 2.f * sampler->next1D();
        v.y() = 1.f - 2.f * sampler->next1D();
        v.z() = 1.f - 2.f * sampler->next1D();
    } while (v.squaredNorm() > 1.f);

    if (v.dot(pole) < 0.f)
        v = -v;
    v /= v.norm();

    return v;
}

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
	return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && ((sample.array() < 1).all())) ? 1.0f : 0.0f;
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
	float r = sqrt(sample.x());
	float th = 2 * M_PI * sample.y();

	return Point2f(r * cos(th), r * sin(th));
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    return (p.x() * p.x() + p.y() * p.y()) <= 1.0f ? INV_PI : 0.0f;
}

Vector3f Warp::squareToUniformSphereCap(const Point2f &sample, float cosThetaMax) {
    
	float z = (1 - cosThetaMax) * sample.x() + cosThetaMax;
	float th = 2 * M_PI * sample.y();
	float r = sqrt(1 - z * z);
	float x = r * cos(th);
	float y = r * sin(th);

	return Vector3f(x, y, z);

}

float Warp::squareToUniformSphereCapPdf(const Vector3f &v, float cosThetaMax) {
	float diff = abs(v.x()*v.x() + v.y()*v.y() + v.z()*v.z() - 1.0f);
    return ((diff <= 0.0001f) && (v.z() >= cosThetaMax)) ? 1/(2*M_PI*(1-cosThetaMax)) : 0.0f;
}

Vector3f Warp::squareToUniformCylinder(const Point2f &sample) {

	float th = 2 * M_PI * sample.y();
	float r = 1;
	
	float z = 2 * sample.x() - 1;
	float x = r * cos(th);
	float y = r * sin(th);

	return Vector3f(x, y, z);

}

float Warp::squareToUniformCylinderPdf(const Vector3f &v) {
	return (abs(v.x()*v.x() + v.y()*v.y() - 1.0f) <= 0.001f) ? INV_TWOPI : 0.0f;
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {

	float z = 2 * sample.x() - 1;
	float th = 2 * M_PI * sample.y();
	float r = sqrt(1 - z * z);
	float x = r * cos(th);
	float y = r * sin(th);

	return Vector3f(x, y, z);

}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
	float diff = abs((v.x() * v.x() + v.y() * v.y() + v.z() * v.z()) - 1.0f);
	return (diff <= 0.0001f) ? INV_FOURPI : 0.0f;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    
	float z = sample.x();
	float th = 2 * M_PI * sample.y();
	float r = sqrt(1 - z * z);
	float x = r * cos(th);
	float y = r * sin(th);

	return Vector3f(x, y, z);
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
	float diff = abs(v.x()*v.x() + v.y()*v.y() + v.z()*v.z() - 1.0f);
	return (diff <= 0.0001f) && (v.z() >= 0) ? INV_TWOPI : 0.0f;
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    
	float r = sqrt(sample.x());
	float th = 2 * M_PI * sample.y();

	float x = r * cos(th);
	float y = r * sin(th);
	float t = sqrt(x*x + y*y);
	float z = sqrt(1 - t*t);

	return Vector3f(x, y, z);
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
	float diff = abs(v.x()*v.x() + v.y()*v.y() + v.z()*v.z() - 1.0f);
	float th = acos(Vector3f(0, 0, 1).dot(v.normalized()));
	return (diff <= 0.001f) && (v.z() >= 0) ? cos(th)*INV_PI : 0;
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    
	float phi = 2 * M_PI*sample.y();
	float th = atan(sqrt(-(alpha*alpha)*log(sample.x())));

	float x = sin(th) * cos(phi);
	float y = sin(th) * sin(phi);
	float z = cos(th);

	return Vector3f(x, y, z);
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
	float diff = abs(m.x()*m.x() + m.y()*m.y() + m.z()*m.z() - 1.0f);
	float th = acos(Vector3f(0, 0, 1).dot(m.normalized()));
	if ((diff <= 0.0001f) && (m.z() >= 0)) {
		return exp(-(tan(th)*tan(th)) / (alpha*alpha)) / (M_PI * alpha * alpha * pow(cos(th), 3));
	}
	else {
		return 0.0f;
	}
}

Vector3f Warp::squareToUniformTriangle(const Point2f &sample) {
    float su1 = sqrtf(sample.x());
    float u = 1.f - su1, v = sample.y() * su1;
    return Vector3f(u,v,1.f-u-v);
}

NORI_NAMESPACE_END
