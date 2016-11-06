/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Romain Prévost

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

#include <nori/shape.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Sphere : public Shape {
public:
    Sphere(const PropertyList &propList) {
        m_position = propList.getPoint3("center", Point3f());
        m_radius = propList.getFloat("radius", 1.f);

        m_bbox.expandBy(m_position - Vector3f(m_radius));
        m_bbox.expandBy(m_position + Vector3f(m_radius));
    }

    virtual BoundingBox3f getBoundingBox(uint32_t index) const override { return m_bbox; }

    virtual Point3f getCentroid(uint32_t index) const override { return m_position; }

    virtual bool rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const override {

		// Some variables used for calculating the intersection (variables defining eq.)
		float a = ray.d.squaredNorm();
		float b = 2 * (ray.o.cwiseProduct(ray.d).sum() - m_position.cwiseProduct(ray.d).sum());
		float c = m_position.squaredNorm() - (2 * m_position.cwiseProduct(ray.o).sum()) + ray.o.squaredNorm() - m_radius * m_radius;
		// Calculating the discriminant
		float disc = b * b - 4 * a * c;

		if (disc >= 0) {
			// Positive discriminant - there is an intersection, solve for t
			Vector2f tCalc = Vector2f((-b + sqrt(disc)) / (2 * a), (-b - sqrt(disc)) / (2 * a));

			// Find the smallest positive result
			float tmin = fmin(tCalc.x(), tCalc.y());
			float tmax = fmax(tCalc.x(), tCalc.y());
			float ttemp = -1.f;

			if (tmin >= 0) {
				ttemp = tmin;
			}
			else if (tmax >= 0) {
				ttemp = tmax;
			}

			// Check if t of intersection (if any) is in valid range of the ray
			if (ttemp >= 0 && (ttemp >= ray.mint && ttemp <= ray.maxt)) {
				return true;
			}

			// No positive intersection (only one on the 'backside' of the ray or outside 't')
			return false;
		}
		else {
			// Negative discriminant - no intersection
			return false;
		}
    }

    virtual void setHitInformation(uint32_t index, const Ray3f &ray, Intersection & its) const override {
        // Called upon hit on this sphere (so there is a valid intersection); compute it.
		float a = ray.d.squaredNorm();
		float b = 2 * (ray.o.cwiseProduct(ray.d).sum() - m_position.cwiseProduct(ray.d).sum());
		float c = m_position.squaredNorm() - (2 * m_position.cwiseProduct(ray.o).sum()) + ray.o.squaredNorm() - m_radius * m_radius;
		float disc = b * b - 4 * a * c;
		Vector2f tCalc = Vector2f((-b + sqrt(disc)) / (2 * a), (-b - sqrt(disc)) / (2 * a));
		float tmin = fmin(tCalc.x(), tCalc.y());
		float tmax = fmax(tCalc.x(), tCalc.y());
		// Set the solution for t (smallest positive)
		if (tmin >= 0) {
			its.t = tmin;
		}
		else if (tmax >= 0) {
			its.t = tmax;
		}

		// Compute the position of the intersection
		its.p = ray.o + (its.t * ray.d);

		// Compute normal for frame at this point on the sphere
		Vector3f n = its.p - m_position;
		n.normalize();
		// Set the frames
		its.geoFrame = Frame(n);
		its.shFrame = Frame(n);
		
		// Compute the spherical coordinates of the intersection point (for uvs)
		// CAUTION: Assuming "x left->right, z back->front, y bottom->top"
		Point3f lP = its.p - m_position;

		float u = atan2f(lP.y(), lP.x()) / (2 * M_PI) + 0.5f;
		float v = asin(lP.z() / m_radius) * INV_PI + 0.5f;
		
		its.uv = Point2f(u, v);

    }

    virtual void sampleSurface(ShapeQueryRecord & sRec, const Point2f & sample) const override {
        Vector3f q = Warp::squareToUniformSphere(sample);
        sRec.p = m_position + m_radius * q;
        sRec.n = q;
<<<<<<< HEAD
		sRec.pdf = std::pow(1.f / m_radius, 2) * Warp::squareToUniformSpherePdf(Vector3f(0.0f, 0.0f, 1.0f));
    }
    virtual float pdfSurface(const ShapeQueryRecord & sRec) const override {
        return std::pow(1.f / m_radius, 2) * Warp::squareToUniformSpherePdf(Vector3f(0.0f, 0.0f, 1.0f));
=======
        sRec.pdf = std::pow(1.f/m_radius,2) * Warp::squareToUniformSpherePdf(Vector3f(0.0f,0.0f,1.0f));
    }
    virtual float pdfSurface(const ShapeQueryRecord & sRec) const override {
        return std::pow(1.f/m_radius,2) * Warp::squareToUniformSpherePdf(Vector3f(0.0f,0.0f,1.0f));
>>>>>>> c08176a639b9e4159d3e44226146f8470cab2fa6
    }


    virtual std::string toString() const override {
        return tfm::format(
                "Sphere[\n"
                "  center = %s,\n"
                "  radius = %f,\n"
                "  bsdf = %s,\n"
                "  emitter = %s\n"
                "]",
                m_position.toString(),
                m_radius,
                m_bsdf ? indent(m_bsdf->toString()) : std::string("null"),
                m_emitter ? indent(m_emitter->toString()) : std::string("null"));
    }

protected:
    Point3f m_position;
    float m_radius;
};

NORI_REGISTER_CLASS(Sphere, "sphere");
NORI_NAMESPACE_END
