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

#include <nori/emitter.h>
#include <nori/warp.h>
#include <nori/shape.h>

NORI_NAMESPACE_BEGIN

class AreaEmitter : public Emitter {
public:
    AreaEmitter(const PropertyList &props) {
        m_radiance = props.getColor("radiance");
    }

    virtual std::string toString() const override {
        return tfm::format(
                "AreaLight[\n"
                "  radiance = %s,\n"
                "]",
                m_radiance.toString());
    }

    virtual Color3f eval(const EmitterQueryRecord &lRec) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

		// Check if 'ref'-point lies on positive side of emitter-surface (use vector
		// from to intersection point to emitter)
		if (lRec.n.dot(lRec.ref - lRec.p) >= 0.0f) {
			return m_radiance;
		}
		else {
			return Color3f(0.0f);
		}

    }

	virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const override {
		if (!m_shape)
			throw NoriException("There is no shape attached to this Area light!");

		// Sample a point on the emitter shape (get normals, pdf, ...)
		ShapeQueryRecord sRec = ShapeQueryRecord(lRec.ref);
		m_shape->sampleSurface(sRec, sample);
		lRec.p = sRec.p;

		if (sRec.n.dot(lRec.ref - lRec.p) < 0.0f) {
			return Color3f(0.0f);
		}

		// Compute distance and (normalized) vector from light to sampler position.
		Vector3f diff = (sRec.p - lRec.ref);
		float dis = diff.norm();
		diff.normalize();
		// ShadowRay (vector from ref to emitter)
		Ray3f sRay = Ray3f(lRec.ref, diff, 0.0001f, dis);

		// Set some fields of the EmitterQueryRecord
		lRec.n = sRec.n;
		lRec.shadowRay = sRay;
		lRec.wi = diff;

		float cT = lRec.n.dot(-lRec.wi) / (lRec.n.norm() * lRec.wi.norm());
		if (cT > 0) {
			lRec.pdf = sRec.pdf * (dis * dis) / cT;
		}
		else {
			lRec.pdf = 0.0f;
		}

		if (lRec.pdf > 0.0f) {
			return eval(lRec) / lRec.pdf;
		}
		else {
			return Color3f(0.0f);
		}

        
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

		ShapeQueryRecord sRec = ShapeQueryRecord(lRec.ref, lRec.p);
		m_shape->pdfSurface(sRec);

		float dis = (lRec.p - lRec.ref).norm();
		float cT = lRec.n.dot(-lRec.wi) / (lRec.n.norm() * lRec.wi.norm());
		if (cT > 0) {
			return sRec.pdf * (dis * dis) / cT;
		}
		else {
			return 0.0f;
		}
    }


    virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const override {
        // Returns the exitant radius multplied by M_PI and the surface area

		// First, sample a point on the mesh (uniformly) - reference is a dummy point
		ShapeQueryRecord sRec = ShapeQueryRecord(Point3f(0.0f));
		m_shape->sampleSurface(sRec, sample1);

		// Get a random sample on the hemisphere around the sampled point (looking upwards towards 0,0,1)
		Vector3f dir = Warp::squareToCosineHemisphere(sample2);

		// Transform the direction to the world coordinates at the sampled point
		Vector3f sDirWC = Frame(sRec.n).toWorld(dir);

		// Create the photon-ray
		ray.d = sDirWC;
		ray.o = sRec.p;
		ray.mint = FLT_EPSILON;
		ray.maxt = FLT_MAX;

		// Get the value of the light at the sampled position (with a dummy-reference point - in case if emition is not uniform)
		EmitterQueryRecord lRec = EmitterQueryRecord(sRec.p + sDirWC, sRec.p, sRec.n);
		
		Color3f exRad = eval(lRec);

		return exRad * M_PI; // TODO: Surface area?
    }


protected:
    Color3f m_radiance;
};

NORI_REGISTER_CLASS(AreaEmitter, "area")
NORI_NAMESPACE_END