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
		// from 'ref' to intersection point)
		if (lRec.n.dot(lRec.ref - lRec.p) >= 0.0f) {
			return m_radiance;
		}
		else {
			return Color3f(0.0f);
		}

    }

    virtual Color3f sample(EmitterQueryRecord & lRec, const Point2f & sample) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

		// Sample a point on the emitter shape (get normals, pdf, ...)
		ShapeQueryRecord sRec = ShapeQueryRecord(lRec.ref);
		m_shape->sampleSurface(sRec, sample);
		lRec.p = sRec.p;

		// Compute distance and (normalized) vector from light to sampler position.
		Vector3f diff = (sRec.p - lRec.ref);
		float dis = diff.norm();
		diff.normalize();
		// ShadowRay (vector from ref to emitter)
		Ray3f sRay = Ray3f(lRec.ref, diff, 0.0001f, dis);

		// Set some fields of the EmitterQueryRecord
		lRec.n = sRec.n;
		lRec.pdf = sRec.pdf;
		lRec.shadowRay = sRay;
		lRec.wi = -diff;

		return eval(lRec) / lRec.pdf;
        
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const override {
        if(!m_shape)
            throw NoriException("There is no shape attached to this Area light!");

		// Get the probability from the shape of the emitter. (If valid hit, i.e. on front-side)
		if (lRec.n.dot(lRec.wi) >= 0.0f) {
			return m_shape->pdfSurface(lRec.p);
		}
		else {
			return 0.0f;
		}
    }


    virtual Color3f samplePhoton(Ray3f &ray, const Point2f &sample1, const Point2f &sample2) const override {
        throw NoriException("To implement...");
    }


protected:
    Color3f m_radiance;
};

NORI_REGISTER_CLASS(AreaEmitter, "area")
NORI_NAMESPACE_END