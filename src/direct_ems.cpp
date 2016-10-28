#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/common.h>

NORI_NAMESPACE_BEGIN

class DirectEMSIntegrator : public Integrator {
public:
	DirectEMSIntegrator(const PropertyList &props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {

		// Define exitant radiance (black default)
		Color3f exRad = Color3f(0.0f);

		/* Find the surface that is visible in the requested direction */
		Intersection itsM;
		if (!scene->rayIntersect(ray, itsM)) {
			// No intersection, return black (no light received)
			return exRad;
		}

		// There is an intersection, get the point and the colliding shape
		Point3f p = itsM.p;
		Normal3f n = itsM.shFrame.n;
		const BSDF * objBSDF = itsM.mesh->getBSDF();

		// Iterate over all lights in the scene and get the incoming, direct radiance
		std::vector<Emitter *> lights = scene->getLights();

		// Prepare the EmitterQueryRecord (same 'ref' for all)
		EmitterQueryRecord lRec;
		Color3f sumIncRad = Color3f(0.0f);

		for (std::vector<Emitter *>::iterator it = lights.begin(); it != lights.end(); ++it) {

			lRec = EmitterQueryRecord(p);

			// Query the current emitter
			Emitter* em = *it;

			// Get a random sample
			Point2f sample = Point2f((((float)rand()) / RAND_MAX), (((float)rand()) / RAND_MAX));
			Color3f incRad = em->sample(lRec, sample);

			// Check for an intersection of the shadow ray on the way to the light
			Intersection itsSh;
			scene->rayIntersect(lRec.shadowRay, itsSh);
			// Ensure that the collision is not caused by the emitter itself (only the area between)
			if (lRec.shadowRay.maxt - itsSh.t <= 0.00001f) {
				// No intersection, point fully visible from emitter
				
				// Build BSDFQuery
				BSDFQueryRecord bsdfRec = BSDFQueryRecord(itsM.toLocal(-lRec.wi), itsM.toLocal(-ray.d), ESolidAngle);
				bsdfRec.uv = itsM.uv;
				// Angle between shading normal and direction to emitter
				float cosThetaIn = n.dot(lRec.shadowRay.d) / (n.norm() * lRec.shadowRay.d.norm());
				float cosThetaOut = lRec.n.dot(lRec.shadowRay.d) / (lRec.n.norm() * lRec.shadowRay.d.norm());
				// Compute addition of this emitter to the whole incoming (Radiance / pdf) (brdf * emition in sample direction * cos(theta))
				sumIncRad = sumIncRad + (incRad) * objBSDF->eval(bsdfRec) * abs(cosThetaIn) * cosThetaOut / (lRec.ref - lRec.p).norm() * (lRec.ref - lRec.p).norm();

			}
			// Else: Collision with an object, in shadow from this emitter. Add nothing
			// to exitant radiance.
		}

		// Add emitted radiance from this mesh (if emitter)
		exRad = sumIncRad;
		if (itsM.mesh->isEmitter()) {
			lRec = EmitterQueryRecord(p);
			// Add only value evaluated at this emitter-object (and not divided by pdf as in Emitter::sample())
			exRad += itsM.mesh->getEmitter()->eval(lRec);
		}

		return exRad;

	}

	std::string toString() const {
		return "Direct_EMS[]";
	}
protected:
	float rayLength;
};

NORI_REGISTER_CLASS(DirectEMSIntegrator, "direct_ems");
NORI_NAMESPACE_END