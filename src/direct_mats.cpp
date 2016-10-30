#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/common.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class DirectMatsIntegrator : public Integrator {
public:
	DirectMatsIntegrator(const PropertyList &props) {
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

		// TODO
		// Sample according to the pdf of the brdf of this shape's surface


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
			Point2f sample = sampler->next2D();
			Color3f incRad = em->sample(lRec, sample);

			// Check for an intersection of the shadow ray on the way to the light
			Intersection itsSh;
			bool bColl = scene->rayIntersect(lRec.shadowRay, itsSh);
			// Ensure that the collision is not caused by the emitter itself (only the area between)
			bool bCurrEmitterIntersect = bColl && (lRec.shadowRay.maxt - itsSh.t <= 0.0001f);

			if (!bColl || bCurrEmitterIntersect) {
				// No intersection, point fully visible from emitter

				// Build BSDFQuery
				BSDFQueryRecord bsdfRec = BSDFQueryRecord(itsM.toLocal(lRec.wi), itsM.toLocal(-ray.d), ESolidAngle);
				bsdfRec.uv = itsM.uv;
				// Angle between shading normal and direction to emitter
				float cosThetaIn = n.dot(lRec.wi) / (n.norm() * lRec.wi.norm());
				float cosThetaOut = lRec.n.dot(-lRec.shadowRay.d) / (lRec.n.norm() * lRec.shadowRay.d.norm());
				if (cosThetaIn >= 0 && cosThetaOut >= 0) {
					// Compute addition of the incoming radiance of this emitter
					float dis = (lRec.ref - lRec.p).norm();
					Color3f bsdfRes = objBSDF->eval(bsdfRec);
					Color3f addRad = (incRad * bsdfRes * cosThetaIn * cosThetaOut) / (dis * dis);
					sumIncRad += addRad;
					if (addRad.x() < 0 || addRad.y() < 0 || addRad.z() < 0) {
						printf("Negative radiance at %.2f, %.2f, %.2f\n", p.x(), p.y(), p.z());
					}
				}
				// Else: Emitter-Sample not directly visible, add nothing
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

		if (!exRad.isValid()) {
			printf("Error! Invalid Radiance at %.2f, %.2f, %.2f\n", p.x(), p.y(), p.z());
		}

		return exRad;

	}

	std::string toString() const {
		return "Direct_Mats[]";
	}
protected:
	float rayLength;
};

NORI_REGISTER_CLASS(DirectMatsIntegrator, "direct_mats");
NORI_NAMESPACE_END