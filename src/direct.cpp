#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/common.h>

NORI_NAMESPACE_BEGIN

class DirectIntegrator : public Integrator {
public:
	DirectIntegrator(const PropertyList &props) {
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

		// Iterate over all lights in the scene and get the incoming radiance
		std::vector<Emitter *> lights = scene->getLights();

		// Prepare the EmitterQueryRecord (same 'ref' for all - rest will be overwritten)
		EmitterQueryRecord lRec = EmitterQueryRecord(p);

		for (std::vector<Emitter *>::iterator it = lights.begin(); it != lights.end(); ++it) {

			// Query the current emitter
			Emitter* em = *it;
			Color3f incRad = em->sample(lRec, Point2f(0.0f));

			// Check if sampled successfully
			// TODO!
			
			// Check for an intersection of the shadow ray on the way to the light
			Intersection itsSh;
			if (!scene->rayIntersect(lRec.shadowRay, itsSh)) {
				// No intersection, point fully visible from emitter
				// Multiple incident radiance from this light with BSDF and cos, 
				// add to total exitant radiance.

				// Build BSDFQuery
				BSDFQueryRecord bsdfRec = BSDFQueryRecord(itsM.toLocal(-lRec.wi) , itsM.toLocal(-ray.d), ESolidAngle);
				bsdfRec.uv = itsM.uv;
				// Angle between shading normal and direction to emitter (both normalized)
				float theta = acos(n.dot(lRec.shadowRay.d) / (n.norm() * lRec.shadowRay.d.norm()));
				exRad = exRad + (incRad) * objBSDF->eval(bsdfRec) * abs(cos(theta));

			}
			// Else: Collision with an object, in shadow from this emitter. Add nothing
			// to exitant radiance.
		}

		return exRad;

	}

	std::string toString() const {
		return "AverageVisibility[]";
	}
protected:
	float rayLength;
};

NORI_REGISTER_CLASS(DirectIntegrator, "direct");
NORI_NAMESPACE_END