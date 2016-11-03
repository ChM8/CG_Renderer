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

		// Sample according to the pdf of the brdf of this shape's surface
		// TODO: Check why wo is set in diffuse! If wrong (wi should be set instead), change wi to wo here!
		// Build BSDFQuery
		BSDFQueryRecord bsdfRec = BSDFQueryRecord(itsM.toLocal(-ray.d));
		bsdfRec.uv = itsM.uv;
		// sample
		Color3f bsdfRes = objBSDF->sample(bsdfRec, sampler->next2D());
		// Use the sample direction in world-space for casting a ray
		Vector3f woWC = itsM.toWorld(bsdfRec.wo);
		Ray3f sampleRay = Ray3f(p, woWC);

		// Check for an intersection of the the ray in the sampled direction (from BSDF)
		Intersection itsSh;

		if (scene->rayIntersect(sampleRay, itsSh)) {
			// Intersection of the sampling ray - check if it is an emitter

			if (itsSh.mesh->isEmitter()) {
				// Compute contribution of this emitter
				EmitterQueryRecord lRec = EmitterQueryRecord(p, itsSh.p, itsSh.shFrame.n);
				Color3f incRad = itsSh.mesh->getEmitter()->eval(lRec);

				// Angle between shading normal and direction to emitter
				float cosThetaIn = n.dot(woWC) / (n.norm() * woWC.norm());

				if (cosThetaIn >= 0) {
					// Compute addition of the incoming radiance of this emitter (already divided by pdf in bsdf->sample())
					Color3f addRad = (incRad * bsdfRes * cosThetaIn);
					exRad += addRad;
					if (addRad.x() < 0 || addRad.y() < 0 || addRad.z() < 0) {
						printf("Negative radiance at %.2f, %.2f, %.2f\n", p.x(), p.y(), p.z());
					}
				}

			}
			// else: Not an emitter. Since no indirect illumination is computed, no light received.
		}
		// else: No collision in sample direction. No light received from this direction.
		
		// Add emitted radiance from this mesh (if emitter)
		if (itsM.mesh->isEmitter()) {
			EmitterQueryRecord lRec = EmitterQueryRecord(p);
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