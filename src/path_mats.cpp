#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/common.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class PathMatsIntegrator : public Integrator {
public:
	PathMatsIntegrator(const PropertyList &props) {
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

		// Initial Throughput
		Color3f t = Color3f(1.f);
		int it = 0;
		int minIt = 3;


		// Sample according to the pdf of the brdf of this shape's surface
		// Build BSDFQuery
		BSDFQueryRecord bsdfRec = BSDFQueryRecord(itsM.toLocal(-ray.d));
		bsdfRec.uv = itsM.uv;
		// sample
		Color3f bsdfRes = objBSDF->sample(bsdfRec, sampler->next2D());
		// Use the sample direction in world-space for casting a ray
		Vector3f woWC = itsM.toWorld(bsdfRec.wo);
		Ray3f sampleRay = Ray3f(p, woWC);

		while (true) {

			// Check for an intersection of the the ray in the sampled direction (from BSDF)
			Intersection itsSh;

			if (scene->rayIntersect(sampleRay, itsSh)) {
				// Intersection of the sampling ray - check if it is an emitter

				if (itsSh.mesh->isEmitter()) {
					// Compute contribution of this emitter
					EmitterQueryRecord lRec = EmitterQueryRecord(p, itsSh.p, itsSh.shFrame.n);
					Color3f incRad = itsSh.mesh->getEmitter()->eval(lRec);

					// Compute addition of the incoming radiance of this emitter (already divided by pdf in bsdf->sample())
					Color3f addRad = (incRad.cwiseProduct(bsdfRes));
					exRad += t.cwiseProduct(addRad);

					if (t.hasNaN() || addRad.hasNaN()) {
						printf("t or addRad NAN!\n");
					}

					if (t.cwiseProduct(addRad).x() <= 0) {
						printf("negative rad!\n");
					}

					// Found an emitter. Stop sampling
					//break;
				}
				// Continue with new sample from this position (after playing Russian Roulette)
				
				float probSuccess = std::min(t.maxCoeff(), 0.999f);

				if (sampler->next1D() <= probSuccess && it >= minIt) {
					break;
				}

				// If success, adjust throughput
				t /= probSuccess;

				// Get a new sample direction from the current bsdf
				// Build BSDFQuery
				bsdfRec = BSDFQueryRecord(itsSh.toLocal(-sampleRay.d));
				bsdfRec.uv = itsSh.uv;
				objBSDF = itsSh.mesh->getBSDF();
				// sample
				bsdfRes = objBSDF->sample(bsdfRec, sampler->next2D());
				// Use the sample direction in world-space for casting a ray
				woWC = itsSh.toWorld(bsdfRec.wo);
				sampleRay = Ray3f(itsSh.p, woWC);

				// Adjust throughput again. TODO: Check if w_o as sampled direction is correct
				t *= itsSh.shFrame.n.dot(woWC) * bsdfRes;


			}
			// else: No collision in sample direction. No light received from this direction.
			else {
				break;
			}

		}

		// Add emitted radiance from this mesh (if emitter)
		if (itsM.mesh->isEmitter()) {
			EmitterQueryRecord lRec = EmitterQueryRecord(p);
			// Add only value evaluated at this emitter-object (and not divided by pdf as in Emitter::sample())
			exRad += itsM.mesh->getEmitter()->eval(lRec);
		}

		if (!exRad.isValid()) {
			printf("Error! Invalid Radiance at %.2f, %.2f, %.2f\n", p.x(), p.y(), p.z());
			printf("    Exrad: %.2f, %.2f, %.2f\n", exRad.x(), exRad.y(), exRad.z());
		}
		return exRad;

	}

	std::string toString() const {
		return "Path_Mats[]";
	}
protected:
	float rayLength;
};

NORI_REGISTER_CLASS(PathMatsIntegrator, "path_mats");
NORI_NAMESPACE_END