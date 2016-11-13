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

		// Initial Throughput
		Color3f t = Color3f(1.f);
		// Minimum Iterations until Russian Roulette kicks in
		int it = 0;
		int minIt = 3;

		// Define a container for the ray that is sampled currently
		Ray3f sRay = ray;

		// Path-Tracing until break
		while (true) {
			
			/* Find the surface that is visible in the requested direction */
			Intersection itsM;

			if (!scene->rayIntersect(sRay, itsM)) {
				// No intersection and therefore no incoming light
				break;
			}

			// Get the intersection point and the BSDF there
			Point3f p = itsM.p;
			Normal3f n = itsM.shFrame.n;
			const BSDF * objBSDF = itsM.mesh->getBSDF();

			// There is an intersection - emitter?
			if (itsM.mesh->isEmitter()) {
				// Get exitant light in direction of the current ray
				const Emitter* em = itsM.mesh->getEmitter();

				// Create an EmitterQueryRecord
				EmitterQueryRecord lRec = EmitterQueryRecord(sRay.o, p, n);
				// Get the incident radiance from this emitter (exitant times throughput)
				exRad += t.cwiseProduct(em->eval(lRec)); //* itsM.shFrame.n.dot(-sRay.d);

			}
			
			// Success-probability is the throughput (decreasing with the contribution)
			float succProb = std::min(t.maxCoeff(), 0.999f);
			if (sampler->next1D() >= succProb) {
				// Failed in Russian Roulette - break path-tracing
				break;
			}

			// Else, continue path-tracing

			// Sample a new direction from the bsdf from the current position
			// Build BSDFQuery
			BSDFQueryRecord bsdfRec = BSDFQueryRecord(itsM.toLocal(-sRay.d));
			bsdfRec.uv = itsM.uv;
			// sample
			Color3f bsdfRes = itsM.mesh->getBSDF()->sample(bsdfRec, sampler->next2D());
			// Use the sample direction in world-space for casting a ray
			Vector3f woWC = itsM.toWorld(bsdfRec.wo);
			sRay = Ray3f(itsM.p, woWC);

			// Adjust throughput according to current BSDF / pdf of sample
			// (bsdfRes from sample is already divided by PDF)
			t = t.cwiseProduct(bsdfRes);

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