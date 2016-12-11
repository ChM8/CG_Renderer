#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/common.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class PathMisIntegrator : public Integrator {
public:
	PathMisIntegrator(const PropertyList &props) {
		if (props.has("renderEmitter")) {
			m_bRenderEmitter = props.getBoolean("renderEmitter");
		}
	}

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {

		// Define exitant radiance (black default)
		Color3f exRad = Color3f(0.5f);

		// Initial Throughput
		Color3f t = Color3f(1.f);
		float wMat = 1.0f;
		float matsBSDFPdf = 0.0f;
		bool isDelta = false;
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

			// MAT Sampling
			// Intersection -> emitter?
			Color3f matsEmEval = Color3f(0.0f);
			float matsEmPdf = 0.0f;
			if (itsM.mesh->isEmitter()) {

				if (it == 0 && !m_bRenderEmitter) {
					// First hit is an emitter (that should not be rendered).
					// Adjust the ray so that this intersection won't come up again.
					// And retrace...
					sRay.mint = itsM.t + 0.01f;
					continue;
				}

				// Get exitant light in direction of the current ray
				const Emitter* matsEm = itsM.mesh->getEmitter();

				// Create an EmitterQueryRecord
				EmitterQueryRecord matsLRec = EmitterQueryRecord(sRay.o, p, n);

				matsEmEval = matsEm->eval(matsLRec);
				matsEmPdf = matsEm->pdf(matsLRec);

				// Compute the wMat of the sampled BSDF
				wMat = 0.0f;
				if (isDelta || it == 0) {
					wMat = 1.0f;
				}
				else if (matsEmPdf + matsBSDFPdf != 0.0f) {
					wMat = matsBSDFPdf / (matsEmPdf + matsBSDFPdf);
				}
			}
			else if (it > 0){
				wMat = 0.0f;
			}

			// Emitter sampling
			// Choose a random emitter to sample
			const Emitter* emsEm = scene->getRandomEmitter(sampler->next1D());
			float probEm = 1.0f / scene->getLights().size();

			// Create an EmitterQueryRecord
			EmitterQueryRecord emsLRec = EmitterQueryRecord(itsM.p);
			// Sample the randomly chosen emitter and get the BSDF value (TODO: SolidAngle?)
			Color3f emsEmS = emsEm->sample(emsLRec, sampler->next2D());
			float emsEmPdf = emsLRec.pdf * probEm;
			BSDFQueryRecord emsBSDFRec = BSDFQueryRecord(itsM.shFrame.toLocal(-sRay.d), itsM.shFrame.toLocal(-emsLRec.wi), ESolidAngle);
			Color3f emsBSDFRes = objBSDF->eval(emsBSDFRec);
			float emsBSDFPdf = objBSDF->pdf(emsBSDFRec);

			// Compute Weight and add Radiance
			float wEm = 0.0f;
			if (!isDelta && (emsBSDFPdf != 0.0f || emsEmPdf != 0.0f)) {
				wEm = emsEmPdf / (emsBSDFPdf + emsEmPdf);
			}
			//printf("Emitter: BSDF pdf: %.2f, Emitter pdf: %.2f (%.2f * %.2f) -> wEm: %.2f\n", emsBSDFPdf, emsEmPdf, emsEm->pdf(emsLRec), probEm, wEm);

			// Ajdust weights to sum up to one
			if ((wMat + wEm != 1.0f) && (wMat != 0.0f || wEm != 0.0f)) {
				float fac = 1 / (wMat + wEm);
				wMat *= fac;
				wEm *= fac;
			}

			exRad += wEm * t.cwiseProduct(emsEmS.cwiseProduct(emsBSDFRes));

			// Add the mats-contribution previously computed
			exRad += wMat * t.cwiseProduct(matsEmEval);

			if (!exRad.isValid())
				printf("Not valid exRad!\n");

			
			// Success-probability is the throughput (decreasing with the contribution)
			float succProb = (it >= minIt) ? std::min(t.maxCoeff(), 0.999f) : 1.0f;
			if (sampler->next1D() >= succProb) {
				// Failed in Russian Roulette - break path-tracing
				break;
			}
			// Else, continue path-tracing
			t /= succProb;

			// Sample a new direction from the bsdf from the current position
			// Build BSDFQuery
			BSDFQueryRecord bsdfRec = BSDFQueryRecord(itsM.toLocal(-sRay.d));
			bsdfRec.uv = itsM.uv;
			// sample
			Color3f bsdfRes = objBSDF->sample(bsdfRec, sampler->next2D());
			matsBSDFPdf = objBSDF->pdf(bsdfRec);
			// Use the sample direction in world-space for casting a ray
			Vector3f woWC = itsM.toWorld(bsdfRec.wo);
			sRay = Ray3f(itsM.p, woWC);

			// Check if BSDF at this position is delta
			isDelta = bsdfRec.measure == EDiscrete;

			// Adjust throughput according to current BSDF / pdf of sample
			// (bsdfRes from sample is already divided by PDF)
			float cosThetaInWo = (n.norm() * woWC.norm() != 0.0f) ? n.dot(woWC) / (n.norm() * woWC.norm()) : 0.0f;
			t = t.cwiseProduct(bsdfRes);// * cosThetaInWo;
			if (t.x() != t.x())
				printf("T is NAN!\n");
			it++;
		}
		
		if (!exRad.isValid())
			printf("Not valid exRad!\n");

		return exRad;

	}

	std::string toString() const {
		return "Path_Mats[]";
	}
protected:
	float rayLength;
	bool m_bRenderEmitter;
};

NORI_REGISTER_CLASS(PathMisIntegrator, "path_mis");
NORI_NAMESPACE_END