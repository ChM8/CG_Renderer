#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/common.h>
#include <nori/sampler.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

// Smaller struct needed for more convenient computations
struct MedT
{
	MediaContainer * m;
	float t;

	MedT(MediaContainer * m, float t) : m(m), t(t) {}
};

class PathMisHPMIntegrator : public Integrator {
public:
	PathMisHPMIntegrator(const PropertyList &props) {
		if (props.has("renderEmitter")) {
			m_bRenderEmitter = props.getBoolean("renderEmitter");
		}
		else {
			m_bRenderEmitter = true;
		}
	}

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {

		// Define exitant radiance (black default)
		Color3f exRad = Color3f(0.0f);

		// Initial Throughput
		Color3f t = Color3f(1.f);
		float wMat = 1.0f;
		float matsBSDFPdf = 0.0f;
		bool isDelta = false;
		// Minimum Iterations until Russian Roulette kicks in
		int it = 0;
		int minIt = 3;

		// Prepare for Participating Media (already in a volume?)
		bool inMedia = false;
		std::vector<MediaContainer *> currMedia;
		MediaContainer sampledMedia;
		std::vector<MediaContainer *> allMedia = scene->getMediaContainers();
		for (MediaContainer * m : allMedia) {
			// Check if already within a hpm-container
			if (m->withinContainer(ray.o)) {
				currMedia.push_back(m);
				inMedia = true;
			}
		}
		

		// Define a container for the ray that is sampled currently
		Ray3f sRay = ray;
		// Path-Tracing until break
		while (true) {
			
			/* Find the surface that is visible in the requested direction */
			Intersection itsM;

			bool hitGeom = scene->rayIntersect(sRay, itsM);

			// Check if a media_container is hit
			bool hitPOI = false;
			float tMed = sRay.maxt;
			// Use a placeholder for the new state (current containers)
			std::vector<MedT> tempRmv, tempNew;

			for (MediaContainer * m : allMedia) {
				Intersection itsMedium;
				if (m->setHitInformation(sRay, itsMedium)) {
					// Check if entering a medium/leaving a container
					bool inList = false;
					for (MediaContainer * c : currMedia) {
						if (c->getName().compare(m->getName())) {
							// Apparently leaving one - check if the one that is currently sampled
							if (m->getName().compare(sampledMedia.getName())) {
								// Make sure we don't just leave without any computations
								hitPOI = true;
								if (itsMedium.t <= tMed)
									tMed = itsMedium.t;
							}
							inList = true;
							// Add to candidates for removing
							tempRmv.push_back(MedT(m, itsMedium.t));
						}
					}

					// If not yet in the list -> entered a new volume
					if (!inList) {
						// Add to the candidates for adding to the current volumes
						tempNew.push_back(MedT(m, itsMedium.t));
						hitPOI = true;
						if (itsMedium.t <= tMed)
							tMed = itsMedium.t;
					}

				}
			}

			// Check candidates and adjust vector currMedia
			// TODO

			if (!(hitGeom || hitPOI || inMedia )) {
				// No intersection, not within any medium and therefore no incoming light
				break;
			}
			else if (hitGeom && (itsM.t <= tMed)) {
				// Hit Geometry before Medium POI 

			}
			else {
				// Medium POI or sampled distance reached
			}

			if (inMedia) {
				// Distance sampling in one of the medias

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
		return "Path_Mis_HPM[]";
	}
protected:
	float rayLength;
	bool m_bRenderEmitter;
};

NORI_REGISTER_CLASS(PathMisHPMIntegrator, "path_mis_hpm");
NORI_NAMESPACE_END