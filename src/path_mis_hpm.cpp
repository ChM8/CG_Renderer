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
		// Initial Transmittance (no extinction)
		float tr = 1.0f;
		float wMat = 1.0f;
		float matsBSDFPdf = 0.0f;
		bool isDelta = false;
		// Minimum Iterations until Russian Roulette kicks in
		int it = 0;
		int itTr = 0;
		int minIt = 3;

		// Prepare for Participating Media (already in a volume?)
		bool inMedia = false;
		std::vector<MediaContainer *> currMedia;
		MediaContainer * sampledMedia;
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

		// But if already in media, adjust the maximum ray length according to the sampling distance (woodcock)
		if (inMedia)
			sRay.maxt = sampleDistance(currMedia, sRay.o, sampler->next2D(), sampledMedia);

		// Path-Tracing until break
		while (true) {

			// Play Russian Roulette with the Transmittance (else without geometry, infinite loop)
			float succTr = (itTr >= minIt) ? std::min(tr, 0.999f) : 1.0f;
			if (sampler->next1D() >= succTr) {
				// Failed in Russian Roulette - break path-tracing
				itTr = 0;
				break;
			}
			// Else, continue path-tracing
			tr /= succTr;
			if (tr != tr)
				printf("tr is nan!\n");
			itTr++;
			
			/* Find the surface that is visible in the requested direction */
			Intersection itsM;
			bool hitGeom = scene->rayIntersect(sRay, itsM);

			// Check if a media_container is hit
			bool hitPOI = false;
			// Use a placeholder for the new state (current containers)
			std::vector<MedT> candCont;

			for (MediaContainer * m : allMedia) {
				Intersection itsMedium;
				if (m->setHitInformation(sRay, itsMedium)) {
					// Hit a container
					candCont.push_back(MedT(m, itsMedium.t));
				}
			}

			// Check those MediaContainer intersections, find the closest one (create placeholder with impossible intersection 't')
			MedT nIts = MedT(NULL, sRay.maxt + 1.0f);
			for (MedT c : candCont) {
				// Only stop for new containers or the end of the currenty sampled one
				bool isNew = true;
				for (MediaContainer * s : currMedia) {
					if (c.m->getName().compare(s->getName())) {
						// Already in this container, check if currently sampling it 
						if (c.m->getName().compare(sampledMedia->getName()) && (c.t <= nIts.t))
							nIts = c;
						isNew = false;
					}
				}
				if (isNew && (c.t < nIts.t)) {
					// New container with a intersection nearer than the previous nearest.
					nIts = c;
				}
			}

			// Check if there is point of interest from a media container
			hitPOI = nIts.m != NULL;

			if (!(hitGeom || hitPOI || inMedia)) {
				// No intersection, not within any medium and therefore no incoming light
				break;
			}
			else if (hitGeom && (itsM.t <= nIts.t)) {
				// Hit Geometry before medium POI or Scattering event -> handle geometry
				// Default case: compute light reflected by surface but adjust by Transmittance

				// If in Media, adjust transmittance, add emission (before adding light coming from the surface)
				if (inMedia) {
					// Compute current position
					Vector3f diff = itsM.t * sRay.d;
					float dx = sqrt(diff.dot(diff));

					// Anyway, adjust transmittance
					float integ = calcLinearInt(dx, sampledMedia->getExtinction(sRay.o), sampledMedia->getExtinction(itsM.p));
					float addTr = exp(-integ);
					tr = tr * addTr;
					if (tr != tr)
						printf("tr is nan!\n");

					// And add emission
					exRad += tr * calcLinearIntCol(dx, sampledMedia->getEmission(sRay.o), sampledMedia->getEmission(itsM.p));
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
				else if (it > 0) {
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

				// Add new contributions. But also multiplay with transmittance (if the path has lead through media)
				exRad += wEm * t.cwiseProduct(emsEmS.cwiseProduct(emsBSDFRes)) * tr;

				// Add the mats-contribution previously computed (wMat is 0 if not an emitter)
				exRad += wMat * t.cwiseProduct(matsEmEval) * tr;

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

				// HPM: Sample Distance when within medium
				if (inMedia) {
					sRay.d = sampleDistance(currMedia, itsM.p, sampler->next2D(), sampledMedia);
				}

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
			else if (hitPOI) {
				// Hit no Geometry or medium POI is closer
				Vector3f diff = nIts.t * sRay.d;
				float dx = sqrt(diff.dot(diff));
				Point3f pos = sRay.o + diff;

				// Check if entering or leaving a container - adjust currMedia
				bool bLeaving = false;
				std::vector<MediaContainer *> temp;
				for (MediaContainer * m : currMedia) {
					if (nIts.m->getName().compare(m->getName())) {
						bLeaving = true;
					}
					else {
						// Not leaving this container *m, keep it
						temp.push_back(m);
					}
				}
				if (!bLeaving) {
					// Not leaving - entering nIts.m
					temp.push_back(nIts.m);
				}

				// If in media before POI...
				if (inMedia) {
					// Adjust transmittance from last path segment
					float integ = calcLinearInt(dx, sampledMedia->getExtinction(sRay.o), sampledMedia->getExtinction(pos));
					float addTr = exp(-integ);
					tr = tr * addTr;
					if (tr != tr)
						printf("tr is nan!\n");

					// Add emission from medium during last path segment
					exRad += tr * calcLinearIntCol(dx, sampledMedia->getEmission(sRay.o), sampledMedia->getEmission(pos));
				}

				// Sample distance for next step (if still or now in Medium)
				currMedia = temp;
				inMedia = (temp.size() > 0);

				// Direction is still the same
				sRay = Ray3f(pos, sRay.d);

				if (inMedia) {
					// Sample distance according to a medium container
					sRay.maxt = sampleDistance(currMedia, pos, sampler->next2D(), sampledMedia);
				}

			}
			else {
				// No geometry intersection/no media POI - but apparently still in Media (else break; in first if-case)
				// Woodcock-Tracking
				// -> maybe scattering event, sample new direction from phase function
				// or simply continue with the tracking

				// Compute current position
				Vector3f diff = sRay.maxt * sRay.d;
				float dx = sqrt(diff.dot(diff));
				Point3f pos = sRay.o + diff;

				// Anyway, adjust transmittance
				float integ = calcLinearInt(dx, sampledMedia->getExtinction(sRay.o), sampledMedia->getExtinction(pos));
				float addTr = exp(-integ);
				tr = tr * addTr;
				if (tr != tr)
					printf("tr is nan!\n");

				// And add emission
				exRad += tr * calcLinearIntCol(dx, sampledMedia->getEmission(sRay.o), sampledMedia->getEmission(pos));

				// and directly sample some emitter (MIS) - TODO


				// Check if scattering event
				float p = sampledMedia->getExtinction(pos) / sampledMedia->getMajExtinction();

				if (sampler->next1D() < p) {
					// Scattering event! Sample a new direction from the phase function and adjust sRay

					Vector3f dir = sampledMedia->samplePhaseFunction(pos, sampler->next2D());
					if (dir.x() != dir.x())
						printf("Sampled dir is nan!\n");


					// Adjust the transmittance by the scattering coefficient (f_p and pdf of phasefunction cancel out)
					// sigma_s(x) * L(x,w) remains
					tr = tr * sampledMedia->getScattering(pos);
					if (tr != tr)
						printf("tr is nan!\n");

					// Sample a distance
					float dis = sampleDistance(currMedia, pos, sampler->next2D(), sampledMedia);

					sRay = Ray3f(pos, dir);
					sRay.maxt = dis;

				}
				else {
					// No scattering event - just sample a new distance in the same direction
					sRay = Ray3f(pos, sRay.d);
					sRay.maxt = sampleDistance(currMedia, pos, sampler->next2D(), sampledMedia);
				}

			}


		}
		
		if (!exRad.isValid())
			printf("Not valid exRad!\n");

		return exRad;

	}

	std::string toString() const {
		return "Path_Mis_HPM[]";
	}

	// Returns the Woodcock-sampled distance and refers to the sampled MediaContainer in chosenMedia.
	virtual float sampleDistance(const std::vector<MediaContainer *> & media, const Point3f pos, const Point2f sample, MediaContainer * & chosenMedia) const {
		
		// First, choose which media to influence distance (if more than one)
		float t;
		if (media.size() > 1) {
			// Random media drawn weighted by their local density
			float tSum = 0.0f;
			std::vector<float> ts;
			for (int i = 0; i < media.size(); i++) {
				float w = media[i]->getExtinction(pos);
				tSum = tSum + w;
				ts.push_back(w);
			}
			// Get a random number in [0, tSum)
			float rnd = sample.x() * tSum;
			// Subtract each container's weight until rnd is <= 0 -> winner is chosen
			int i = 0;
			while (rnd > ts[i] && i < ts.size()) {
				rnd = rnd - ts[i];
				i++;	
			}

			chosenMedia = media[i];
			t = ts[i];
		}
		else {
			chosenMedia = media[0];
			t = chosenMedia->getExtinction(pos);
		}

		float maj = chosenMedia->getMajExtinction();

		// Now, sample the distance
		float dis = -log(1.0f - sample.y()) / maj;

		return dis;
	}


	virtual Color3f estDirectContr() const {
		throw NoriException("TODO");
	}

	// Computes the integral of a function defined by two values y1,y2 separated by dx
	virtual float calcLinearInt(float dx, float y1, float y2) const {
		float dy = std::abs(y2 - y1);
		float sy = std::min(y1, y2);
		float res = (dx * dy) / 2.0f + dx * sy;
		if (res != res)
			printf("Integ nan!");
		return res;
	}

	virtual Color3f calcLinearIntCol(float dx, Color3f y1, Color3f y2) const {
		float dr = std::abs(y2.x() - y1.x());
		float sr = std::min(y1.x(), y2.x());
		float dg = std::abs(y2.y() - y1.y());
		float sg = std::min(y1.y(), y2.y());
		float db = std::abs(y2.z() - y1.z());
		float sb = std::min(y1.z(), y2.z());
		return Color3f((dx * dr) / 2.0f + dx * sr, (dx * dg) / 2.0f + dx * sg, (dx * db) / 2.0f + dx * sb);
	}

protected:
	float rayLength;
	bool m_bRenderEmitter;
};

NORI_REGISTER_CLASS(PathMisHPMIntegrator, "path_mis_hpm");
NORI_NAMESPACE_END