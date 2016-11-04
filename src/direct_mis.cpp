#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/common.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class DirectMisIntegrator : public Integrator {
public:
	DirectMisIntegrator(const PropertyList &props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {

		// Define exitant radiance (black default)
		Color3f exRad = Color3f(0.0f);

		Point2f sample2D = sampler->next2D();
		float sample1D = sampler->next1D();

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


		// 1 Sample according to the pdf of the brdf of this shape's surface
		// Build BSDFQuery
		BSDFQueryRecord bsdfRecMat = BSDFQueryRecord(itsM.toLocal(-ray.d));
		bsdfRecMat.uv = itsM.uv;
		// Sample to get a direction
		Color3f bsdfResMat = objBSDF->sample(bsdfRecMat, sample2D);
		// Use the sample direction in world-space for casting a ray
		Vector3f woWC = itsM.toWorld(bsdfRecMat.wo);
		// Get the pdf of this sample for computing the weight
		float pdfBSDF = objBSDF->pdf(bsdfRecMat);
		Ray3f sampleRay = Ray3f(p, woWC);

		// Check for an intersection of the the ray in the sampled direction (from BSDF)
		Intersection itsBSDF;

		if (scene->rayIntersect(sampleRay, itsBSDF)) {
			// Intersection of the sampling ray - check if it is an emitter

			if (itsBSDF.mesh->isEmitter()) {
				// Compute contribution of this emitter
				EmitterQueryRecord lRecMat = EmitterQueryRecord(p, itsBSDF.p, itsBSDF.shFrame.n);
				const Emitter* em = itsBSDF.mesh->getEmitter();
				Color3f incRadMat = em->eval(lRecMat);
				float pdfEmitter = em->pdf(lRecMat);

				float wBSDF = 0.0f;
				if (!(pdfBSDF == 0.0f && pdfEmitter == 0.0f)) {
					wBSDF = pdfBSDF / (pdfBSDF + pdfEmitter);
				}

				// Compute addition of the incoming radiance of this emitter (already divided by pdf in bsdf->sample())
				Color3f addRad = (incRadMat.cwiseProduct(bsdfResMat));
				// Adjust by weight
				exRad += wBSDF * addRad;
					
				if (addRad.x() < 0 || addRad.y() < 0 || addRad.z() < 0) {
					printf("MIS - Negative brdf sample radiance at %.2f, %.2f, %.2f\n", p.x(), p.y(), p.z());
				}

			}
			// else: Not an emitter. Since no indirect illumination is computed, no light received.
		}
		// else: No collision in sample direction. No light received from this direction.
		

		// 2 Get a random emitter for sampling
		// Get a random sample
		const Emitter* em = scene->getRandomEmitter(sample1D);
		float pdfEmitterUnif = (1.f / (scene->getLights()).size());

		// Prepare the EmitterQueryRecord
		EmitterQueryRecord lRec;
		lRec = EmitterQueryRecord(p);

		// Get a random sample
		Color3f incRad = em->sample(lRec, sample2D);

		float pdfEmitterEm = pdfEmitterUnif * em->pdf(lRec);

		// Also sample the bsdf for the direction of the emitter-sample
		// Build BSDFQuery
		BSDFQueryRecord bsdfRecEm = BSDFQueryRecord(itsM.toLocal(-ray.d), itsM.toLocal(lRec.wi), ESolidAngle);
		bsdfRecEm.uv = itsM.uv;
		Color3f bsdfResEm = objBSDF->eval(bsdfRecEm);
		float pdfBSDFEmitter = objBSDF->pdf(bsdfRecEm);

		float wEmitter = 0.0f;
		if (!(pdfBSDFEmitter == 0.0f && pdfEmitterEm == 0.0f)) {
			wEmitter = pdfEmitterEm / (pdfBSDFEmitter + pdfEmitterEm);
		}
		// 2 Sample the random emitter and add the incoming radiance

		// Check for an intersection of the shadow ray on the way to the light
		Intersection itsEm;
		bool bColl = scene->rayIntersect(lRec.shadowRay, itsEm);
		// Ensure that the collision is not caused by the emitter itself (only the area between)
		bool bEmitterIntersect = bColl && (lRec.shadowRay.maxt - itsEm.t <= 0.0001f);

		if (!bColl || bEmitterIntersect) {
			// No intersection, point fully visible from emitter

			// Angle between shading normal and direction to emitter
			float cosThetaInEm = n.dot(lRec.wi) / (n.norm() * lRec.wi.norm());
			if (cosThetaInEm >= 0) {
				// Compute addition of the incoming radiance of this emitter
				Color3f addRad = (incRad * bsdfResEm * cosThetaInEm);
				// Adjust result by probability of choosing this specific emitter (uniform here), adjusted by weight
				exRad += wEmitter * addRad;
				
				if (addRad.x() < 0 || addRad.y() < 0 || addRad.z() < 0) {
					printf("MIS - Negative emitter radiance at %.2f, %.2f, %.2f\n", p.x(), p.y(), p.z());
				}
			}
			// Else: Emitter-Sample not directly visible, add nothing
		}
		// Else: Collision with an object, in shadow from this emitter. Add nothing
		// to exitant radiance


		// 3. Add emitted radiance from this mesh (if emitter)
		if (itsM.mesh->isEmitter()) {
			EmitterQueryRecord lRecSelf = EmitterQueryRecord(p);
			// Add only value evaluated at this emitter-object (and not divided by pdf as in Emitter::sample())
			exRad += itsM.mesh->getEmitter()->eval(lRecSelf);
		}

		if (!exRad.isValid()) {
			printf("Error! MIS - Invalid Radiance at %.2f, %.2f, %.2f\n", p.x(), p.y(), p.z());
		}

		return exRad;

	}

	std::string toString() const {
		return "Direct_Mis[]";
	}
protected:
	float rayLength;
};

NORI_REGISTER_CLASS(DirectMisIntegrator, "direct_mis");
NORI_NAMESPACE_END