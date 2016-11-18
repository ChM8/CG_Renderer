/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>
#include <nori/photon.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class PhotonMapper : public Integrator {
public:
    /// Photon map data structure
    typedef PointKDTree<Photon> PhotonMap;

    PhotonMapper(const PropertyList &props) {
        /* Lookup parameters */
        m_photonCount  = props.getInteger("photonCount", 1000000);//TODO CORRECT AGAIN!
        m_photonRadius = props.getFloat("photonRadius", 0.0f /* Default: automatic */);
    }

    virtual void preprocess(const Scene *scene) override {
        cout << "Gathering " << m_photonCount << " photons .. ";
        cout.flush();

        /* Create a sample generator for the preprocess step */
        Sampler *sampler = static_cast<Sampler *>(
            NoriObjectFactory::createInstance("independent", PropertyList()));

        /* Allocate memory for the photon map */
        m_photonMap = std::unique_ptr<PhotonMap>(new PhotonMap());
        m_photonMap->reserve(m_photonCount);

		/* Estimate a default photon radius */
		if (m_photonRadius == 0)
			m_photonRadius = scene->getBoundingBox().getExtents().norm() / 500.0f;

		/* How to add a photon?
		 * m_photonMap->push_back(Photon(
		 *	Point3f(0, 0, 0),  // Position
		 *	Vector3f(0, 0, 1), // Direction
		 *	Color3f(1, 2, 3)   // Power
		 * ));
		 */

		// put your code to trace photons here
		int diff = m_photonCount;

		while (diff > 0) {
			// Not yet all photons sampled - get some
			std::vector<Photon> sPhs = std::vector<Photon>();
			samplePhoton(scene, &sPhs, sampler, diff);

			// Push the sampled photons into the map
			for (const Photon ph : sPhs) {
				//Photon p = Photon(ph);
				m_photonMap->push_back(ph);
				//printf("Pushing photons with power: %.2f,%.2f,%.2f - %.2f,%.2f,%.2f\n", p.getPower().x(), ph.getPower().y(), ph.getPower().z(), p.getPower().x(), p.getPower().y(), p.getPower().z());
			}

			diff--;

		}

		/* Build the photon map */
        m_photonMap->build();
    }

    virtual Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const override {
    	
		/* How to find photons?
		 * std::vector<uint32_t> results;
		 * m_photonMap->search(Point3f(0, 0, 0), // lookup position
		 *                     m_photonRadius,   // search radius
		 *                     results);
		 *
		 * for (uint32_t i : results) {
		 *    const Photon &photon = (*m_photonMap)[i];
		 *    cout << "Found photon!" << endl;
		 *    cout << " Position  : " << photon.getPosition().toString() << endl;
		 *    cout << " Power     : " << photon.getPower().toString() << endl;
		 *    cout << " Direction : " << photon.getDirection().toString() << endl;
		 * }
		 */

		// Default: no light
		Color3f exRad = Color3f(0.0f);

		// Throughput
		Color3f t = Color3f(1.0f);
		int it = 0, minIt = 3;

		Ray3f sRay = _ray;

		// Trace path until break (diffuse/russian roulette)
		while (true) {

			Intersection itsM;

			if (!scene->rayIntersect(sRay, itsM)) {
				// No intersection with geometry - no light received from this direction
				break;
			}

			// Get information about intersection
			Point3f p = itsM.p;
			Normal3f n = itsM.geoFrame.n;
			const BSDF * objBSDF = itsM.mesh->getBSDF();

			// Check if this mesh emits light
			if (itsM.mesh->isEmitter()) {
				// If so, add emitted radiance
				const Emitter* em = itsM.mesh->getEmitter();

				// Create an EmitterQueryRecord
				EmitterQueryRecord lRec = EmitterQueryRecord(sRay.o, p, n);
				// Get the incident radiance from this emitter (exitant times throughput)
				exRad += t.cwiseProduct(em->eval(lRec));
			}

			// Check if the surface is diffuse
			if (objBSDF->isDiffuse()) {
				// Query the PhotonMap and stop path-tracing
				std::vector<uint32_t> results;
				m_photonMap->search(itsM.p,	m_photonRadius, results);
				
				// Density Estimation of Photons
				// Iterate over all the photons found in the map
				Color3f phContr = Color3f(0.0f);
				for (uint32_t i : results) {
					const Photon &ph = (*m_photonMap)[i];
					/*Point3f posPh = ph.getPosition();
					Vector3f dirPh = ph.getDirection();
					Color3f powPh = ph.getPower();
					printf("Photon %d: Pos:[%.2f,%.2f,%.2f], Dir: [%.2f,%.2f,%.2f], Pow: [%.2f,%.2f,%.2f]\n", i, posPh.x(), posPh.y(), posPh.z(), dirPh.x(), dirPh.y(), dirPh.z(), powPh.x(), powPh.y(), powPh.z());*/

					// Query the BSDF
					BSDFQueryRecord bRecPh = BSDFQueryRecord(itsM.toLocal(-sRay.d), itsM.toLocal(ph.getDirection()), ESolidAngle);
					Color3f bResPh = objBSDF->eval(bRecPh);
					
					// Adjust the power by the number of photons
					phContr += bResPh.cwiseProduct(ph.getPower() / m_photonCount);
				}

				exRad += t.cwiseProduct((phContr) / (M_PI * m_photonRadius * m_photonRadius));
				
				break;
			}

			// Else, Russian Roulette
			float succProb = (it >= minIt) ? std::min(t.maxCoeff(), 0.999f) : 1.0f;
			if (sampler->next1D() >= succProb) {
				// Failed in Russian Roulette - break path-tracing
				break;
			}

			// Sample the bsdf for the next direction
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
			float cosThetaIn = (n.norm() * woWC.norm() > 0.0f) ? n.dot(woWC) / (n.norm() * woWC.norm()) : 0.0f;

			t = t.cwiseProduct(bsdfRes);// *cosThetaIn;


		}

		//printf("Exrad: %.2f\n", exRad);
		return exRad;
    }

    virtual std::string toString() const override {
        return tfm::format(
            "PhotonMapper[\n"
            "  photonCount = %i,\n"
            "  photonRadius = %f\n"
            "]",
            m_photonCount,
            m_photonRadius
        );
    }
	 
	virtual void samplePhoton(const Scene * scene, std::vector<Photon> * resPh, Sampler * sampler, int maxSamples) {

		// Choose a random emitter (TODO: atm assuming only AreaEmitters in the scene)
		const Emitter * emR = scene->getRandomEmitter(sampler->next1D());
		float emProb = 1.0f / scene->getLights().size();

		// Sample a photon from the random emitter
		Ray3f sRay;
		Color3f W = emR->samplePhoton(sRay, sampler->next2D(), sampler->next2D());

		// Variabls for Russian Roulette
		int it = 0, minIt = 3;

		// Path-trace the photon (and add a photon to the result list at every diffuse surface - up to maxSamples photons)
		while (resPh->size() < maxSamples) {
			// Trace the current ray
			Intersection itsM;

			if (!scene->rayIntersect(sRay, itsM)) {
				// No intersection and therefore no place to store any photons
				return;
			}

			// Intersection in the scene with current ray
			const BSDF * objBsdf = itsM.mesh->getBSDF();

			if (objBsdf->isDiffuse()) {
				// Surface is diffuse, create and store a photon at this position
				resPh->push_back(Photon(itsM.p, -sRay.d, W));
			}

			// Play Russian Roulette (if trace should be continued)
			float succProb = (it >= minIt) ? std::min(W.maxCoeff(), 0.999f) : 1.0f;
			if (sampler->next1D() >= succProb) {
				// Failed in Russian Roulette - break path-tracing
				return;
			}
			W /= succProb;

			// Sample the bsdf at the current position and create a new ray to follow
			BSDFQueryRecord bRec = BSDFQueryRecord(itsM.toLocal(-sRay.d));
			Color3f bsdfRes = objBsdf->sample(bRec, sampler->next2D());
			Vector3f woWC = itsM.toWorld(bRec.wo);
			sRay = Ray3f(itsM.p, woWC);

			// Adjust the power according to the bsdf/theta/pdf (bsdf/pdf should be calculated in sample())
			Vector3f n = itsM.geoFrame.n;
			float cosThetaIn = 0.0f;
			if (n.norm() * woWC.norm() > 0.0f) {
				float cosThetaIn = n.dot(woWC) / (n.norm() * woWC.norm());
			}
			else {
				break;
			}

			W = W.cwiseProduct(bsdfRes);

			it++;
		}

		return;
	}


private:
    int m_photonCount;
    float m_photonRadius;
    std::unique_ptr<PhotonMap> m_photonMap;
};

NORI_REGISTER_CLASS(PhotonMapper, "photonmapper");
NORI_NAMESPACE_END
