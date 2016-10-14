#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AverageVisibility : public Integrator {
public:
	AverageVisibility(const PropertyList &props) {
		rayLength = props.getFloat("length");
	}

	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
		// Colors used below
		Color3f colBright = Color3f(1.0f);
		Color3f colDark = Color3f(0.0f);

		/* Find the surface that is visible in the requested direction */
		Intersection its;
		if (!scene->rayIntersect(ray, its))	{
			// No intersection, no 'shadows'
			return colBright;
		}
		
		// There is an intersection, get the point and the corr. normal
		Point3f p = its.p;
		Normal3f n = its.shFrame.n;
		// Generate a random direction to check the visibility
		Vector3f visRayDir = Warp::sampleUniformHemisphere(sampler, n);
		// Adjust length of this ray
		Ray3f rayHem = Ray3f(p, visRayDir, 0.0001f, rayLength);

		// check for an intersection
		if (!scene->rayIntersect(rayHem, its)) {
			// No intersection, point fully visible from hemisphere
			return colBright;
		}
		else {
			// Collision with an object, in shadow (dark Pixel)
			return colDark;
		}
	}

	std::string toString() const {
		return "AverageVisibility[]";
	}
protected:
	float rayLength;
};

NORI_REGISTER_CLASS(AverageVisibility, "av");
NORI_NAMESPACE_END