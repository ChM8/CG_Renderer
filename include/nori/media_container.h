#if !defined(__NORI_MEDIA_CONTAINER_H)
#define __NORI_MEDIA_CONTAINER_H

#include <Eigen/Geometry>
#include <nori/shape.h>
#include <nori/phasefunction.h>
#include <nori/henyey_greenstein.h>

NORI_NAMESPACE_BEGIN

// Extend shape for simple intersection
class MediaContainer : public NoriObject {
public:
	MediaContainer();
	MediaContainer(const PropertyList &propList);
	MediaContainer(const std::string name, Vector3f trans, Vector3f rotAxis, float rotAngle, Vector3f scale, /*PhaseFunction*/HenyeyGreenstein &p, float absC, float sctC, Color3f em);
	virtual Vector3f samplePhaseFunction(Point3f pos, Point2f sample) const;
	virtual float getAbsorbtion(Point3f pos) const;
	virtual float getScattering(Point3f pos) const;
	virtual Color3f getEmission(Point3f pos) const;
	virtual float getExtinction(Point3f pos) const;
	virtual float getMajExtinction() const;
	virtual bool withinContainer(Point3f pos) const;

	virtual std::string getName() const;
	
	// Shape overrides
	EClassType getClassType() const override { return EMediaContainer; }
	virtual std::string toString() const override;
	virtual BoundingBox3f getBoundingBox(uint32_t index) const ;
	virtual Point3f getCentroid(uint32_t index) const ;
	virtual bool inRng(float x, float min, float max) const;
	virtual bool rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const ;
	virtual bool MediaContainer::setHitInformation(const Ray3f & ray, Intersection & its) const;
	virtual void sampleSurface(ShapeQueryRecord & sRec, const Point2f & sample) const;
	virtual float pdfSurface(const ShapeQueryRecord & sRec) const ;

	// Little container for some values needed during the calculations
	struct homMedia
	{
		/*PhaseFunction*/HenyeyGreenstein * p;
		float a;
		float s;
		Color3f e;

		homMedia() {}
	};

protected:
	std::string m_name;
	Vector3f m_translation;
	Vector3f m_rotationAxis;
	float m_rotationAngle;
	Vector3f m_scale;
	// Transformation with scaling
	Eigen::Transform<float, 3, Eigen::Affine> m_toLocalSc;
	Eigen::Transform<float, 3, Eigen::Affine> m_toWorldSc;
	// Transformation without scaling
	Eigen::Transform<float, 3, Eigen::Affine> m_toLocal;
	Eigen::Transform<float, 3, Eigen::Affine> m_toWorld;

	bool m_isVDB;
	homMedia m_homogeneous;
	float m_majExtinction;
	// Grid m_grid;

};


NORI_NAMESPACE_END

#endif /* __NORI_MEDIA_CONTAINER_H */