#if !defined(__NORI_HENYEY_GREENSTEIN_H)
#define __NORI_HENYEY_GREENSTEIN_H

NORI_NAMESPACE_BEGIN

// Abstract interface for the implementations of phase functions (e.g. HenyeyGreenstein)
class HenyeyGreenstein : public PhaseFunction {
public:

	HenyeyGreenstein(float g);

	virtual Vector3f sample(Point2f sample);

	virtual float pdf(float cosTh);

protected:
	float m_g;
};

NORI_NAMESPACE_END

#endif /* __NORI_HENYEY_GREENSTEIN_H */