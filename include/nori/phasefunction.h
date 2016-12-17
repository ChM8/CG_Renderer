#if !defined(__NORI_PHASEFUNCTION_H)
#define __NORI_PHASEFUNCTION_H

NORI_NAMESPACE_BEGIN

// Abstract interface for the implementations of phase functions (e.g. HenyeyGreenstein)
class PhaseFunction {
public:
	
	virtual Vector3f sample(Point2f sample) = 0;

	virtual float pdf(float cosTh) = 0;

protected:

};

NORI_NAMESPACE_END

#endif /* __NORI_PHASEFUNCTION_H */