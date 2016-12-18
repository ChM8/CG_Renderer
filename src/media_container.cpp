#include <Eigen/Geometry>
#include <nori/object.h>
#include <filesystem/resolver.h>
#include <nori/media_container.h>


NORI_NAMESPACE_BEGIN


// Constructor
MediaContainer::MediaContainer(const PropertyList &propList) {
		
	if (!propList.has("filename")) {

		// Check if homogeneous system is created (phase function needs to be defined - let's hope the others are too. Could be solved better)
		if (!propList.has("henyey_greenstein")) {
			throw NoriException("Not enough information for the MediaContainer!");
		}

		// Get the values
		m_name = propList.getString("name");
		m_translation = propList.getVector3("position", Vector3f(0.0f));
		m_rotationAxis = propList.getVector3("rotation_axis", Vector3f(0.0f));
		m_rotationAngle = propList.getFloat("rotation_angle", 0.0f);
		m_scale = propList.getVector3("scale", Vector3f(0.0f));
			
		m_homogeneous.p = &HenyeyGreenstein(propList.getFloat("henyey_greenstein", 0.0f));
		m_homogeneous.a = propList.getFloat("absorption", 0.0f);
		m_homogeneous.s = propList.getFloat("scattering", 0.0f);

		m_majExtinction = m_homogeneous.a + m_homogeneous.s;

		m_toLocalSc = Eigen::Translation3f(m_translation) * Eigen::AngleAxisf(m_rotationAngle, m_rotationAxis) * Eigen::Scaling(m_scale);
		m_toWorldSc = m_toLocalSc.inverse();

		m_toLocal = Eigen::Translation3f(m_translation) * Eigen::AngleAxisf(m_rotationAngle, m_rotationAxis);
		m_toWorld = m_toLocal.inverse();

		return;
	}

	filesystem::path filename =
		getFileResolver()->resolve(propList.getString("filename"));

	// TODO: Load the .vdb
	m_isVDB = true;

}

MediaContainer::MediaContainer()
{
	MediaContainer("media_container", Vector3f(0.0f), Vector3f(0.0f), 0.0f, Vector3f(0.0f), HenyeyGreenstein(0.0f), 0.0f, 0.0f);
}

// Create a container with homogeneous media (default: cube 1x1x1 - adjust by scale)
MediaContainer::MediaContainer(const std::string name, Vector3f trans, Vector3f rotAxis, float rotAngle, Vector3f scale, PhaseFunction &p, float absC, float sctC) {
	m_name = name;
	m_translation = trans;
	m_rotationAxis = rotAxis;
	m_rotationAngle = rotAngle;
	m_scale = scale;

	m_isVDB = false;
	m_homogeneous.p = &p;
	m_homogeneous.a = absC;
	m_homogeneous.s = sctC;

	m_majExtinction = absC + sctC;

	m_toLocalSc = Eigen::Translation3f(trans) * Eigen::AngleAxisf(rotAngle, rotAxis) * Eigen::Scaling(scale);
	m_toWorldSc = m_toLocalSc.inverse();

	m_toLocal = Eigen::Translation3f(trans) * Eigen::AngleAxisf(rotAngle, rotAxis);
	m_toWorld = m_toLocal.inverse();

}

Vector3f MediaContainer::samplePhaseFunction(Point3f pos, Point2f sample) const {
		
	if (!m_isVDB) {
		// Homogeneous system
		return m_homogeneous.p->sample(sample);
	}
	else {
		Point3f posL = m_toLocalSc * pos;
			
		throw NoriException("VDB to implement!");
	}

}

float MediaContainer::getAbsorbtion(Point3f pos) const {
	if (!m_isVDB) {
		// Homogeneous system
		return m_homogeneous.a;
	}
	else {
		Point3f posL = m_toLocalSc * pos;

		throw NoriException("VDB to implement!");
	}
}

float MediaContainer::getScattering(Point3f pos) const {
	if (!m_isVDB) {
		// Homogeneous system
		return m_homogeneous.s;
	}
	else {
		Point3f posL = m_toLocalSc * pos;

		throw NoriException("VDB to implement!");
	}
}

float MediaContainer::getExtinction(Point3f pos) const {
	if (!m_isVDB) {
		// Homogeneous system
		return m_homogeneous.s + m_homogeneous.a;
	}
	else {
		Point3f posL = m_toLocalSc * pos;

		throw NoriException("VDB to implement!");
	}
}

float MediaContainer::getMajExtinction() const {
	return m_majExtinction;
}

bool MediaContainer::withinContainer(Point3f pos) const {

	// Transform to local space of container for simpler test
	//Point3f posL = Eigen::Translation3f(m_translation) * Eigen::AngleAxisf(m_rotationAngle, m_rotationAxis) * pos;
	Point3f posL = m_toLocalSc * pos;

	if (!m_isVDB) {
		// Homogeneous system
		/*float x = m_scale.x() / 2.0f;
		float y = m_scale.y() / 2.0f;
		float z = m_scale.z() / 2.0f;
		if (pos.x() < x && pos.x() > -x
			&& pos.y() < y && pos.y() > -y
			&& pos.z() < z && pos.z() > -z) {
			// Position is within the volume
			return true;
		}*/

		// Default is a 1x1x1 cube (point was scaled from world->local)
		if (pos.x() < 0.5f && pos.x() > -0.5f
			&& pos.y() < 0.5f && pos.y() > -0.5f
			&& pos.z() < 0.5f && pos.z() > -0.5f) {
			// Position is within the volume
			return true;
		}

		// Not within the volume
		return false;
	}
	else {
		// VDB to implement
		throw NoriException("VDB to implement!");
	}
}

std::string MediaContainer::getName() const
{
	return m_name;
}

std::string MediaContainer::toString() const {
	return tfm::format(
		"Mesh[\n"
		"  name = \"%s\",\n]",
		m_name
		);

}

BoundingBox3f MediaContainer::getBoundingBox(uint32_t index) const
{
	if (!m_isVDB) {
		// Default is a 1x1x1 cube (scaling by transformation)
		std::vector<Point3f> ps;
		Point3f min, max = Point3f(0.0f);
		ps.push_back(m_toWorldSc * Point3f(-0.5f, -0.5f, -0.5f));
		ps.push_back(m_toWorldSc * Point3f(0.5f, -0.5f, -0.5f));
		ps.push_back(m_toWorldSc * Point3f(-0.5f, 0.5f, -0.5f));
		ps.push_back(m_toWorldSc * Point3f(-0.5f, -0.5f, 0.5f));
		ps.push_back(m_toWorldSc * Point3f(0.5f, 0.5f, -0.5f));
		ps.push_back(m_toWorldSc * Point3f(0.5f, -0.5f, 0.5f));
		ps.push_back(m_toWorldSc * Point3f(-0.5f, 0.5f, 0.5f));
		ps.push_back(m_toWorldSc * Point3f(0.5f, 0.5f, 0.5f));

		for (Point3f p : ps) {
			if (p.x() < min.x())
				min.x() = p.x();
			if (p.y() < min.y())
				min.y() = p.y();
			if (p.z() < min.z())
				min.z() = p.z();
			if (p.x() > max.x())
				min.x() = p.x();
			if (p.y() > max.y())
				min.y() = p.y();
			if (p.z() > max.z())
				min.z() = p.z();
		}

		return BoundingBox3f(min, max);
	}
	else {
		throw NoriException("To implement!");
	}
}


Point3f MediaContainer::getCentroid(uint32_t index) const
{
	if (!m_isVDB) {
		
		return m_toWorldSc * Point3f(0.0f);
		
	}
	else {
		throw NoriException("To implement!");
	}
}

bool MediaContainer::inRng(float x, float min, float max) const {
	return (x > min && x < max);
}

bool MediaContainer::rayIntersect(uint32_t index, const Ray3f & ray, float & u, float & v, float & t) const
{
	if (!m_isVDB) {

		Ray3f lRay = Ray3f(ray);
		lRay.o = m_toLocal * lRay.o;
		lRay.d = m_toLocal * lRay.d;
		
		// Default is a 1x1x1 cube - scale indicates axis aligned bounding box in local system (with respect to position/rotation)
		Point3f min = -m_scale / 2.0f;
		Point3f max = m_scale / 2.0f;
		// First check y/z planes
		Vector2f tc = (Vector2f(min.x(), max.x()) - Vector2f(lRay.o.x())) / lRay.d.x();
		// Get the smaller result (if one)
		tc = (tc.y() < tc.x()) ? Vector2f(tc.y(), tc.x()) : tc;
		if (inRng(tc.x(), lRay.mint, lRay.maxt)) {
			Point3f p = lRay.o + tc.x() * lRay.d;
			if (inRng(p.y(), min.y(), max.y()) && inRng(p.z(), min.z(), max.z())) {
				// Valid intersection
				t = tc.x();
				return true;
			}
		}
		if (inRng(tc.y(), lRay.mint, lRay.maxt)) {
			Point3f p = lRay.o + tc.y() * lRay.d;
			if (inRng(p.y(), min.y(), max.y()) && inRng(p.z(), min.z(), max.z())) {
				// Valid intersection
				t = tc.y();
				return true;
			}
		}

		// Check x/z planes
		tc = (Vector2f(min.y(), max.y()) - Vector2f(lRay.o.y())) / lRay.d.y();
		// Get the smaller result (if one)
		tc = (tc.y() < tc.x()) ? Vector2f(tc.y(), tc.x()) : tc;
		if (inRng(tc.x(), lRay.mint, lRay.maxt)) {
			Point3f p = lRay.o + tc.x() * lRay.d;
			if (inRng(p.x(), min.x(), max.x()) && inRng(p.z(), min.z(), max.z())) {
				// Valid intersection
				t = tc.x();
				return true;
			}
		}
		if (inRng(tc.y(), lRay.mint, lRay.maxt)) {
			Point3f p = lRay.o + tc.y() * lRay.d;
			if (inRng(p.x(), min.x(), max.x()) && inRng(p.z(), min.z(), max.z())) {
				// Valid intersection
				t = tc.y();
				return true;
			}
		}

		// Lastly, check x/y planes
		tc = (Vector2f(min.z(), max.z()) - Vector2f(lRay.o.z())) / lRay.d.z();
		// Get the smaller result (if one)
		tc = (tc.y() < tc.x()) ? Vector2f(tc.y(), tc.x()) : tc;
		if (inRng(tc.x(), lRay.mint, lRay.maxt)) {
			Point3f p = lRay.o + tc.x() * lRay.d;
			if (inRng(p.x(), min.x(), max.x()) && inRng(p.y(), min.y(), max.y())) {
				// Valid intersection
				t = tc.x();
				return true;
			}
		}
		if (inRng(tc.y(), lRay.mint, lRay.maxt)) {
			Point3f p = lRay.o + tc.y() * lRay.d;
			if (inRng(p.x(), min.x(), max.x()) && inRng(p.y(), min.y(), max.y())) {
				// Valid intersection
				t = tc.y();
				return true;
			}
		}

	}
	else {
		throw NoriException("To implement!");
	}
}

// Returns true if there is a valid hit, sets t. Else returns false.
bool MediaContainer::setHitInformation(const Ray3f & ray, Intersection & its) const
{

	if (!m_isVDB) {

		Ray3f lRay = Ray3f(ray);
		lRay.o = m_toLocal * lRay.o;
		lRay.d = m_toLocal * lRay.d;

		bool hit = false;
		its.t = ray.maxt;

		// Default is a 1x1x1 cube - scale indicates axis aligned bounding box in local system (with respect to position/rotation)
		Point3f min = -m_scale / 2.0f;
		Point3f max = m_scale / 2.0f;
		// First check y/z planes
		Vector2f tc = (Vector2f(min.x(), max.x()) - Vector2f(lRay.o.x())) / lRay.d.x();
		// Get the smaller result (if one)
		tc = (tc.y() < tc.x()) ? Vector2f(tc.y(), tc.x()) : tc;
		if (inRng(tc.x(), lRay.mint, lRay.maxt) && (tc.x() < its.t)) {
			Point3f p = lRay.o + tc.x() * lRay.d;
			if (inRng(p.y(), min.y(), max.y()) && inRng(p.z(), min.z(), max.z())) {
				// Valid intersection
				its.t = tc.x();
				hit = true;
				its.p = m_toWorld * p;
				/*its.geoFrame = Frame(m_toWorld * Vector3f(p.x(), 0.0f, 0.0f).normalized());
				its.shFrame = its.geoFrame;
				its.mesh = this;*/
			}
		}
		else if (inRng(tc.y(), lRay.mint, lRay.maxt) && (tc.x() < its.t)) {
			Point3f p = lRay.o + tc.y() * lRay.d;
			if (inRng(p.y(), min.y(), max.y()) && inRng(p.z(), min.z(), max.z())) {
				// Valid intersection
				its.t = tc.y();
				hit = true;
				its.p = m_toWorld * p;
				/*its.geoFrame = Frame(m_toWorld * Vector3f(p.x(), 0.0f, 0.0f).normalized());
				its.shFrame = its.geoFrame;
				its.mesh = this;*/
			}
		}

		// Check x/z planes
		tc = (Vector2f(min.y(), max.y()) - Vector2f(lRay.o.y())) / lRay.d.y();
		// Get the smaller result (if one)
		tc = (tc.y() < tc.x()) ? Vector2f(tc.y(), tc.x()) : tc;
		if (inRng(tc.x(), lRay.mint, lRay.maxt) && (tc.x() < its.t)) {
			Point3f p = lRay.o + tc.x() * lRay.d;
			if (inRng(p.x(), min.x(), max.x()) && inRng(p.z(), min.z(), max.z())) {
				// Valid intersection
				its.t = tc.x();
				hit = true;
				its.p = m_toWorld * p;
				/*its.geoFrame = Frame(m_toWorld * Vector3f(p.y(), 0.0f, 0.0f).normalized());
				its.shFrame = its.geoFrame;
				its.mesh = this;*/
			}
		}
		else if (inRng(tc.y(), lRay.mint, lRay.maxt) && (tc.x() < its.t)) {
			Point3f p = lRay.o + tc.y() * lRay.d;
			if (inRng(p.x(), min.x(), max.x()) && inRng(p.z(), min.z(), max.z())) {
				// Valid intersection
				its.t = tc.y();
				hit = true;
				its.p = m_toWorld * p;
				/*its.geoFrame = Frame(m_toWorld * Vector3f(p.y(), 0.0f, 0.0f).normalized());
				its.shFrame = its.geoFrame;
				its.mesh = this;*/
			}
		}

		// Lastly, check x/y planes
		tc = (Vector2f(min.z(), max.z()) - Vector2f(lRay.o.z())) / lRay.d.z();
		// Get the smaller result (if one)
		tc = (tc.y() < tc.x()) ? Vector2f(tc.y(), tc.x()) : tc;
		if (inRng(tc.x(), lRay.mint, lRay.maxt) && (tc.x() < its.t)) {
			Point3f p = lRay.o + tc.x() * lRay.d;
			if (inRng(p.x(), min.x(), max.x()) && inRng(p.y(), min.y(), max.y())) {
				// Valid intersection
				its.t = tc.x();
				hit = true;
				its.p = m_toWorld * p;
				/*its.geoFrame = Frame(m_toWorld * Vector3f(p.z(), 0.0f, 0.0f).normalized());
				its.shFrame = its.geoFrame;
				its.mesh = this;*/
			}
		}
		else if (inRng(tc.y(), lRay.mint, lRay.maxt) && (tc.x() < its.t)) {
			Point3f p = lRay.o + tc.y() * lRay.d;
			if (inRng(p.x(), min.x(), max.x()) && inRng(p.y(), min.y(), max.y())) {
				// Valid intersection
				its.t = tc.y();
				hit = true;
				its.p = m_toWorld * p;
				/*its.geoFrame = Frame(m_toWorld * Vector3f(p.z(), 0.0f, 0.0f).normalized());
				its.shFrame = its.geoFrame;
				its.mesh = this;*/
			}
		}

		return hit;

	}
	else {
		throw NoriException("To implement!");
	}
}

void MediaContainer::sampleSurface(ShapeQueryRecord & sRec, const Point2f & sample) const
{
	throw NoriException("Not yet implemented");
}

float MediaContainer::pdfSurface(const ShapeQueryRecord & sRec) const
{
	if (!m_isVDB) {
		// For default 1x1x1 cube, scale denotes the edge lenght of the box
		return 1.0f / (m_scale.dot(m_scale));
	}
	else {
		throw NoriException("Not yet implemented");
	}
}

NORI_REGISTER_CLASS(MediaContainer, "media_container");
NORI_NAMESPACE_END
