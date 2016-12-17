#include <Eigen/Geometry>
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

	m_toLocal = Eigen::Translation3f(trans) * Eigen::AngleAxisf(rotAngle, rotAxis);
	m_toWorld = m_toLocal.inverse();
}

Vector3f MediaContainer::samplePhaseFunction(Point3f pos, Point2f sample) const {
		
	if (!m_isVDB) {
		// Homogeneous system
		return m_homogeneous.p->sample(sample);
	}
	else {
		Point3f posL = m_toLocal * pos;
			
		throw NoriException("VDB to implement!");
	}

}

float MediaContainer::getAbsorbtion(Point3f pos) const {
	if (!m_isVDB) {
		// Homogeneous system
		return m_homogeneous.a;
	}
	else {
		Point3f posL = m_toLocal * pos;

		throw NoriException("VDB to implement!");
	}
}

float MediaContainer::getScattering(Point3f pos) const {
	if (!m_isVDB) {
		// Homogeneous system
		return m_homogeneous.s;
	}
	else {
		Point3f posL = m_toLocal * pos;

		throw NoriException("VDB to implement!");
	}
}

float MediaContainer::getExtinction(Point3f pos) const {
	if (!m_isVDB) {
		// Homogeneous system
		return m_homogeneous.s + m_homogeneous.a;
	}
	else {
		Point3f posL = m_toLocal * pos;

		throw NoriException("VDB to implement!");
	}
}

float MediaContainer::getMajExtinction() const {
	return m_majExtinction;
}

bool MediaContainer::withinContainer(Point3f pos) const {

	Point3f posL = m_toLocal * pos;
		
	if (!m_isVDB) {
		// Homogeneous system
		float x = m_scale.x() / 2.0f;
		float y = m_scale.y() / 2.0f;
		float z = m_scale.z() / 2.0f;
		if (pos.x() < x && pos.x() > -x
			&& pos.y() < y && pos.y() > -y
			&& pos.z() < z && pos.z() > -z) {
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

std::string MediaContainer::toString() const {
	return tfm::format(
		"Mesh[\n"
		"  name = \"%s\",\n]",
		m_name
		);

}

NORI_REGISTER_CLASS(MediaContainer, "media_container");
NORI_NAMESPACE_END