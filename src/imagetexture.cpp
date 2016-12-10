/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Romain Prévost

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

#include <nori/texture.h>
#include <nori/lodepng.h>
#include <filesystem/resolver.h>

NORI_NAMESPACE_BEGIN

template <typename T>
class ImageTexture : public Texture<T> {
public:
	ImageTexture(const PropertyList &props);

    virtual std::string toString() const override;

    virtual T eval(const Point2f & uv) override {
        return m_value;
    }

protected:
	std::string m_filename;
	filesystem::path m_filepath;
	std::vector<Color4f> m_image;
	int m_width;
	int m_height;
	unsigned m_loadErr;
};

template <>
ImageTexture<Color3f>::ImageTexture(const PropertyList &props) {
    m_filename = props.getString("filename", std::string());
	filesystem::path m_filepath =
		getFileResolver()->resolve(props.getString("filename"));

	// If there was a filename, try to load the image. Else abort
	if (m_filename.empty()) {
		m_loadErr = 1;
		return;
	}
	
	unsigned w, h;
	std::vector<unsigned char> image;
	m_loadErr = lodepng::decode(image, w, h, m_filepath.str());
	// Assumin no image size > than max-int.
	m_width = (int) w;
	m_height = (int) h;

	m_image.clear();

	// run through the image and collect the pixel colors
	for (int i = (m_width * m_height - 1); i >= 0 ; i--) {
		int ind = i * 4;

		float r = ((unsigned int)image[ind]) / 255.0f;
		float g = ((unsigned int)image[ind+1]) / 255.0f;
		float b = ((unsigned int)image[ind+2]) / 255.0f;
		float a = ((unsigned int)image[ind+3]) / 255.0f;

		if (r < 0.0f || g < 0.0f || b < 0.0f || a < 0.0f) {
			printf("Invalid pixel!");
		}

		m_image.push_back(Color4f(r, g, b, a));
	}

}

template<>
Color3f ImageTexture<Color3f>::eval(const Point2f & uv) {

	if (m_loadErr != 0.0f)
		return Color3f(0.0f, 1.0f, 0.0f);

	// Read out the pixel values - may not be the most efficient method (running through the vector many times)
	// Bilinear filter
	int x1 = std::floor(uv.x() * m_width);
	int x2 = std::ceil(uv.x() * m_width);
	int y1 = std::ceil(uv.y() * m_height);
	int y2 = std::floor(uv.y() * m_height);

	float x = uv.x() * m_width;
	float y = uv.y() * m_height;

	// If one difference 0 -> problem with division below
	if (x2 - x1 == 0.0f || y2 - y1 == 0.0f) {
		Color4f t0 = m_image[y1 * m_width + x1];
		return Color3f(t0.x(), t0.y(), t0.z());
	}

	Color4f t1 = (x2 - x) / (x2 - x1) * m_image[y1 * m_width + x1] + (x - x1) / (x2 - x1) * m_image[y1 * m_width + x2];
	Color4f t2 = (x2 - x) / (x2 - x1) * m_image[y2 * m_width + x1] + (x - x1) / (x2 - x1) * m_image[y2 * m_width + x2];
	Color4f r = (y2 - y) / (y2 - y1) * t1 + (y - y1) / (y2 - y1) * t2;

	/* // DEBUG
	printf("BilFilter -- wx1: %.2f, wx2: %.2f, wx3: %.2f, wx4: %.2f, wy1: %.2f, wy2: %.2f\n", (x2 - x) / (x2 - x1), (x - x1) / (x2 - x1), (x2 - x) / (x2 - x1), (x - x1) / (x2 - x1), (y2 - y) / (y2 - y1), (y - y1) / (y2 - y1));
	Color4f t = m_image[y1 * m_width + x1];
	printf("  -- Color1: %.2f, %.2f, %.2f\n", t.x(), t.y(), t.z());
	t = m_image[y1 * m_width + x2];
	printf("  -- Color2: %.2f, %.2f, %.2f\n", t.x(), t.y(), t.z());
	t = m_image[y2 * m_width + x1];
	printf("  -- Color3: %.2f, %.2f, %.2f\n", t.x(), t.y(), t.z());
	t = m_image[y2 * m_width + x2];
	printf("  -- Color4: %.2f, %.2f, %.2f\n", t.x(), t.y(), t.z());*/

	return Color3f(r.x(), r.y(), r.z());
}

int indPixel(int x, int y, int width) {
	return (y * width + x);
}

Color4f getPixelColor(std::vector<unsigned char> image, int width, int height, int x, int y) {
	int numInputChannels = 4;
	int loc = numInputChannels * (width * y + x);
	float r = ((unsigned int) image[loc]) / 255.0f;
	float g = ((unsigned int) image[loc + 1]) / 255.0f;
	float b = ((unsigned int) image[loc + 2]) / 255.0f;
	float a = ((unsigned int)image[loc + 3]) / 255.0f;

	return Color4f(r, g, b, a);
}


template <>
std::string ImageTexture<Color3f>::toString() const {
    return tfm::format(
            "ImageTexture[ %s ]",
            m_filename);
}

NORI_REGISTER_TEMPLATED_CLASS(ImageTexture, Color3f, "image_texture")
NORI_NAMESPACE_END