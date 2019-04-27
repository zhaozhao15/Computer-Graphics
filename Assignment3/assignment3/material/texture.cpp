#include "texture.h"

namespace Orchid
{
	Texture::Texture(const char * filename)
	{
	}
	Vector3d Texture::get_pixel(unsigned x, unsigned y) const
	{
		if (!loaded)
			return (Vector3d(1, 0, 1));

		double r, g, b;
		r = (double)_image.at(y*_width * 4 + x) / 255.;
		g = (double)_image.at(y*_width * 4 + x + 1) / 255.;
		b = (double)_image.at(y*_width * 4 + x + 2) / 255.;
		return Vector3d(r, g, b);
	}
	Vector3d Texture::get_pixel(double u, double v) const
	{
		if (!loaded)
			return (Vector3d(1, 0, 1));

		int x = (fmod(fabs(u), 1.0)) * (_width - 1);
		int y = (1. - fmod(fabs(v), 1.0)) * (_height - 1);
		//printf("%f, %f\n", u, v);
		double r, g, b;
		try {
			r = (double)_image.at(y*_width * 4 + x * 4) / 255.;
			g = (double)_image.at(y*_width * 4 + x * 4 + 1) / 255.;
			b = (double)_image.at(y*_width * 4 + x * 4 + 2) / 255.;
			return Vector3d(r, g, b);
		}
		catch (const std::out_of_range& e) {
			printf("error with uv, yx: %lf, %lf - %i, %i (width, height: %i, %i) \n", u, v, x, y, _width, _height);
			return Vector3d(0, 1, 0);
		}
	}
	bool Texture::is_loaded() const
	{
		return loaded;
	}
}
