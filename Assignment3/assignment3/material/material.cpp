#include "material.h"
#include "../myrand.h"
namespace Orchid
{

	Material::Material
	(MaterialType t, Vector3d colour, Vector3d emission, Texture texture)
		:_type{ t }
		, _colour{ colour }
		, _emission{ emission }
		, _texture{ texture }  //nead addd
	{

	}

	Material & Material::operator=(Material  m)
	{
		this->_type = m._type;
		this->_emission = m._emission;
		this->_texture = m._texture;
		this->_colour = m._colour;
		return *this;
	}

	Vector3d Material::get_colour_at(double u, double v) const
	{
		if (_texture.is_loaded())
			return _texture.get_pixel(u, v);
		return _colour;
	}

	Ray Material::get_reflected_ray
	(
		const Ray & r,
		Vector3d & p,
		const Vector3d & n,
		unsigned short * Xi
	) const
	{
		if (_type == SPEC) {
			double roughness = 0;
			Vector3d reflected = r.direction() - n * 2.0 * n.dot(r.direction());
			reflected = Vector3d(//jitter
				reflected.x() + (myrand(Xi) - 0.5)*roughness,
				reflected.y() + (myrand(Xi) - 0.5)*roughness,
				reflected.z() + (myrand(Xi) - 0.5)*roughness
			).normalized();

			return Ray(p, -reflected);
		}
		// Ideal diffuse reflection
		if (_type == DIFF) {
			Vector3d nl = n.dot(r.direction())<0 ? n : n * (-1.0);
			double r1 = 2 * PI*myrand(Xi), r2 = myrand(Xi), r2s = sqrt(r2);
			Vector3d w = nl;
			Vector3d u;
			if (fabs(w.x()) > .1)
				u = Vector3d(0, 1).cross(w).normalized();
			else
				u = Vector3d(1).cross(w).normalized();
			Vector3d v = w.cross(u);
			Vector3d d = (u*cos(r1)*r2s + v * sin(r1)*r2s + w * sqrt(1 - r2)).normalized();
			return Ray(p, d);
		}
	}

}