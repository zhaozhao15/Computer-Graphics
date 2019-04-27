#include "sphere.h"
namespace  Orchid
{
	Sphere::Sphere(Vector3d position, double radius, Material material)

	{
		_position = position;
		_radius = radius;
		_material = material;
	}
	ObjectIntersection Sphere::getIntersection(const Ray & ray)
	{
		Vector3d o = ray.origin();
		Vector3d d = ray.direction();
		Vector3d v = o - _position;
		double eps = 0.0001;
		double delta = d.dot(v)*d.dot(v) - (v.dot(v) - _radius * _radius);
		double t;
		Vector3d isc = o + d * t;
		Vector3d nor = (_position - isc).normalized();
		double distance = (t = -d.dot(v) - sqrt(delta))>eps ? t : ((t = -d.dot(v) + sqrt(delta))>eps ? t : 0);
		if (delta < 0) return ObjectIntersection(false, 0, Vector3d(), _material);
		else if (distance!=0) return ObjectIntersection(true, distance, nor, _material);
		else return ObjectIntersection(false, distance, nor, _material);
	}

}
