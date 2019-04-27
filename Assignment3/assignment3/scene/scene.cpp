#include "scene.h"
#include <random> 
#include "../sample/sampler.h"
#include <iostream>
#include <ctime>
#include <cstdlib>
#include "../myrand.h"
namespace Orchid
{


	void Scene::add(Object * object)
	{
		_objects.push_back(object);
	}

	ObjectIntersection Scene::intersect(const Ray & ray)
	{
		ObjectIntersection isct = ObjectIntersection();
		ObjectIntersection temp;
		long size = _objects.size();

		for (int i = 0; i < size; i++)//search all the objects to get a hit
		{
			temp = _objects.at((unsigned)i)->getIntersection(ray);
			if (temp._hit) {
				if (isct._u == 0 || temp._u < isct._u) isct = temp;
			}
		}
		return isct;
	}

	Vector3d Scene::trace_ray(const Ray & ray, int depth, unsigned short * Xi)
	{
		ObjectIntersection isct = intersect(ray);
		// If no hit, return world colour
		if (!isct._hit) return Vector3d();

		if (isct._material.getType() == EMIT) return isct._material.get_emission();
		Vector3d colour = isct._material.get_colour();

		// Calculate max reflection
		double p = colour.x()>colour.y() && colour.x()>colour.z() ? colour.x() : colour.y()>colour.z() ? colour.y() : colour.z();
		

		// Russian roulette termination.
		// If random number between 0 and 1 is > p, terminate and return hit object's emmission
		double rnd = myrand(Xi);
		if (++depth>8) {
			if (rnd<p*0.9) { // Multiply by 0.9 to avoid infinite loop with colours of 1.0
				colour = colour * (0.9 / p);
			}
			else {
				return isct._material.get_emission();
			}
		}

		Vector3d x = ray.origin() + ray.direction() * isct._u;
		Ray reflected = isct._material.get_reflected_ray(ray, x, isct._normal, Xi);

		Vector3d light = { 214.0f, 548.6f, 227.0f };
		Vector3d direct_direction = (light - x).normalized();
		Ray direct_light = Ray(x, direct_direction);
		ObjectIntersection direct_hit = intersect(direct_light);
		Vector3d direct_color = Vector3d();
		if (direct_hit._material.getType() == EMIT) {
			double cos_direct = direct_direction.dot(ray.direction());
			if (cos_direct >= 0) {
				direct_color = cos_direct * Vector3d{ 1.0, 1.0, 1.0 };
			}
		}

		return colour * trace_ray(reflected, depth, Xi) + direct_color;
		//return colour;
	}
}