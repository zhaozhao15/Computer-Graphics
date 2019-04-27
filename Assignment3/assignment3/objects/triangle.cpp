#include "triangle.h"

namespace Orchid
{
	Triangle::Triangle()
	{

	}
	Triangle::Triangle(const Vector3d & p0, const Vector3d & p1, const Vector3d & p2, Material material)
	{
		_p0 = p0;
		_p1 = p1;
		_p2 = p2;
		_material = material;
	}
	Triangle::Triangle(const Triangle & tri)
	{
		this->operator=(tri);
	}
	Triangle::~Triangle()
	{

	}
	Triangle & Triangle::operator=(const Triangle & tri)
	{
		// TODO: insert return statement here
		this->_p0 = tri._p0;
		this->_p1 = tri._p1;
		this->_p2 = tri._p2;
		this->_material = tri._material;
		return *this;
	}
	Vector3d Triangle::gravity() const
	{
		return Vector3d();
	}
	AABBox Triangle::get_bounding_box()
	{
		Vector3d bl = Vector3d(
			std::min(std::min(_p0.x(), _p1.x()), _p2.x()),
			std::min(std::min(_p0.y(), _p1.y()), _p2.y()),
			std::min(std::min(_p0.z(), _p1.z()), _p2.z())
		);
		Vector3d tr = Vector3d(
			std::max(std::max(_p0.x(), _p1.x()), _p2.x()),
			std::max(std::max(_p0.y(), _p1.y()), _p2.y()),
			std::max(std::max(_p0.z(), _p1.z()), _p2.z())
		);

		return AABBox(bl, tr);
	}
	bool Triangle::triangleInBbox()
	{
		return false;
	}
	Vector3d Triangle::get(int id) const
	{
		Assertion(0 <= id && id <= 2, "ID must be between 0 and 2");
		if (0 == id)
			return _p0;
		if (1 == id)
			return _p1;
		if (2 == id)
			return _p2;
	}
	Vector3d Triangle::normal() const
	{
		const Vector3d e1 = _p1 - _p0;
		const Vector3d e2 = _p2 - _p0;
		Vector3d result = Vector3d::cross(e1, e2);
		double a = sqrt(pow(result.x(), 2) + pow(result.y(), 2) + pow(result.z(), 2));
		result /= a;
		return result;
	}
	ObjectIntersection Triangle::getIntersection(const Ray & ray)
	{
		const Vector3d e1 = _p1 - _p0;
		const Vector3d e2 = _p2 - _p0;
		Vector3d n = Vector3d::cross(e1, e2).normalized();

		Vector3d o = ray.origin();
		Vector3d d = ray.direction();
		double c = n.dot(_p0);
		double t = (c - n.dot(o)) / (n.dot(d));
		Vector3d cor = o + d * t;

		double area0 = Triangle::area();

		const Vector3d v10 = _p0 - cor;
		const Vector3d v11 = _p1 - cor;
		double area1 = 0.5 * Vector3d::cross(v10, v11).norm();

		const Vector3d v20 = _p0 - cor;
		const Vector3d v21 = _p2 - cor;
		double area2 = 0.5 * Vector3d::cross(v20, v21).norm();

		const Vector3d v30 = _p1 - cor;
		const Vector3d v31 = _p2 - cor;
		double area3 = 0.5 * Vector3d::cross(v30, v31).norm();

		if (-area0 + (area1 + area2 + area3) < area0*0.1) {
			return ObjectIntersection(true, t, n, _material);
		}
		else {
			return ObjectIntersection(false, 0.0, Vector3d(0), Material(DIFF, Vector3d(0.0)));
		}
	}

	bool Triangle::intersection(const Ray & ray)
	{
		const Vector3d e1 = _p1 - _p0;
		const Vector3d e2 = _p2 - _p0;
		Vector3d n = Vector3d::cross(e1, e2);

		Vector3d o = ray.origin();
		Vector3d d = ray.direction();

		Vector3d Normal = Vector3d::cross(_p0 - _p1, _p1 - _p2).normalized();
		double c = Normal.dot(_p0);
		double t = (c - Normal.dot(o)) / (Normal.dot(d));
		Vector3d cor = o + d * t;

		double area0 = Triangle::area();
		const Vector3d v10 = _p0 - cor;
		const Vector3d v11 = _p1 - cor;
		double area1 = 0.5 * Vector3d::cross(v10, v11).norm();

		const Vector3d v20 = _p0 - cor;
		const Vector3d v21 = _p2 - cor;
		double area2 = 0.5 * Vector3d::cross(v20, v21).norm();

		const Vector3d v30 = _p1 - cor;
		const Vector3d v31 = _p2 - cor;
		double area3 = 0.5 * Vector3d::cross(v30, v31).norm();

		if (-area0 + (area1 + area2 + area3) < area0*0.1) {
			return true;
		}
		else {
			return false;
		}
	}


	double Triangle::area() const
	{
		const Vector3d e1 = _p1 - _p0;
		const Vector3d e2 = _p2 - _p0;
		return 0.5 * Vector3d::cross(e1, e2).norm();
	}
}
