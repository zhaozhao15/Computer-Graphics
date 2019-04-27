#include "quad.h"
namespace Orchid
{
	Quad::Quad(
		const Vector3d & a,
		const Vector3d & b,
		const Vector3d & c,
		const Vector3d & d,
		const Material & material)
		: _a{ a }
		, _b{ b }
		, _c{ c }
		, _d{ d }
		, _material{ material }
	{
	}
	ObjectIntersection Quad::getIntersection(const Ray & ray)
	{
		//triangle abc
		const Vector3d e1 = _b - _a;
		const Vector3d e2 = _c - _a;
		Vector3d n = Vector3d::cross(e1, e2).normalized();
		Vector3d o = ray.origin();
		Vector3d d = ray.direction();
		//Vector3d Normal = Vector3d::cross(_a - _b, _b - _c).normalized();
		double c = n.dot(_a);
		double t = (c - n.dot(o)) / (n.dot(d));
		Vector3d cor = o + d * t;

		double area0 = 0.5 * Vector3d::cross(e1, e2).norm();

		const Vector3d v10 = _a - cor;
		const Vector3d v11 = _b - cor;
		double area1 = 0.5 * Vector3d::cross(v10, v11).norm();

		const Vector3d v20 = _a - cor;
		const Vector3d v21 = _c - cor;
		double area2 = 0.5 * Vector3d::cross(v20, v21).norm();

		const Vector3d v30 = _b - cor;
		const Vector3d v31 = _c - cor;
		double area3 = 0.5 * Vector3d::cross(v30, v31).norm();

		if (-area0 + (area1 + area2 + area3) <= area0*0.01) {
			return ObjectIntersection(true, t, n, _material);
		}else {
			//triangle acd
			const Vector3d e3 = _a - _d;
			const Vector3d e4 = _a - _c;
			double area0_ = 0.5 * Vector3d::cross(e3, e4).norm();

			const Vector3d v10_ = _a - cor;
			const Vector3d v11_ = _d - cor;
			double area1_ = 0.5 * Vector3d::cross(v10_, v11_).norm();

			const Vector3d v20_ = _c - cor;
			const Vector3d v21_ = _d - cor;
			double area2_ = 0.5 * Vector3d::cross(v20_, v21_).norm();

			const Vector3d v30_ = _a - cor;
			const Vector3d v31_ = _c - cor;
			double area3_ = 0.5 * Vector3d::cross(v30_, v31_).norm();

			if ((area1_ + area2_ + area3_) - area0_ <= area0_ * 0.01) {
				return ObjectIntersection(true, t, n, _material);
			}
			else {
				return ObjectIntersection(false, 0.0, Vector3d(0), Material(DIFF, Vector3d(0.0)));
			}
		}
		
	}
}
