/*
Orchid source code is Copyright(c) 2018
Li Wei.
This file is part of Orchid.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
- Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef _MSC_VER
#pragma once
#endif // !_MSC_VER

#ifndef _TRIANGLEH_
#define _TRIANGLEH_
#include "objetcs.h"
#include "../accel/aabbox.h"
namespace Orchid
{
	class Triangle :public Object
	{
	public:
		Vector3d _p0, _p1, _p2;
		Material _material;
		Vector3d t0 = Vector3d();
		Vector3d t1 = Vector3d();
		Vector3d t2 = Vector3d();

		Triangle();
		Triangle(const Vector3d& p0, const Vector3d&p1, const Vector3d&p2, Material material);
		Triangle(Vector3d v0_, Vector3d v1_, Vector3d v2_, Vector3d t0_, Vector3d t1_, Vector3d t2_, Material m_) {
			_p0 = v0_, _p1 = v1_, _p2 = v2_,
				t0 = t0_, t1 = t1_, t2 = t2_;
			_material = m_;
		}
		Triangle(const Triangle& tri);
		~Triangle();
		Triangle& operator=(const Triangle& tri);
		Vector3d gravity() const;
		Vector3d get_midpoint() {
			return (_p0 + _p1 + _p2) / 3;
		}

		// Returns barycentric coordinates of point p on the triangle
		Vector3d barycentric(Vector3d p) {
			Vector3d v2_ = p - _p0;
			const Vector3d e1 = _p1 - _p0;
			const Vector3d e2 = _p2 - _p0;
			double d00 = e1.dot(e1);
			double d01 = e1.dot(e2);
			double d11 = e2.dot(e2);
			double d20 = v2_.dot(e1);
			double d21 = v2_.dot(e2);
			double d = d00 * d11 - d01 * d01;
			double v = (d11*d20 - d01 * d21) / d;
			double w = (d00*d21 - d01 * d20) / d;
			double u = 1 - v - w;
			return Vector3d(u, v, w);
		}

		// Returns the colour at point p on the triangle
		Vector3d get_colour_at(Vector3d p) {
			if (_material.getType() == DIFF) return Vector3d(0.9f, 0.9f, 0.0f);

			Vector3d b = barycentric(p);
			Vector3d c = Vector3d();
			c = c + (t0 * b.x());
			c = c + (t1 * b.y());
			c = c + (t2 * b.z());

			return _material.get_colour_at(c.x(), c.y());
		}

		AABBox get_bounding_box();
		bool triangleInBbox();
		Vector3d get(int id) const;
		Vector3d normal() const;
		virtual ObjectIntersection getIntersection(const Ray& ray);
		bool Triangle::intersection(const Ray & ray);
		double area() const;
	};

}
#endif // !_TRAIANGLEH_
