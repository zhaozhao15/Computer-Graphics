#include "mesh.h"

namespace Orchid
{
	Mesh::Mesh(Vector3d p_, const char * file_path, Material m_)
	{
		_p = p_, m_m = m_;

		std::string mtlbasepath;
		std::string inputfile = file_path;
		unsigned long pos = inputfile.find_last_of("/");
		mtlbasepath = inputfile.substr(0, pos + 1);

		printf("Loading %s...\n", file_path);
		// Attempt to load mesh
		std::string err = tinyobj::LoadObj(m_shapes, m_materials, inputfile.c_str(), mtlbasepath.c_str());

		if (!err.empty()) {
			std::cerr << err << std::endl;
			exit(1);
		}
		printf(" - Generating k-d tree...\n\n");

		long shapes_size, indices_size, materials_size;
		shapes_size = m_shapes.size();
		materials_size = m_materials.size();

		// Load materials/textures from obj
		// TODO: Only texture is loaded at the moment, need to implement material types and colours
		for (int i = 0; i < materials_size; i++) {
			std::string texture_path = "";

			if (!m_materials[i].diffuse_texname.empty()) {
				if (m_materials[i].diffuse_texname[0] == '/') texture_path = m_materials[i].diffuse_texname;
				texture_path = mtlbasepath + m_materials[i].diffuse_texname;
				materials.push_back(Material(DIFF, Vector3d(1, 1, 1), Vector3d(), texture_path.c_str()));
			}
			else {
				materials.push_back(Material(DIFF, Vector3d(1, 1, 1), Vector3d()));
			}

		}

		// Load triangles from obj
		int newsize = 200;	/////////////////////////////////////////////////////////
		for (int i = 0; i < shapes_size; i++) {
			indices_size = m_shapes[i].mesh.indices.size() / 3;
			for (size_t f = 0; f < indices_size; f++) {

				// Triangle vertex coordinates
				Vector3d v0_ = Vector3d(
					m_shapes[i].mesh.positions[m_shapes[i].mesh.indices[3 * f] * 3],
					m_shapes[i].mesh.positions[m_shapes[i].mesh.indices[3 * f] * 3 + 1],
					m_shapes[i].mesh.positions[m_shapes[i].mesh.indices[3 * f] * 3 + 2]
				)*newsize + _p;

				Vector3d v1_ = Vector3d(
					m_shapes[i].mesh.positions[m_shapes[i].mesh.indices[3 * f + 1] * 3],
					m_shapes[i].mesh.positions[m_shapes[i].mesh.indices[3 * f + 1] * 3 + 1],
					m_shapes[i].mesh.positions[m_shapes[i].mesh.indices[3 * f + 1] * 3 + 2]
				)*newsize + _p;

				Vector3d v2_ = Vector3d(
					m_shapes[i].mesh.positions[m_shapes[i].mesh.indices[3 * f + 2] * 3],
					m_shapes[i].mesh.positions[m_shapes[i].mesh.indices[3 * f + 2] * 3 + 1],
					m_shapes[i].mesh.positions[m_shapes[i].mesh.indices[3 * f + 2] * 3 + 2]
				)*newsize + _p;

				Vector3d t0_, t1_, t2_;

				//Attempt to load triangle texture coordinates
				if (m_shapes[i].mesh.indices[3 * f + 2] * 2 + 1 < m_shapes[i].mesh.texcoords.size()) {
					t0_ = Vector3d(
						m_shapes[i].mesh.texcoords[m_shapes[i].mesh.indices[3 * f] * 2],
						m_shapes[i].mesh.texcoords[m_shapes[i].mesh.indices[3 * f] * 2 + 1],
						0
					);

					t1_ = Vector3d(
						m_shapes[i].mesh.texcoords[m_shapes[i].mesh.indices[3 * f + 1] * 2],
						m_shapes[i].mesh.texcoords[m_shapes[i].mesh.indices[3 * f + 1] * 2 + 1],
						0
					);

					t2_ = Vector3d(
						m_shapes[i].mesh.texcoords[m_shapes[i].mesh.indices[3 * f + 2] * 2],
						m_shapes[i].mesh.texcoords[m_shapes[i].mesh.indices[3 * f + 2] * 2 + 1],
						0
					);
				}
				else {
					t0_ = Vector3d();
					t1_ = Vector3d();
					t2_ = Vector3d();
				}

				if (m_shapes[i].mesh.material_ids[f] < materials.size())
					tris.push_back(new Triangle(v0_, v1_, v2_, t0_, t1_, t2_, materials[m_shapes[i].mesh.material_ids[f]]));
				else
					tris.push_back(new Triangle(v0_, v1_, v2_, t0_, t1_, t2_, m_m));
			}
		}

		// Clean up
		m_shapes.clear();
		m_materials.clear();
		node = KDNode().build(tris, 0);
		printf("\n");
	}
	ObjectIntersection Mesh::getIntersection(const Ray & r)
	{
		double t = 0, tmin = INFINITY;
		Vector3d normal = Vector3d();
		Vector3d colour = Vector3d();
		bool hit = node->hit(node, r, t, tmin, normal, colour);
		return ObjectIntersection(hit, tmin, normal, m_m);

	}

	KDNode* KDNode::build(std::vector<Triangle*> &tris, int depth) {
		KDNode* node = new KDNode();
		node->leaf = false;
		node->triangles = std::vector<Triangle*>();
		node->left = NULL;
		node->right = NULL;
		node->box = AABBox();

		if (tris.size() == 0) return node;

		if (depth > 25 || tris.size() <= 6) {
			node->triangles = tris;
			//printf("size: %d\n", node->triangles.size());
			node->leaf = true;
			node->box = tris[0]->get_bounding_box();
			for (long i = 1; i<tris.size(); i++) {
				node->box.expand(tris[i]->get_bounding_box());
			}
			//printf("box: %f %f %f\n", node->box.tr.x(), node->box.tr.y(), node->box.tr.z());
			node->left = new KDNode();
			node->right = new KDNode();
			node->left->triangles = std::vector<Triangle*>();
			node->right->triangles = std::vector<Triangle*>();

			return node;
		}

		node->box = tris[0]->get_bounding_box();
		Vector3d midpt = Vector3d();
		double tris_recp = 1.0 / tris.size();

		for (long i = 1; i<tris.size(); i++) {
			node->box.expand(tris[i]->get_bounding_box());
			midpt = midpt + (tris[i]->get_midpoint() * tris_recp);
		}

		std::vector<Triangle*> left_tris;
		std::vector<Triangle*> right_tris;
		int axis = node->box.get_longest_axis();

		for (long i = 0; i<tris.size(); i++) {
			switch (axis) {
			case 0:
				midpt.x() >= tris[i]->get_midpoint().x() ? right_tris.push_back(tris[i]) : left_tris.push_back(tris[i]);
				break;
			case 1:
				midpt.y() >= tris[i]->get_midpoint().y() ? right_tris.push_back(tris[i]) : left_tris.push_back(tris[i]);
				break;
			case 2:
				midpt.z() >= tris[i]->get_midpoint().z() ? right_tris.push_back(tris[i]) : left_tris.push_back(tris[i]);
				break;
			}
		}

		if (tris.size() == left_tris.size() || tris.size() == right_tris.size()) {
			node->triangles = tris;
			node->leaf = true;
			node->box = tris[0]->get_bounding_box();

			for (long i = 1; i<tris.size(); i++) {
				node->box.expand(tris[i]->get_bounding_box());
			}
			node->left = new KDNode();
			node->right = new KDNode();
			node->left->triangles = std::vector<Triangle*>();
			node->right->triangles = std::vector<Triangle*>();

			return node;
		}

		node->left = build(left_tris, depth + 1);
		node->right = build(right_tris, depth + 1);

		return node;
	}

	// Finds nearest triangle in kd tree that intersects with ray.
	bool KDNode::hit(KDNode *node, const Ray &ray, double &t, double &tmin, Vector3d &normal, Vector3d &c) {
		double dist;
		if (node->box.intersection(ray, dist)) {
			if (dist > tmin) return false;
			bool hit_tri = false;
			bool hit_left = false;
			bool hit_right = false;
			long tri_idx;

			if (!node->leaf) {
				hit_left = hit(node->left, ray, t, tmin, normal, c);

				hit_right = hit(node->right, ray, t, tmin, normal, c);

				return hit_left || hit_right;
			}
			else {
				long triangles_size = node->triangles.size();
				for (long i = 0; i<triangles_size; i++) {
					if (node->triangles[i]->intersection(ray)) {
						hit_tri = true;

						Vector3d o = ray.origin();
						Vector3d d = ray.direction();
						Vector3d Normal = node->triangles[i]->normal().normalized();
						double c1 = Normal.dot(node->triangles[i]->_p0);
						double t1 = (c1 - Normal.dot(o)) / (Normal.dot(d));
						tmin = t1;
						tri_idx = i;
					}
				}
				if (hit_tri) {
					Vector3d p = ray.origin() + ray.direction() * tmin;
					normal = node->triangles[tri_idx]->normal();
					c = node->triangles[tri_idx]->get_colour_at(p);
					return true;
				}
			}
		}
		//c = Vector3d(233.0f, 1.0f, 2.0f);
		return false;
	}
}
