#pragma once

#include "curve.hpp"

namespace grapholon {
	class CurveDeformer {

	public:
		static bool deform_curve(DeformableSplineCurve& curve, bool source_control_point, Vector3f target_position) {
			return false;
		}

	};

}

#if 0
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include "CGAL/Surface_mesh_deformation.h"


#include "curve.hpp"



namespace grapholon {
	typedef CGAL::Simple_cartesian<double>                                   Kernel;
	typedef Kernel::Point_3 Point_3;
	//typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3> Polyhedron;
	typedef CGAL::Surface_mesh<Point_3> Polyhedron;
	typedef boost::graph_traits<Polyhedron>::vertex_descriptor    vertex_descriptor;
	typedef boost::graph_traits<Polyhedron>::vertex_iterator        vertex_iterator;
	typedef boost::graph_traits<Polyhedron>::halfedge_descriptor halfedge_descriptor;
	typedef boost::graph_traits<Polyhedron>::halfedge_iterator    halfedge_iterator;

	//typedef CGAL::Surface_mesh_deformation<Polyhedron> Surface_mesh_deformation;

	// a simple model to `SurfaceMeshDeformationWeights` concept, which provides uniform weights
	struct Identity_weight
	{
		template<class VertexPointMap>
		double operator()(typename boost::graph_traits<Polyhedron>::halfedge_descriptor  /*e*/, const Polyhedron& /*p*/, VertexPointMap /*v*/)
		{
			return 1.0;
		}
	};

	typedef std::map<vertex_descriptor, std::size_t>   Internal_vertex_map;
	typedef std::map<halfedge_descriptor, std::size_t>     Internal_hedge_map;
	typedef boost::associative_property_map<Internal_vertex_map>   Vertex_index_map;
	typedef boost::associative_property_map<Internal_hedge_map>     Hedge_index_map;

	typedef CGAL::Surface_mesh_deformation<Polyhedron, Vertex_index_map, Hedge_index_map, CGAL::ORIGINAL_ARAP, Identity_weight> Surface_mesh_deformation;

	class CurveDeformer {


	public:

		static Point_3 to_point3(Vector3f vec) {
			return Point_3(vec.X(), vec.Y(), vec.Z());
		}

		static Vector3f to_vec3(Point_3 point) {
			return Vector3f(point.x(), point.y(), point.z());
		}

		static bool deform_curve(DeformableSplineCurve& curve, bool source_control_point, Vector3f target_position) {
			Polyhedron mesh;

			//todo : handle size = 2 case

			std::vector<vertex_descriptor> mesh_vertices;

			for (GRuint i(0); i < curve.size(); i++) {
				mesh_vertices.push_back(mesh.add_vertex(to_point3(curve[i].first)));
			}

			for (GRuint i(1); i < curve.size() - 1; i++) {
				mesh.add_face(mesh_vertices[i], mesh_vertices[i - 1], mesh_vertices[i + 1]);
			}


			halfedge_iterator eb, ee;

			// Create and initialize the vertex index map
			Internal_vertex_map internal_vertex_index_map;
			Vertex_index_map vertex_index_map(internal_vertex_index_map);


			vertex_iterator vb, ve;
			std::size_t counter = 0;
			for (boost::tie(vb, ve) = vertices(mesh); vb != ve; ++vb, ++counter) {
				put(vertex_index_map, *vb, counter);
			}
			// Create and initialize the halfedge index map
			Internal_hedge_map internal_hedge_index_map;
			Hedge_index_map hedge_index_map(internal_hedge_index_map);
			counter = 0;
			for (boost::tie(eb, ee) = halfedges(mesh); eb != ee; ++eb, ++counter) {
				put(hedge_index_map, *eb, counter);
			}

			Surface_mesh_deformation deform_mesh(mesh,
				vertex_index_map,
				hedge_index_map,
				get(CGAL::vertex_point, mesh),
				Identity_weight());

			// Init the indices of the halfedges and the vertices.
			//set_halfedgeds_items_id(mesh);
			// Create a deformation object
			//Surface_mesh_deformation deform_mesh(mesh,);
			// Definition of the region of interest (use the whole mesh)
			//vertex_iterator vb, ve;
			boost::tie(vb, ve) = vertices(mesh);
			deform_mesh.insert_roi_vertices(vb, ve);
			// Select two control vertices ...

			vertex_descriptor source_point = mesh_vertices[0];
			vertex_descriptor target_point = mesh_vertices.back();

			vertex_descriptor control = source_control_point ? source_point : target_point;
			// ... and insert them
			deform_mesh.insert_control_vertex(source_point);
			deform_mesh.insert_control_vertex(target_point);
			// The definition of the ROI and the control vertices is done, call preprocess
			bool is_matrix_factorization_OK = deform_mesh.preprocess();
			if (!is_matrix_factorization_OK) {
				std::cerr << "Error in preprocessing, check documentation of preprocess()" << std::endl;
				return false;
			}
			// Use set_target_position() to set the constained position
			// of control_1. control_2 remains at the last assigned positions
			Surface_mesh_deformation::Point constrained_pos(to_point3(target_position));
			deform_mesh.set_target_position(control, constrained_pos);
			// Deform the mesh, the positions of vertices of 'mesh' are updated
			deform_mesh.deform();
			// The function deform() can be called several times if the convergence has not been reached yet
			deform_mesh.deform();

			counter = 0;
			for (boost::tie(vb, ve) = vertices(mesh); vb != ve; ++vb, ++counter) {
				curve[counter].first = to_vec3(mesh.point(mesh_vertices[counter]));
			}

			curve.update_tangents();

			return true;
		}

	};
};

#endif