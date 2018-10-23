//Copyright (c) 2018 Valentin NIGOLIAN
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.


#pragma once

#include "boost/graph/adjacency_list.hpp"

#include "common.hpp"
#include "GrapholonTypes.hpp"

struct Point3d {
	GRfloat X = 0.f;
	GRfloat Y = 0.f;
	GRfloat Z = 0.f;
};

typedef std::vector<Point3d> Curve3d;

struct VertexProperties {
	Point3d position;
};

typedef std::pair<GRuint, GRuint> Edge;

typedef std::vector<Edge> OutEdgeList;

struct EdgeProperties {
	Curve3d curve;
};

/** Boost Graph Library stuff*/

typedef boost::adjacency_list<
	boost::listS, boost::listS, boost::undirectedS,
	VertexProperties, EdgeProperties>
	InternalBoostGraph;

typedef InternalBoostGraph::vertex_descriptor VertexDescriptor;
typedef InternalBoostGraph::edge_descriptor EdgeDescriptor;
typedef InternalBoostGraph::vertex_iterator VertexIterator;
typedef boost::graph_traits<InternalBoostGraph>::edge_iterator EdgeIterator;

class SkeletalGraph {
private:


	InternalBoostGraph internal_graph_;

	/** Vertices and edges */

public:
	SkeletalGraph(GRuint vertex_count, OutEdgeList out_edge_list) :
		internal_graph_(out_edge_list.begin(), out_edge_list.end(), vertex_count) {

		std::cout << "created SkeletalGraph with " << vertex_count << " vertices and " << out_edge_list.size() << " edges : " << std::endl;
	}

	SkeletalGraph(GRuint vertex_count = 0) :internal_graph_(vertex_count) {
		std::cout << "created SkeletalGraph with " << vertex_count << " vertices and no edges : " << std::endl;

	}

	/** size getters */

	GRuint vertex_count() const{
		return (GRuint)internal_graph_.m_vertices.size();
	}

	GRuint edge_count() const {
		return (GRuint)internal_graph_.m_edges.size();
	}

	/** Vertex stuff */
	VertexDescriptor add_vertex(VertexProperties properties) {
		return boost::add_vertex(properties, internal_graph_);
	}

	void remove_vertex(VertexDescriptor vertex) {
		boost::remove_vertex(vertex, internal_graph_);
	}

	VertexProperties& get_vertex(VertexDescriptor vertex) {
		return internal_graph_[vertex];
	}

	std::pair<VertexIterator, VertexIterator> vertices() {
		return boost::vertices(internal_graph_);
	}


	/** Edge stuff*/
	
	std::pair<EdgeDescriptor, bool> add_edge(VertexDescriptor from, VertexDescriptor to, EdgeProperties properties) {
		return boost::add_edge(from, to, properties, internal_graph_);
	}

	void remove_edge(EdgeDescriptor edge) {
		boost::remove_edge(edge, internal_graph_);
	}

	EdgeProperties& get_edge(EdgeDescriptor edge) {
		return internal_graph_[edge];
	}

	std::pair<EdgeIterator, EdgeIterator> edges() {
		return boost::edges(internal_graph_);
	}

	VertexProperties& get_edge_source(EdgeDescriptor edge) {
		return internal_graph_[boost::source(edge, internal_graph_)];
	}

	VertexProperties& get_edge_target(EdgeDescriptor edge) {
		return internal_graph_[boost::target(edge, internal_graph_)];
	}

	/** Print stuff */

	void print_composition() {
		std::cout << " SkeletalGraph contains " << internal_graph_.m_vertices.size() << " vertices and " << internal_graph_.m_edges.size() << " edges :" << std::endl;


		std::pair<VertexIterator, VertexIterator> vp;

		GRuint iteration_count(0);

		std::cout << "------ vertices ------" << std::endl;
		for (vp = boost::vertices(internal_graph_); vp.first != vp.second; ++vp.first) {
			VertexDescriptor v = *vp.first;
			Point3d pos = internal_graph_[v].position;
			std::cout << iteration_count<<" : "<<pos.X << " "<<pos.Y<<" "<<pos.Z<<std::endl;
			iteration_count++;
		}
		std::cout << std::endl<<std::endl;

		std::pair<EdgeIterator, EdgeIterator> ep;

		iteration_count = 0;

		std::cout << "------- edges -------" << std::endl;
		for (ep = boost::edges(internal_graph_); ep.first != ep.second; ++ep.first) {
			EdgeDescriptor e = *ep.first;
			Curve3d curve = internal_graph_[e].curve;

			VertexDescriptor from(boost::source(e, internal_graph_)), to(boost::target(e, internal_graph_));
			Point3d from_point(internal_graph_[from].position), to_point(internal_graph_[to].position);

			std::cout << iteration_count
				<< " : (" << from_point.X << " " << from_point.Y << " " << from_point.Z
				<< ") --> ";
			for (auto middle_point : curve) {
				std::cout << "(" << middle_point.X << " " << middle_point.Y << " " << middle_point.Z << ") --> ";
			}
			
			std::cout<<"(" << to_point.X << " " << to_point.Y << " " << to_point.Z
				<< ")" << std::endl;
			iteration_count++;
		}
		std::cout << std::endl;
	}
	
};
