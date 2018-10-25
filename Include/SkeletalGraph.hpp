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

#include "Curve.hpp"

namespace grapholon {

	struct VertexProperties {
		Vector3f position;
	};

	typedef std::pair<GRuint, GRuint> Edge;

	typedef std::vector<Edge> OutEdgeList;

	struct EdgeProperties {
		SplineCurve curve;
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

		GRuint edge_spline_count_ = 0;

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

		GRuint vertex_count() const {
			return (GRuint)internal_graph_.m_vertices.size();
		}

		GRuint edge_count() const {
			return (GRuint)internal_graph_.m_edges.size();
		}

		GRuint edge_spline_count() const {
			return edge_spline_count_;
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
			edge_spline_count_ += (GRuint)properties.curve.size();
			return boost::add_edge(from, to, properties, internal_graph_);
		}

		void remove_edge(EdgeDescriptor edge) {
			edge_spline_count_ -= (GRuint)internal_graph_[edge].curve.size();

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

		std::string to_string() {
			std::stringstream msg;

			msg << " SkeletalGraph contains " << internal_graph_.m_vertices.size() << " vertices and " << internal_graph_.m_edges.size() << " edges :" << std::endl;


			std::pair<VertexIterator, VertexIterator> vp;

			GRuint iteration_count(0);

			msg << "------ vertices ------" << std::endl;
			for (vp = boost::vertices(internal_graph_); vp.first != vp.second; ++vp.first) {
				VertexDescriptor v = *vp.first;
				msg << iteration_count << " : " << std::endl << internal_graph_[v].position.to_string() << std::endl;

				iteration_count++;
			}
			msg << std::endl << std::endl;

			std::pair<EdgeIterator, EdgeIterator> ep;

			iteration_count = 0;

			msg << "------- edges -------" << std::endl;
			for (ep = boost::edges(internal_graph_); ep.first != ep.second; ++ep.first) {
				EdgeDescriptor e = *ep.first;
				msg << iteration_count << " : " << std::endl << internal_graph_[e].curve.to_string() << std::endl;

				iteration_count++;
			}
			msg << std::endl;

			return msg.str();
		}


	};
}