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
#include "boost/graph/copy.hpp"

#include <queue>
#include <set>
#include <map>

#include <fstream>

#include "Curve.hpp"

//TODO : add sizes on each edge_to_collapse and vertex

namespace grapholon {

#define DEFAULT_VERTEX_RADIUS 1.f

	struct VertexProperties {
		Vector3f position;
		GRfloat radius = 1.f;
		bool is_part_of_cycle = false;///<used for cycle detection
		bool is_in_spanning_tree = false;///<used for cycle detection
		void* cycle_parent = nullptr;///<used for cycle detection

		void* BFS_parent = nullptr;
		GRuint BFS_path_cost = (GRuint)-1;
	};

	struct EdgeProperties {
		DeformableSplineCurve curve;
		bool is_part_of_cycle = false;
	};

	/** Boost Graph Library stuff*/
	typedef boost::adjacency_list<
		boost::listS, boost::listS, boost::bidirectionalS,
		VertexProperties, EdgeProperties>
		InternalBoostGraph;

	typedef InternalBoostGraph::vertex_descriptor VertexDescriptor;
	typedef InternalBoostGraph::edge_descriptor EdgeDescriptor;
	typedef InternalBoostGraph::vertex_iterator VertexIterator;
	typedef InternalBoostGraph::edge_iterator EdgeIterator;
	typedef InternalBoostGraph::in_edge_iterator InEdgeIterator;
	typedef InternalBoostGraph::out_edge_iterator OutEdgeIterator;

	//handy aliases
	typedef std::pair<VertexDescriptor, VertexDescriptor> VertexPair;
	typedef std::pair<EdgeDescriptor, EdgeDescriptor> EdgePair;

	typedef std::vector<VertexDescriptor> VertexVector;
	typedef std::vector<EdgeDescriptor> EdgeVector;

	typedef std::pair<VertexDescriptor, EdgeVector> VertexNeighborhood;

	/*struct GraphOperationResult {
		std::vector<std::pair<grapholon::VertexDescriptor, grapholon::VertexProperties>> added_vertices;
		std::vector<std::pair<grapholon::EdgeDescriptor, grapholon::EdgeProperties>> added_edges;

		std::vector<std::pair<grapholon::VertexDescriptor, grapholon::VertexProperties>> removed_vertices;
		std::vector<std::pair<grapholon::EdgeDescriptor, grapholon::EdgeProperties>> removed_edges;
	};*/

	class SkeletalGraph {
	private:

		GRuint edge_spline_count_ = 0;

		InternalBoostGraph internal_graph_;

		/** Vertices and edges */

	public:

		static VertexDescriptor null_vertex() {
			return InternalBoostGraph::null_vertex();
		}

		typedef enum { SOURCE, TARGET, MIDPOINT } COLLAPSE_OPTION;

		SkeletalGraph(GRuint vertex_count = 0) :internal_graph_(vertex_count) {
			std::cout << "created SkeletalGraph with " << vertex_count << " vertices and no edges : " << std::endl;

		}

		SkeletalGraph* copy() {
			SkeletalGraph* new_graph = new SkeletalGraph();
			//boost::copy_graph<InternalBoostGraph, InternalBoostGraph>(this->internal_graph_, new_graph->internal_graph_);
			new_graph->internal_graph_ = internal_graph_;
			return new_graph;
		}

		~SkeletalGraph() {

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



		/**************************************************************************************************** Vertex stuff */
		VertexDescriptor add_vertex(VertexProperties properties) {
			return boost::add_vertex(properties, internal_graph_);
		}

		EdgeVector remove_vertex(VertexDescriptor vertex) {
			if (vertex != InternalBoostGraph::null_vertex()) {
				std::vector<EdgeDescriptor> removed_edges = clear_vertex(vertex);
				boost::remove_vertex(vertex, internal_graph_);
				return removed_edges;
			}
			return std::vector<EdgeDescriptor>();
		}

		EdgeVector clear_vertex(VertexDescriptor vertex) {
			EdgeVector removed_edges;

			std::pair<InEdgeIterator, InEdgeIterator> in_edges = boost::in_edges(vertex, internal_graph_);
			std::pair<OutEdgeIterator, OutEdgeIterator> out_edges = boost::out_edges(vertex, internal_graph_);

			for (InEdgeIterator e_it(in_edges.first); e_it != in_edges.second; e_it++) {
				edge_spline_count_ -= (GRuint)internal_graph_[*e_it].curve.size();
				removed_edges.push_back(*e_it);
			}
			for (OutEdgeIterator e_it(out_edges.first); e_it != out_edges.second; e_it++) {
				edge_spline_count_ -= (GRuint)internal_graph_[*e_it].curve.size();
				removed_edges.push_back(*e_it);
			}

			boost::clear_vertex(vertex, internal_graph_);
			return removed_edges;
		}

		VertexProperties& get_vertex(VertexDescriptor vertex) {
			return internal_graph_[vertex];
		}

		const VertexProperties& get_vertex(VertexDescriptor vertex) const {
			return internal_graph_[vertex];
		}

		std::pair<VertexIterator, VertexIterator> vertices() {
			return boost::vertices(internal_graph_);
		}

		bool update_vertex_position(VertexDescriptor vertex, Vector3f new_position, bool maintain_shape_around_tip = true) {

			internal_graph_[vertex].position = new_position;

			std::pair<InEdgeIterator, InEdgeIterator> in_edges = boost::in_edges(vertex, internal_graph_);
			std::pair<OutEdgeIterator, OutEdgeIterator> out_edges = boost::out_edges(vertex, internal_graph_);


			for (InEdgeIterator e_it(in_edges.first); e_it != in_edges.second; e_it++) {
				if (!internal_graph_[*e_it].curve.pseudo_elastic_deform(false, new_position, maintain_shape_around_tip)) {
					return false;
				}
			}
			for (OutEdgeIterator e_it(out_edges.first); e_it != out_edges.second; e_it++) {
				if (!internal_graph_[*e_it].curve.pseudo_elastic_deform(true, new_position, maintain_shape_around_tip)) {
					return false;
				}
			}

			return true;
		}

		bool extrude_tip_vertex(VertexDescriptor vertex, Vector3f new_position, GRfloat min_spline_length) {
			//checking that this vertex is indeed a target and a tip (i.e. of in-degree 1 and out-degree 0)
			if (boost::in_degree(vertex, internal_graph_) != 1 || boost::out_degree(vertex, internal_graph_) != 0) {
				return false;
			}


			//update the vertex's position
			internal_graph_[vertex] = { new_position };

			std::pair<InEdgeIterator, InEdgeIterator> in_edges = boost::in_edges(vertex, internal_graph_);
			DeformableSplineCurve& edge_curve = get_edge(*(in_edges.first)).curve;
			
			//update the curve's back

			edge_curve.back() = PointTangent(new_position, (new_position - edge_curve[edge_curve.size() - 2].first).normalize());
			//and add a new point at the same place if it's far enough from the point before the back
			if (new_position.distance(edge_curve.before_back().first) >= min_spline_length) {
				edge_curve.add_middle_point(PointTangent(new_position, (new_position - edge_curve.before_back().first).normalize()));
				edge_spline_count_++;
				return true;
			}
			return false;
		}

		/** Returns the vertex that was removed, its surrounding removed edges and the new added edges
		\param option if it's the source or the target (i.e. vertex_one or vertex_two) that must be kept*/
		std::pair<VertexNeighborhood, EdgeVector> merge_vertices(VertexDescriptor vertex_one, VertexDescriptor vertex_two, COLLAPSE_OPTION option = SOURCE) {

			//add an edge from one vertex to the other
			std::pair<EdgeDescriptor, bool> new_edge_to_collapse = add_edge(vertex_one, vertex_two);

			if (!new_edge_to_collapse.second) {
				throw std::invalid_argument("Could not merge vertices");
			}				

			//and collapse it 
			std::pair<VertexNeighborhood, EdgeVector> removed_vertex_and_added_edges = collapse_edge(new_edge_to_collapse.first, option);
			VertexDescriptor cleared_vertex = removed_vertex_and_added_edges.first.first;
			remove_vertex(cleared_vertex);

			return removed_vertex_and_added_edges;

		}


		GRuint degree(VertexDescriptor vertex) {
			return (GRuint) (boost::in_degree(vertex, internal_graph_) + boost::out_degree(vertex, internal_graph_));
		}

		bool is_edge_source_or_target(EdgeDescriptor edge, VertexDescriptor vertex) const {
			return boost::source(edge, internal_graph_) == vertex || boost::target(edge, internal_graph_) == vertex;
		}

		/** Returns the vertex of edge that is not also a vertex of adjacent edge.
		If the edges are not adjacent or something else is wrong it returns a null_vertex*/
		VertexDescriptor find_vertex_not_connected_to_adjacent_edge(EdgeDescriptor edge, EdgeDescriptor adjacent_edge) {
			VertexDescriptor edge_source = boost::source(edge, internal_graph_);
			VertexDescriptor edge_target = boost::target(edge, internal_graph_);

			VertexDescriptor adjacent_edge_source = boost::source(adjacent_edge, internal_graph_);
			VertexDescriptor adjacent_edge_target = boost::target(adjacent_edge, internal_graph_);

			if (edge_source == adjacent_edge_source) {
				return edge_target;
			}
			else if (edge_source == adjacent_edge_target) {
				return edge_target;
			}
			else if (edge_target == adjacent_edge_source) {
				return edge_source;
			}
			else if (edge_target == adjacent_edge_target) {
				return edge_source;
			}
			else {
				return InternalBoostGraph::null_vertex();
			}

		}


		VertexVector shortest_path(VertexDescriptor from, VertexDescriptor to) {

			if (from == to) {
				return { from };
			}

			//reset
			VertexIterator vi, vi_end, next;
			boost::tie(vi, vi_end) = vertices();
			for (next = vi; vi != vi_end; vi = next) {
				++next;
				get_vertex(*vi).BFS_path_cost = (GRuint)-1;
				get_vertex(*vi).BFS_parent = nullptr;

			}

			//we start from the target vertex so that we can fill the path in the right order when back-tracking
			VertexDescriptor start_vertex = to;

			GRuint iteration_count(0);

			std::queue<VertexDescriptor> vertex_queue;
			vertex_queue.push(start_vertex);

			//get_vertex(start_vertex).is_in_spanning_tree = true;
			get_vertex(start_vertex).BFS_path_cost = 0;

			bool found_target = false;

			while (iteration_count < vertex_count() * 2 && !vertex_queue.empty()) {

				//std::cout << std::endl << " at iteration " << iteration_count << " queue is : " << std::endl;
				//print(vertex_queue);

				VertexDescriptor current_vertex = vertex_queue.front();
				vertex_queue.pop();

				GRuint current_cost = get_vertex(current_vertex).BFS_path_cost;

				//std::cout << "current vertex is " << get_vertex(current_vertex).position.to_string() << std::endl;

				std::pair<InEdgeIterator, InEdgeIterator> in_edges = boost::in_edges(current_vertex, internal_graph_);
				std::pair<OutEdgeIterator, OutEdgeIterator> out_edges = boost::out_edges(current_vertex, internal_graph_);
				for (InEdgeIterator e_it(in_edges.first); e_it != in_edges.second; e_it++) {
					VertexDescriptor source = boost::source(*e_it, internal_graph_);
					if (get_vertex(current_vertex).BFS_parent != source) {
						//std::cout << "checking vertex at " << get_vertex(source).position.to_string() << std::endl;
						if (get_vertex(source).BFS_parent == nullptr) {
							vertex_queue.push(source);
						}
						if (get_vertex(source).BFS_path_cost > current_cost + 1) {

							get_vertex(source).BFS_path_cost = current_cost + 1;
							get_vertex(source).BFS_parent = current_vertex;
							//	std::cout << "set parent of " << get_vertex(source).position.to_string() << " as " << get_vertex(current_vertex).position.to_string() << std::endl;

							if (source == from) {
								found_target = true;
							}
						}
					}
				}
				for (OutEdgeIterator e_it(out_edges.first); e_it != out_edges.second; e_it++) {
					VertexDescriptor target = boost::target(*e_it, internal_graph_);
					//std::cout << "checking vertex at " << get_vertex(target).position.to_string() << std::endl;
					if (get_vertex(current_vertex).BFS_parent != target) {
						if (get_vertex(target).BFS_parent == nullptr) {
							vertex_queue.push(target);
						}
						if (get_vertex(target).BFS_path_cost > current_cost + 1) {
							get_vertex(target).BFS_path_cost = current_cost + 1;
							get_vertex(target).BFS_parent = current_vertex;

							if (target == from) {
								found_target = true;
							}
						}
					}
				}
				iteration_count++;
			}

			if (!found_target) {
				throw std::invalid_argument("Couldn't find path between vertices");
			}

			VertexVector path = { from };
			VertexDescriptor next_parent = get_vertex(from).BFS_parent;


			while (next_parent != nullptr) {
				path.push_back(next_parent);
				next_parent = get_vertex(next_parent).BFS_parent;
			}


			//cleanup
			boost::tie(vi, vi_end) = vertices();
			for (next = vi; vi != vi_end; vi = next) {
				++next;
				get_vertex(*vi).BFS_path_cost = (GRuint)-1;
				get_vertex(*vi).BFS_parent = nullptr;

			}

			return path;
		}


		EdgeDescriptor join_vertices(VertexDescriptor source, VertexDescriptor target) {
			//TODO similar to join_edges but without the start and end edges and without the deformation

		}

		/****************************************************************************************************************************** Edge stuff*/

		std::pair<EdgeDescriptor, bool> add_edge(VertexDescriptor from, VertexDescriptor to, EdgeProperties properties) {

			if (from == InternalBoostGraph::null_vertex()
				|| to == InternalBoostGraph::null_vertex()) {
				return { EdgeDescriptor(), false };
			}
			edge_spline_count_ += (GRuint)properties.curve.size();

			std::pair<EdgeDescriptor, bool> new_edge = boost::add_edge(from, to, properties, internal_graph_);

			get_edge(new_edge.first).is_part_of_cycle = (get_vertex(from).is_part_of_cycle && get_vertex(to).is_part_of_cycle);

			return new_edge;
		}


		std::pair<EdgeDescriptor, bool> add_edge(VertexDescriptor from, VertexDescriptor to) {
			edge_spline_count_ += 2;
			Vector3f position_from = get_vertex(from).position;
			Vector3f position_to = get_vertex(to).position;

			EdgeProperties properties({
				DeformableSplineCurve(
					PointTangent(position_from, (position_to-position_from).normalize()),
					PointTangent(position_to, (position_to - position_from).normalize())
				)
			});

			return add_edge(from, to, properties);
		}


		VertexPair remove_edge(EdgeDescriptor edge) {

			VertexDescriptor source = boost::source(edge, internal_graph_);
			VertexDescriptor target = boost::target(edge, internal_graph_);

			VertexPair vertices(InternalBoostGraph::null_vertex(), InternalBoostGraph::null_vertex());

			if (boost::edge(source, target, internal_graph_).second) {

				edge_spline_count_ -= (GRuint)internal_graph_[edge].curve.size();


				bool remove_source = false;
				bool remove_target = false;

				if (degree(source) == 1) {
					remove_source = true;
				}
				if (degree(target) == 1) {
					remove_target = true;
				}

				boost::remove_edge(edge, internal_graph_);


				//we remove the vertices after the edge otherwise the edge would become invalid
				if (remove_source && vertex_count() != 1) {
					boost::remove_vertex(source, internal_graph_);
					vertices.first = source;
				}

				if (remove_target && vertex_count() != 1) {
					boost::remove_vertex(target, internal_graph_);
					vertices.second = target;
				}
			}

			return vertices;
		}

		const EdgeProperties& get_edge(EdgeDescriptor edge) const {
			return internal_graph_[edge];
		}

		EdgeProperties& get_edge(EdgeDescriptor edge) {
			return internal_graph_[edge];
		}

		std::pair<EdgeVector, bool> edge_exists(VertexDescriptor from, VertexDescriptor to, bool check_both_directions) {
			if (from == InternalBoostGraph::null_vertex() || to == InternalBoostGraph::null_vertex()) {
				return { {},false };
			}
			EdgeVector edges;
			bool found_edge = false;
			if (boost::edge(from, to, internal_graph_).second) {
				edges.push_back(boost::edge(from, to, internal_graph_).first);
				found_edge = true;
			}
			if (check_both_directions && boost::edge(to, from, internal_graph_).second) {
				edges.push_back(boost::edge(to, from, internal_graph_).first);
				found_edge = true;
			}

			return { edges, found_edge };
		}

		//TODO : maybe better
		GRfloat get_edge_radius(EdgeDescriptor edge, GRuint segment_index) const {

			GRfloat r1 = get_vertex(boost::source(edge, internal_graph_)).radius;
			GRfloat r2 = get_vertex(boost::target(edge, internal_graph_)).radius;
			GRfloat r_start = (2.f * r1*r2) / (r1 + r2);
			GRfloat r_end = r2;
			/*if (r1 < r2) {
				r_end = r_start;
				r_start = r2;
			}*/

			GRuint length = (GRuint)get_edge(edge).curve.size();
			segment_index = MIN(segment_index, length-1);

			return (1.f - (GRfloat)segment_index / ((GRfloat)length - 1))*(r_start - r_end) + r_end;
		}

		std::pair<EdgeIterator, EdgeIterator> edges() {
			return boost::edges(internal_graph_);
		}

		const VertexProperties& get_edge_source(EdgeDescriptor edge) const {
			return internal_graph_[boost::source(edge, internal_graph_)];
		}

		const VertexProperties& get_edge_target(EdgeDescriptor edge) const {
			return internal_graph_[boost::target(edge, internal_graph_)];
		}


		bool is_simple_edge(EdgeDescriptor edge) const {
			//the case where size < 2 shouldn't happen but better safe than sorry
			return internal_graph_[edge].curve.size() <= 2;
		}


		/** returns the list of vertices and edges that were removed and the list of vertices and edges that were added*/
		std::pair<std::pair<VertexVector, EdgeVector>, std::pair<VertexVector, EdgeVector>> split_edge_along_curve(
			EdgeDescriptor edge_to_split, 
			std::vector<std::pair<VertexDescriptor,VertexDescriptor>> new_edges_sources_and_targets) {

			VertexDescriptor edge_to_split_source = boost::source(edge_to_split, internal_graph_);
			VertexDescriptor edge_to_split_target = boost::target(edge_to_split, internal_graph_);
			if (!boost::edge(edge_to_split_source, edge_to_split_target, internal_graph_).second) {
				return { {{},{}},{{},{}} };
			}
			VertexVector new_vertices;
			EdgeVector new_edges;

			DeformableSplineCurve removed_edge_curve = get_edge(edge_to_split).curve;


			//std::cout << "removed edge curve : " << removed_edge_curve.to_string() << std::endl << std::endl;
			//std::cout << "removed edge  : " << edge_to_split << std::endl;

			std::vector<std::pair<VertexDescriptor, DeformableSplineCurve>> orphan_vertices_and_curves; //vertices adjacent to the split edge that are not among the list of vertices to connect
			
			GRuint iteration_count(0);

			EdgeVector edges_to_remove;

			for (auto source_target : new_edges_sources_and_targets) {
				VertexDescriptor new_source_vertex = source_target.first;
				VertexDescriptor new_target_vertex = source_target.second;
				DeformableSplineCurve new_curve_start;
				DeformableSplineCurve new_curve_middle;
				DeformableSplineCurve new_curve_end;

				//std::cout << std::endl << " iteration " << iteration_count << " : " << std::endl;
				//std::cout << "source : " << new_source_vertex << ", target : " << new_target_vertex << std::endl;

				bool reverse_middle = false;

				//first the in-edges of the source
				std::pair<InEdgeIterator, InEdgeIterator> in_ep;
				for (in_ep = boost::in_edges(edge_to_split_source, internal_graph_); in_ep.first != in_ep.second; ++in_ep.first) {

					EdgeDescriptor in_edge = *in_ep.first;
					VertexDescriptor in_edge_source = boost::source(in_edge, internal_graph_);
					if (in_edge_source == new_source_vertex) {
						//std::cout << "new source is source of in edge of source" << std::endl;
						new_curve_start =  get_edge(in_edge).curve;
						reverse_middle = false;
						edges_to_remove.push_back(in_edge);
					}
					else if (in_edge_source == new_target_vertex) {
						//std::cout << "new target is source of in edge of source" << std::endl;
						new_curve_end = DeformableSplineCurve(get_edge(in_edge).curve, true);
						//std::cout << "new curve end : " << new_curve_end.to_string() << std::endl;
						reverse_middle = true;
						edges_to_remove.push_back(in_edge);
					}
				}
				//and then its out-edges
				std::pair<OutEdgeIterator, OutEdgeIterator> out_ep;
				for (out_ep = boost::out_edges(edge_to_split_source, internal_graph_); out_ep.first != out_ep.second; ++out_ep.first) {

					EdgeDescriptor out_edge = *(out_ep.first);
					VertexDescriptor out_edge_target = boost::target(out_edge, internal_graph_);

					if (out_edge_target == new_source_vertex) {
						//std::cout << "new source is target of out edge of source" << std::endl;
						new_curve_start = DeformableSplineCurve(get_edge(out_edge).curve, true);
						//std::cout << "new curve start : " << new_curve_start.to_string() << std::endl;
						reverse_middle = false;
						edges_to_remove.push_back(out_edge);
					}
					else if (out_edge_target == new_target_vertex) {
						//std::cout << "new target is target of in edge of source" << std::endl;
						new_curve_end = get_edge(out_edge).curve;
						reverse_middle = true;
						edges_to_remove.push_back(out_edge);
					}
				}


				//then the in-edges of the target
			//	std::pair<InEdgeIterator, InEdgeIterator> in_ep;
				for (in_ep = boost::in_edges(edge_to_split_target, internal_graph_); in_ep.first != in_ep.second; ++in_ep.first) {

					EdgeDescriptor in_edge = *in_ep.first;
					VertexDescriptor in_edge_source = boost::source(in_edge, internal_graph_);

					if (in_edge_source == new_source_vertex) {
						//std::cout << "new source is source of in edge of target" << std::endl;
						new_curve_start = get_edge(in_edge).curve;
						//std::cout << "new curve start : " << new_curve_start.to_string() << std::endl;
						reverse_middle = true;
						edges_to_remove.push_back(in_edge);
					}
					else if (in_edge_source == new_target_vertex) {
						//std::cout << "new target is target of in edge of target" << std::endl;
						new_curve_end = DeformableSplineCurve(get_edge(in_edge).curve, true);
						//std::cout << "new curve end : " << new_curve_end.to_string() << std::endl;
						reverse_middle = false;
						edges_to_remove.push_back(in_edge);
					}
				}
				//and its out-edges
				//std::pair<OutEdgeIterator, OutEdgeIterator> out_ep;
				for (out_ep = boost::out_edges(edge_to_split_target, internal_graph_); out_ep.first != out_ep.second; ++out_ep.first) {

					EdgeDescriptor out_edge = *out_ep.first;
					VertexDescriptor out_edge_target = boost::target(out_edge, internal_graph_);

					if (out_edge_target == new_source_vertex) {
						//	std::cout << "new source is target of out edge of target" << std::endl;
						new_curve_start = DeformableSplineCurve(get_edge(out_edge).curve, true);
						//	std::cout << "new curve start : " << new_curve_start.to_string() << std::endl;
						reverse_middle = true;
						edges_to_remove.push_back(out_edge);
					}
					else if (out_edge_target == new_target_vertex) {
						//	std::cout << "new target is target of out edge of target" << std::endl;
						new_curve_end = get_edge(out_edge).curve;
						reverse_middle = false;
						edges_to_remove.push_back(out_edge);
					}
				}

				//remove the last segment the start curve
				new_curve_start.pop_back();
				//std::cout << " start curve : " << std::endl << new_curve_start.to_string() << std::endl;
				
				//prepare the middle curve by reversing it if necessary, deforming it and removing its back
				new_curve_middle = DeformableSplineCurve(removed_edge_curve, reverse_middle);
				new_curve_middle.pseudo_elastic_deform(true, new_curve_start.back().first);
				new_curve_middle.pseudo_elastic_deform(false, new_curve_end[1].first);

				new_curve_middle.pop_back();

				//append the middle curve
				new_curve_start.append(new_curve_middle, 1);
				//std::cout << " middle appended : " << std::endl << new_curve_start.to_string() << std::endl;

				//and append the end curve without its first element
				new_curve_start.append(new_curve_end, 1);
				//std::cout << " end appended : " << std::endl << new_curve_start.to_string() << std::endl;

				//and finally add the new edge tying the source and target vertex together
				EdgeDescriptor new_edge = add_edge(new_source_vertex, new_target_vertex, { new_curve_start}).first;

				new_edges.push_back(new_edge);

				iteration_count++;
				//std::cout << std::endl << std::endl;
			}

			VertexVector removed_vertices;
	
			//remove the split edge and the edges that have been handled

			edges_to_remove.push_back(edge_to_split);

			//	std::cout << " edges to remove : " << edges_to_remove.size() << std::endl;

			iteration_count = 0;
			for (auto edge : edges_to_remove) {
				//std::cout << " iteration : " << iteration_count << std::endl;
				VertexPair vertices = remove_edge(edge);
				//std::cout << "removed edge " << edge << std::endl;
				if (vertices.first != InternalBoostGraph::null_vertex()) {
					removed_vertices.push_back(vertices.first);
					//	std::cout << "removed vertex " << vertices.first << std::endl;
				}
				if (vertices.second != InternalBoostGraph::null_vertex()) {
					removed_vertices.push_back(vertices.second);
					//	std::cout << "removed vertex " << vertices.second << std::endl;
				}
			}
			//std::cout << " still goooood " << std::endl;

			return { {removed_vertices, edges_to_remove}, {new_vertices, new_edges} };
		}



		std::pair<VertexPair,EdgePair> cut_edge_at(EdgeDescriptor edge_to_cut, GRuint segment_index, Vector3f new_vertex_position) {
			DeformableSplineCurve original_curve = internal_graph_[edge_to_cut].curve;
			if (segment_index >= original_curve.size()-1) {
				throw std::invalid_argument("Cannot split edge at invalid segment index");
			}
			try {
				//first get the direction from the cut position to the previous and next segment
				Vector3f direction_to_previous_segment = (original_curve[segment_index].first - new_vertex_position).normalize();
				Vector3f direction_to_next_segment     = (original_curve[segment_index + 1].first - new_vertex_position).normalize();
				
				GRfloat displacement_factor = 2.f;

				Vector3f left_vertex_final_position = new_vertex_position +  direction_to_previous_segment * displacement_factor;
				Vector3f right_vertex_final_position = new_vertex_position + direction_to_next_segment * displacement_factor;

				//first split the edge
				std::pair<VertexDescriptor, EdgePair> right_vertex_neighborhood = split_edge_at(edge_to_cut, segment_index, right_vertex_final_position);
				VertexDescriptor right_vertex = right_vertex_neighborhood.first;
				EdgeDescriptor right_edge = right_vertex_neighborhood.second.second;
				//std::cout << "new right vertex : " << right_vertex << std::endl;
				//std::cout << "new right edge : " << right_edge << std::endl;

				//find the new edge incident to the new vertex
				EdgeDescriptor left_edge_temp = right_vertex_neighborhood.second.first;

				GRuint last_segment_index = (GRuint)internal_graph_[left_edge_temp].curve.size() - 2;
				
				//Vector3f last_segment_middle_position = (internal_graph_[left_edge_temp].curve.back().first + internal_graph_[left_edge_temp].curve.before_back().first)*0.5f;
				
				//split the left edge at the last segment
				std::pair<VertexDescriptor, EdgePair> left_vertex_neighborhood = split_edge_at(left_edge_temp, last_segment_index, left_vertex_final_position);
				VertexDescriptor left_vertex = left_vertex_neighborhood.first;
				EdgeDescriptor left_edge = left_vertex_neighborhood.second.first;

				//and remove the the middle edge
				EdgeDescriptor middle_edge = *(boost::in_edges(right_vertex, internal_graph_).first);
			
				remove_edge(middle_edge);


				return std::pair<VertexPair, EdgePair>( VertexPair(left_vertex, right_vertex), EdgePair(left_edge, right_edge));
			}
			catch (std::invalid_argument e) {
				throw e;
			}
		}



		/** Returns the descriptor of the vertex that is now in the middle of the split edge*/
		std::pair<VertexDescriptor, EdgePair> split_edge_at(EdgeDescriptor edge_to_split, GRuint segment_index, Vector3f new_vertex_position) {

			SplineCurve edge_curve(get_edge(edge_to_split).curve);

			if (segment_index >= edge_curve.size() - 1) {
				throw std::invalid_argument("Cannot split edge at invalid segment index");
			}

			VertexDescriptor new_vertex = add_vertex({ new_vertex_position, get_edge_radius(edge_to_split, segment_index) });

			//std::cout << "new middle vertex is : " << new_vertex << std::endl;

			//compute the new end and start points
			PointTangent source_pt = edge_curve.front();
			PointTangent target_pt = edge_curve.back();

			PointTangent new_target_pt = PointTangent(new_vertex_position, (new_vertex_position - edge_curve[segment_index].first).normalize());
			PointTangent new_source_pt = PointTangent(new_vertex_position, (edge_curve[segment_index + 1].first - new_vertex_position).normalize());

			//create the base of the new curves (with just the start and end positions)
			SplineCurve first_half(source_pt, new_target_pt);
			SplineCurve second_half(new_source_pt, target_pt);


			//and fill those with the old PointTangents
			for (GRuint i(1); i <= segment_index; i++) {
				first_half.add_middle_point(edge_curve[i]);
			}
			if (first_half.size() >= 3) {
				first_half[first_half.size() - 2].second = (first_half.back().first - first_half[first_half.size() - 3].first).normalize();
			}

			for (GRuint i(segment_index + 1); i < edge_curve.size() - 1; i++) {
				second_half.add_middle_point(edge_curve[i]);
			}
			if (second_half.size() >= 3) {
				second_half[1].second = (second_half[2].first - second_half[0].first).normalize();
			}


			//add the new edges
			EdgeDescriptor left_edge = add_edge(boost::source(edge_to_split, internal_graph_), new_vertex, { first_half }).first;
			EdgeDescriptor right_edge = add_edge(new_vertex, boost::target(edge_to_split, internal_graph_), { second_half }).first;

			/*std::cout << "new left edge : " << left_edge << std::endl;
			std::cout << "new right edge : " << right_edge << std::endl;
			std::cout << "------" << std::endl;*/

			get_edge(left_edge).is_part_of_cycle = get_edge(edge_to_split).is_part_of_cycle;
			get_edge(right_edge).is_part_of_cycle = get_edge(edge_to_split).is_part_of_cycle;

			//and remove the old one
			remove_edge(edge_to_split);

			return std::pair<VertexDescriptor, EdgePair>(new_vertex, EdgePair( left_edge, right_edge ));
		}


		/** returns the vertexdescriptor of the vertex that was removed, its edges (also removed) and the edges that were added */
		std::pair<VertexNeighborhood, EdgeVector> collapse_edge(EdgeDescriptor edge_to_collapse, COLLAPSE_OPTION option = SOURCE) {

			//first attach the edge_to_collapse's source's edges to the edge_to_collapse's target
			VertexDescriptor source = boost::source(edge_to_collapse, internal_graph_);
			VertexDescriptor target = boost::target(edge_to_collapse, internal_graph_);


			//check if edge exists
			if (source == InternalBoostGraph::null_vertex() 
				|| target == InternalBoostGraph::null_vertex()
				|| !boost::edge(source, target, internal_graph_).second) {
				throw std::invalid_argument("edge doesn't exist");
			}

			//by default we keep the source
			VertexDescriptor to_keep = source;
			VertexDescriptor to_remove = target;
			Vector3f new_position = internal_graph_[to_keep].position;

			if (option == TARGET) {
				to_keep = target;
				to_remove = source;
				new_position = internal_graph_[to_keep].position;
			}
			else if (option == MIDPOINT) {
				new_position = (internal_graph_[source].position + internal_graph_[target].position)*0.5f;
			}

			/** gather the list of edges to add */
			std::vector<VertexDescriptor> sources_to_add;
			std::vector<VertexDescriptor> targets_to_add;
			std::vector<EdgeProperties> source_props_to_add;
			std::vector<EdgeProperties> target_props_to_add;

			//first the in-edges
			std::pair<InEdgeIterator, InEdgeIterator> in_ep;
			for (in_ep = boost::in_edges(to_remove, internal_graph_); in_ep.first != in_ep.second; ++in_ep.first) {
				if (*in_ep.first != edge_to_collapse) {
					VertexDescriptor new_source = boost::source(*in_ep.first, internal_graph_);
					if (new_source != to_keep) {

						EdgeProperties new_props = internal_graph_[*in_ep.first];
						new_props.curve.back() = PointTangent(new_position, (new_position - new_props.curve[new_props.curve.size() - 2].first).normalize());
						sources_to_add.push_back(new_source);
						source_props_to_add.push_back(new_props);
					}
				}
			}

			//and then the out-edges
			std::pair<OutEdgeIterator, OutEdgeIterator> out_ep;
			for (out_ep = boost::out_edges(to_remove, internal_graph_); out_ep.first != out_ep.second; ++out_ep.first) {
				if (*out_ep.first != edge_to_collapse) {
					VertexDescriptor new_target = boost::target(*out_ep.first, internal_graph_);
					if (new_target != to_keep) {

						EdgeProperties new_props = internal_graph_[*out_ep.first];
						new_props.curve.front() = PointTangent(new_position, (new_props.curve[0].first - new_position).normalize());
						targets_to_add.push_back(new_target);
						target_props_to_add.push_back(new_props);
					}
				}
			}

			//remove the edges of the vertex to remove
			VertexNeighborhood removed_neighborhood = { to_remove, clear_vertex(to_remove) };

			EdgeVector new_edges;

			//add the new edges
			for (GRuint i(0); i < sources_to_add.size(); i++) {
				new_edges.push_back(add_edge(sources_to_add[i], to_keep, source_props_to_add[i]).first);
			}

			for (GRuint i(0); i < targets_to_add.size(); i++) {
				new_edges.push_back(add_edge(to_keep, targets_to_add[i], target_props_to_add[i]).first);
			}

			//and update the vertex to keep's position
			internal_graph_[to_keep].position = new_position;

			return {removed_neighborhood, new_edges};
		}


		//concatenates all the curves from the edges joined by the succession of vertices (path)
		DeformableSplineCurve convert_to_curve(VertexVector path) {
			DeformableSplineCurve new_curve;
			new_curve.clear();

			if (path.size() > 1) {
				EdgeDescriptor first_edge;

				if (boost::edge(path[0], path[1], internal_graph_).second) {
					first_edge = boost::edge(path[0], path[1], internal_graph_).first;
				}else if(boost::edge(path[1], path[0], internal_graph_).second) {
					first_edge = boost::edge(path[1], path[0], internal_graph_).first;
				}
				else {
					return new_curve;
				}

				new_curve = DeformableSplineCurve(get_edge(first_edge).curve, path[0] == boost::target(first_edge, internal_graph_));

				//std::cout << "curve start : " << new_curve.to_string() << std::endl;

				for (size_t i(1); i < path.size() - 1; i++) {
					EdgeDescriptor next_edge;

					if (boost::edge(path[i], path[i+1], internal_graph_).second) {
						next_edge = boost::edge(path[i], path[i+1], internal_graph_).first;
					}
					else if (boost::edge(path[i+1], path[i], internal_graph_).second) {
						next_edge = boost::edge(path[i+1], path[i], internal_graph_).first;
					}
					else {
						return new_curve;
					}
					
					new_curve.append(get_edge(next_edge).curve, 1, path[i] == boost::target(next_edge, internal_graph_));
				}
			}

			return new_curve;
		}


		VertexVector shortest_path(EdgeDescriptor source_edge, EdgeDescriptor target_edge) {
			VertexDescriptor source_edge_source = boost::source(source_edge, internal_graph_);
			VertexDescriptor source_edge_target = boost::target(source_edge, internal_graph_);

			VertexDescriptor target_edge_source = boost::source(target_edge, internal_graph_);
			VertexDescriptor target_edge_target = boost::target(target_edge, internal_graph_);

			try {
				std::vector<VertexVector> paths = {
					shortest_path(source_edge_source, target_edge_source),
					shortest_path(source_edge_source, target_edge_target),
					shortest_path(source_edge_target, target_edge_source),
					shortest_path(source_edge_target, target_edge_target)
				};
				
				size_t min_size_index = 0;
				size_t min_size = (size_t)-1;
				for (GRuint i(0); i < paths.size(); i++) {

					//std::cout << "path length : " << paths[i].size() << std::endl;
					if (paths[i].size() < min_size) {
						min_size = paths[i].size();
						min_size_index = i;
					}
				}
				
				return paths[min_size_index];

			}
			catch (std::invalid_argument e) {
				throw e;
			}

		}

		std::pair<EdgePair, EdgeDescriptor> join_edges(EdgeDescriptor source_edge, EdgeDescriptor target_edge, GRfloat new_edge_displacement = 1.f) {

			if (source_edge == target_edge) {
				throw std::invalid_argument("Cannot join an edge to itself");
			}

			VertexDescriptor source_edge_source = boost::source(source_edge, internal_graph_);
			VertexDescriptor source_edge_target = boost::target(source_edge, internal_graph_);

			VertexDescriptor target_edge_source = boost::source(target_edge, internal_graph_);
			VertexDescriptor target_edge_target = boost::target(target_edge, internal_graph_);

			try {
				VertexVector source_to_source_path = shortest_path(source_edge_source, target_edge_source);
				VertexVector source_to_target_path = shortest_path(source_edge_source, target_edge_target);
				VertexVector target_to_source_path = shortest_path(source_edge_target, target_edge_source);
				VertexVector target_to_target_path = shortest_path(source_edge_target, target_edge_target);


				DeformableSplineCurve new_edge_curve_start;
				DeformableSplineCurve new_edge_curve_middle;
				DeformableSplineCurve new_edge_curve_end;

				VertexDescriptor new_source;
				VertexDescriptor new_target;

				std::vector<size_t> sizes = { source_to_source_path.size(),source_to_target_path.size(),  target_to_source_path.size(), target_to_target_path.size()};
				size_t min_size_index = 0;
				size_t min_size = (size_t)-1;
				for (GRuint i(0); i < sizes.size(); i++) {

					std::cout << "path length : " << sizes[i] << std::endl;
					if (sizes[i] < min_size) {
						min_size = sizes[i];
						min_size_index = i;
					}
				}

				switch (min_size_index) {
				default: 
				case(0): {
					new_edge_curve_start  = DeformableSplineCurve(get_edge(source_edge).curve, true);
					new_edge_curve_middle = convert_to_curve(source_to_source_path);
					new_edge_curve_end    = DeformableSplineCurve(get_edge(target_edge).curve, false);
					new_source = source_edge_target;
					new_target = target_edge_target;
					break;
				}
				case(1): {
					new_edge_curve_start = DeformableSplineCurve(get_edge(source_edge).curve, true);
					new_edge_curve_middle = convert_to_curve(source_to_target_path);
					new_edge_curve_end = DeformableSplineCurve(get_edge(target_edge).curve, true);
					new_source = source_edge_target;
					new_target = target_edge_source;
					break;
				}
				case(2): {
					new_edge_curve_start = DeformableSplineCurve(get_edge(source_edge).curve, false);
					new_edge_curve_middle = convert_to_curve(target_to_source_path);
					new_edge_curve_end = DeformableSplineCurve(get_edge(target_edge).curve, false);
					new_source = source_edge_source;
					new_target = target_edge_target;
					break;
				}
				case(3): {
					new_edge_curve_start = DeformableSplineCurve(get_edge(source_edge).curve, false);
					new_edge_curve_middle = convert_to_curve(target_to_target_path);
					new_edge_curve_end = DeformableSplineCurve(get_edge(target_edge).curve, true);
					new_source = source_edge_source;
					new_target = target_edge_source;
					break;
				}
				}

				Vector3f first_junction_point = new_edge_curve_start.back().first + (new_edge_curve_start.before_back().first - new_edge_curve_start.back().first).normalize()*new_edge_displacement;
				Vector3f second_junction_point = new_edge_curve_end.front().first + (new_edge_curve_end.after_front().first - new_edge_curve_end.front().first).normalize()*new_edge_displacement;

				new_edge_curve_start.pseudo_elastic_deform(false, first_junction_point);
			
				if (new_edge_curve_middle.size()) {
					new_edge_curve_middle.pseudo_elastic_deform(true, first_junction_point);
					new_edge_curve_middle.pseudo_elastic_deform(false, second_junction_point);
					new_edge_curve_start.append(new_edge_curve_middle, 1);
				}

				new_edge_curve_end.pseudo_elastic_deform(true, second_junction_point);

				new_edge_curve_start.append(new_edge_curve_end, 1);

				//add the new edge
				EdgeDescriptor new_edge = add_edge(new_source, new_target, { new_edge_curve_start }).first;

				//remove the old edges
				remove_edge(source_edge);
				remove_edge(target_edge);

				return {{source_edge, target_edge }, new_edge };
			}
			catch (std::invalid_argument e) {
				throw e;
			}

		}



		/************************************************************************************* General operations*/

		bool export_to_file(std::string filename, GRfloat scale = 1.0f) {
			std::ofstream output_file(filename.c_str());

			std::map<VertexDescriptor, GRuint> vertex_index_map;


			if (output_file.is_open()) {

				output_file << "<scale>" << scale << "</scale>" << std::endl;

				GRuint iteration_count(0);
				std::pair<VertexIterator, VertexIterator> vp;

				output_file << "<vertices>\n";

				for (vp = boost::vertices(internal_graph_); vp.first != vp.second; ++vp.first) {
					VertexDescriptor v = *vp.first;
					VertexProperties props = internal_graph_[v];
					output_file << "<vertex>\n";
					output_file << "<pos>" << props.position.to_compact_string() << "</pos>\n";
					output_file << "<radius>" << props.radius << "</radius>\n";
					output_file << "<cycle>" << props.is_part_of_cycle << "</cycle>\n";
					output_file << "</vertex>\n";

					vertex_index_map.insert({v, iteration_count});
					iteration_count++;
				}
				output_file << "</vertices>\n";


				iteration_count = 0;

				output_file << "<edges>\n";
				std::pair<EdgeIterator, EdgeIterator> ep ;
				for (ep = boost::edges(internal_graph_); ep.first != ep.second; ++ep.first) {
					output_file << "<edge>\n";

					EdgeDescriptor e = *ep.first;
					EdgeProperties props = internal_graph_[e];
					std::map<VertexDescriptor, GRuint>::iterator index_it = vertex_index_map.find(boost::source(e, internal_graph_));
					if (index_it == vertex_index_map.end()) {
						std::cerr << "ERROR : invalid edge found when trying to export graph" << std::endl;
						output_file.close();
						return false;
					}
					output_file << "<source>" << index_it->second <<"</source>\n";

					index_it = vertex_index_map.find(boost::target(e, internal_graph_));
					if (index_it == vertex_index_map.end()) {
						std::cerr << "ERROR : invalid edge found when trying to export graph" << std::endl;
						output_file.close();
						return false;
					}
					output_file << "<target>" << index_it->second << "</target>\n";

					output_file << "<cycle>" << props.is_part_of_cycle << "</cycle>\n";

					output_file << "<curve>\n";
					output_file << props.curve.to_compact_string();

					output_file << "</curve>\n";

					output_file << "</edge>\n";
				}
				output_file << "</edges>\n";



				output_file.close();
				return true;
			}

			return false;
		}


		/** NOTE : this allocates a new Graph. It should be deleted when done with it*/
		static void import_from_file(std::string filename, SkeletalGraph* graph, GRfloat& scale) {
			scale = 1.0f;

			if (graph == nullptr) {
				throw std::runtime_error("ERROR - Cannot set nullptr SkeletalGraph");
			}

			std::ifstream input_file(filename);
			if (! input_file.is_open()) {
				throw std::runtime_error("ERROR - Could not open specified file. No SkeletalGraph was imported");
			}

			VertexVector vertices;

			std::string line;
			bool reading_vertices(false);
			bool reading_edges(false);
			bool reading_vertex(false);
			bool reading_edge(false);
			bool reading_curve(false);

			VertexProperties v_props;
			EdgeProperties e_props;
			GRuint e_source_index = 0;
			GRuint e_target_index = 0;
			DiscreteCurve e_curve;


			while (std::getline(input_file, line)) {
				if (line.substr(0, 7) == "<scale>") {
					sscanf_s(line.c_str(), "<scale>%f</scale>", &scale);
				}
				else if (line == "<vertices>") {
					reading_vertices = true;
				}
				else if (line == "</vertices>") {
					reading_vertices = false;
				}
				else if (reading_vertices) {

					if (line == "<vertex>") {
						reading_vertex = true;
						v_props = VertexProperties();
					}
					else if (line == "</vertex>") {
						reading_vertex = false;
						vertices.push_back(graph->add_vertex(v_props));
					}
					else if (reading_vertex) {
						if (line.substr(0, 5) == "<pos>") {
							GRfloat x(0.f), y(0.f), z(0.f);
							if (sscanf_s(line.c_str(), "<pos>%f %f %f</pos>", &x, &y, &z) == 3) {
								v_props.position = Vector3f(x, y, z);
							}
							else {
								std::cerr << "could not read position from line : " << line << std::endl;
								v_props.position = Vector3f(0.f);
							}

						}else if (line.substr(0, 8) == "<radius>") {
							GRfloat radius = DEFAULT_VERTEX_RADIUS;
							if (sscanf_s(line.c_str(), "<radius>%f</radius>", &radius) == 1) {
								v_props.radius = radius;
							}
							else {
								std::cerr << "could not read radius from line : " << line << std::endl;
								v_props.radius = DEFAULT_VERTEX_RADIUS;
							}

						}
						else if (line.substr(0, 7) == "<cycle>") {
							GRuint cycle = false;
							if (sscanf_s(line.c_str(), "<cycle>%u</cycle>", &cycle) == 1) {
								v_props.is_part_of_cycle = cycle;
							}
							else {
								std::cerr << "could not read if in cycle from line : " << line << std::endl;
								v_props.is_part_of_cycle = false;
							}
						}
					}

				}
				else if (line == "<edges>") {
					reading_edges = true;
					if (!vertices.size()) {
						break;
					}
				}
				else if (line == "</edges>") {
					reading_edges = false;
				}
				else if (reading_edges) {
					if (line == "<edge>") {
						reading_edge = true;
						e_props = EdgeProperties();
						e_source_index = 0;
						e_target_index = 0;
						e_curve = DiscreteCurve();
					}
					else if (line == "</edge>") {
						reading_edge = false;
						if (e_source_index < vertices.size() && e_target_index < vertices.size()) {
							graph->add_edge(vertices[e_source_index], vertices[e_target_index], e_props);
						}
						else {
							std::cerr << " ERROR - could not add edge with invalid vertex indices : " << e_source_index << ", " << e_target_index << std::endl;
						}
					}
					else if (reading_edge) {
						if (line == "<curve>") {
							reading_curve = true;
						}
						else if (line == "</curve>") {
							reading_curve = false;
							try {
								e_props.curve = DeformableSplineCurve(e_curve);
							}
							catch (std::runtime_error e) {
								std::cerr << e.what() << std::endl;
							}
						}
						else if (reading_curve) {
							GRfloat x(0.f), y(0.f), z(0.f);
							if (sscanf_s(line.c_str(), "%f %f %f", &x, &y, &z) == 3) {
								e_curve.push_back(Vector3f(x, y, z));
							}
							else {
								std::cerr << "could not curve point from line : " << line << std::endl;
							}
						}
						else if (line.substr(0, 8) == "<source>") {							
							if (sscanf_s(line.c_str(), "<source>%u</source>", &e_source_index) != 1) {
								std::cerr << "could not read source from line : " << line << std::endl;
							}
						}
						else if (line.substr(0, 8) == "<target>") {
							if (sscanf_s(line.c_str(), "<target>%u</target>", &e_target_index) != 1) {
								std::cerr << "could not read target from line : " << line << std::endl;
								std::cerr << " read : " << sscanf_s(line.c_str(), "<target>%u</target>", &e_target_index) << std::endl;
							}
						}
						else if (line.substr(0, 7) == "<cycle>") {
							GRuint cycle = 0;
							if (sscanf_s(line.c_str(), "<cycle>%u</cycle>", &cycle) == 1) {
								e_props.is_part_of_cycle = cycle;
							}else{
								std::cerr << "could not read cycle from line : " << line << std::endl;
							}
						}
						
					}
				}
			}
		}


		void find_cycle_in_spanning_tree(VertexDescriptor vertex_one, VertexDescriptor vertex_two) {

			//std::cout << " vertex one at : " << get_vertex(vertex_one).position.to_string() << std::endl;
			//std::cout << " vertex two at : " << get_vertex(vertex_two).position.to_string() << std::endl;

			
			//first get the parent->child list from root to both vertices
			std::list<VertexDescriptor> path_one;
			path_one.push_back(vertex_one);

			std::list<VertexDescriptor> path_two;
			path_two.push_back(vertex_two);

			const GRuint iteration_count_limit(vertex_count() + 1);

			VertexDescriptor current_vertex = vertex_one;
			VertexDescriptor parent_vertex = get_vertex(vertex_one).cycle_parent;
			GRuint iteration_count(0);
			while (iteration_count < iteration_count_limit 
				&& parent_vertex != nullptr) {

				path_one.push_front(parent_vertex);
				current_vertex = parent_vertex;
				parent_vertex = get_vertex(current_vertex).cycle_parent;
				iteration_count++;
			}
			VertexDescriptor root_one = current_vertex;

			/*std::cout << " vertex one path is " << std::endl;
			for (auto vertex : path_one) {
				std::cout << get_vertex(vertex).position.to_string() << " ";
			}
			std::cout << std::endl;
			*/

			current_vertex = vertex_two;
			parent_vertex = get_vertex(vertex_two).cycle_parent;
			iteration_count = 0;

			while (iteration_count < iteration_count_limit 
				&& parent_vertex != nullptr) {

				path_two.push_front(parent_vertex);
				current_vertex = parent_vertex;
				parent_vertex = get_vertex(current_vertex).cycle_parent;
				iteration_count++;
			}
			VertexDescriptor root_two = current_vertex;

			/*std::cout << " vertex two path is " << std::endl;
			for (auto vertex : path_two) {
				std::cout << get_vertex(vertex).position.to_string() << " ";
			}
			std::cout << std::endl;
			*/
			
			if (path_one.front() != path_two.front()) {
				std::cerr << "ERROR - both paths don't have the same root !!" << std::endl;

			}

			VertexDescriptor bifurcation = path_one.front();
			//trim the start of both paths until the first bifurcation
			while (path_one.size()
				&& path_two.size()
				&& path_one.front() == path_two.front()) {

				bifurcation = path_one.front();
				path_one.pop_front();
				path_two.pop_front();
			}
			//std::cout << "bifurcation is at " << get_vertex(bifurcation).position.to_string() << std::endl;

			//set the bifurcation as part of cycle
			get_vertex(bifurcation).is_part_of_cycle = true;


			/*std::cout << " vertex one path is now " << std::endl;
			for (auto vertex : path_one) {
				std::cout << get_vertex(vertex).position.to_string() << " ";
			}
			std::cout << std::endl;

			std::cout << " vertex two path is now " << std::endl;
			for (auto vertex : path_two) {
				std::cout << get_vertex(vertex).position.to_string() << " ";
			}
			std::cout << std::endl;
			*/

			iteration_count = 0;

			//same for all the first path' vertices and edges
			VertexDescriptor last_vertex = bifurcation;
			for (auto next_vertex : path_one) {
			//	std::cout << "iteration : " << iteration_count << std::endl;
				get_vertex(next_vertex).is_part_of_cycle = true;
				std::pair<EdgeDescriptor, bool> in_edge = boost::edge( next_vertex, last_vertex,internal_graph_);
				std::pair<EdgeDescriptor, bool> out_edge = boost::edge(last_vertex, next_vertex, internal_graph_);
				if (in_edge.second) {
					get_edge(in_edge.first).is_part_of_cycle = true;
				}
				else if (out_edge.second) {
					get_edge(out_edge.first).is_part_of_cycle = true;

				}else{
					std::cerr << "ERROR - edge from " << get_vertex(next_vertex).position.to_string() << " to " << get_vertex(last_vertex).position.to_string() << " has somehow disappeared" << std::endl;
					exit(EXIT_FAILURE);
				}
				iteration_count++;
				last_vertex = next_vertex;
			}

			//std::cout << "done with path one" << std::endl;


			iteration_count = 0;
			//and the second path's vertices and edges
			last_vertex = bifurcation;
			for (auto next_vertex : path_two) {
				//std::cout << "iteration : " << iteration_count << std::endl;
				get_vertex(next_vertex).is_part_of_cycle = true;
				std::pair<EdgeDescriptor, bool> in_edge = boost::edge(next_vertex, last_vertex, internal_graph_);
				std::pair<EdgeDescriptor, bool> out_edge = boost::edge(last_vertex, next_vertex, internal_graph_);
				if (in_edge.second) {
					get_edge(in_edge.first).is_part_of_cycle = true;
				}
				else if (out_edge.second) {
					get_edge(out_edge.first).is_part_of_cycle = true;
				}
				else {
					std::cerr << "ERROR - edge from " << get_vertex(next_vertex).position.to_string() << " to " << get_vertex(last_vertex).position.to_string() << " has somehow disappeared" << std::endl;
					exit(EXIT_FAILURE);
				}
				iteration_count++;
				last_vertex = next_vertex;
			}

			//and finally all edges connecting the two vertices
			std::pair<InEdgeIterator, InEdgeIterator> in_edges = boost::in_edges(vertex_one, internal_graph_);
			std::pair<OutEdgeIterator, OutEdgeIterator> out_edges = boost::out_edges(vertex_one, internal_graph_);
			for (InEdgeIterator e_it(in_edges.first); e_it != in_edges.second; e_it++) {
				VertexDescriptor source = boost::source(*e_it, internal_graph_);
				if (source == vertex_two) {
					get_edge(*e_it).is_part_of_cycle = true;
				}
			}
			for (OutEdgeIterator e_it(out_edges.first); e_it != out_edges.second; e_it++) {
				VertexDescriptor target = boost::target(*e_it, internal_graph_);
				if (target == vertex_two){
					get_edge(*e_it).is_part_of_cycle = true;
				}
			}

			//std::cout << std::endl << std::endl;
		}




		void print_cycles() {
			VertexIterator vi, vi_end, next;
			boost::tie(vi, vi_end) = vertices();
			for (next = vi; vi != vi_end; vi = next) {
				++next;
				if (get_vertex(*vi).is_part_of_cycle) {
					std::cout << get_vertex(*vi).position.to_string() << std::endl;
				}
			}

			std::pair<EdgeIterator, EdgeIterator> edges = boost::edges(internal_graph_);
			for (EdgeIterator e_it(edges.first); e_it != edges.second; e_it++) {
				EdgeDescriptor e = *e_it;
				if (get_edge(*e_it).is_part_of_cycle) {

					std::cout << " : |"
						<< internal_graph_[boost::source(e, internal_graph_)].position.to_string() << "| -> "
						<< std::endl << internal_graph_[e].curve.to_string() << std::endl
						<< " -> |" << internal_graph_[boost::target(e, internal_graph_)].position.to_string() << "| " << std::endl << std::endl;
				}

			}

		}




		void find_cycles() {

			//reset
			VertexIterator vi, vi_end, next;
			boost::tie(vi, vi_end) = vertices();
			for (next = vi; vi != vi_end; vi = next) {
				++next;
				get_vertex(*vi).is_part_of_cycle = false;
			}

			EdgeIterator ei, ei_end, e_next;
			boost::tie(ei, ei_end) = edges();
			for (e_next = ei; ei != ei_end; ei = e_next) {
				++e_next;
				get_edge(*ei).is_part_of_cycle = false;
			}

			VertexDescriptor start_vertex = *boost::vertices(internal_graph_).first;

			GRuint iteration_count(0);

			std::queue<VertexDescriptor> vertex_queue;
			vertex_queue.push(start_vertex);

			get_vertex(start_vertex).is_in_spanning_tree = true;

			//std::set<VertexDescriptor> spanning_tree;
			//spanning_tree.insert(start_vertex);

			std::vector<std::vector<VertexDescriptor>> cycles;

			while (iteration_count < vertex_count() * 2 && !vertex_queue.empty()) {

				//std::cout << std::endl << " at iteration " << iteration_count << " queue is : " << std::endl;
				//print(vertex_queue);

				VertexDescriptor current_vertex = vertex_queue.front();
				vertex_queue.pop();

				//std::cout << "current vertex is " << get_vertex(current_vertex).position.to_string() << std::endl;

				std::pair<InEdgeIterator, InEdgeIterator> in_edges = boost::in_edges(current_vertex, internal_graph_);
				std::pair<OutEdgeIterator, OutEdgeIterator> out_edges = boost::out_edges(current_vertex, internal_graph_);
				for (InEdgeIterator e_it(in_edges.first); e_it != in_edges.second; e_it++) {
					VertexDescriptor source = boost::source(*e_it, internal_graph_);
					if (get_vertex(current_vertex).cycle_parent != source) {
						//std::cout << "checking vertex at " << get_vertex(source).position.to_string() << std::endl;
						if (get_vertex(source).is_in_spanning_tree) {
						//	std::cout << "   found cycle at " << get_vertex(source).position.to_string()<<std::endl;
							find_cycle_in_spanning_tree(current_vertex, source);

						}
						else {
							vertex_queue.push(source);
							get_vertex(source).is_in_spanning_tree = true;
							get_vertex(source).cycle_parent = current_vertex;
						//	std::cout << "set parent of " << get_vertex(source).position.to_string() << " as " << get_vertex(current_vertex).position.to_string() << std::endl;
						}
					}
				}
				for (OutEdgeIterator e_it(out_edges.first); e_it != out_edges.second; e_it++) {
					VertexDescriptor target = boost::target(*e_it, internal_graph_);
					//std::cout << "checking vertex at " << get_vertex(target).position.to_string() << std::endl;
					if (get_vertex(current_vertex).cycle_parent != target) {
						if (get_vertex(target).is_in_spanning_tree) {
					//		std::cout << "   found cycle at " << get_vertex(target).position.to_string() << std::endl;
							find_cycle_in_spanning_tree(current_vertex, target);

						}
						else {
							vertex_queue.push(target);
							get_vertex(target).is_in_spanning_tree = true;
							get_vertex(target).cycle_parent = current_vertex;
					//		std::cout << "set parent of " << get_vertex(target).position.to_string() << " as " << get_vertex(current_vertex).position.to_string() << std::endl;
						}
					}
				}

				iteration_count++;
			}

			//cleanup
		//	VertexIterator vi, vi_end, next;
			boost::tie(vi, vi_end) = vertices();
			for (next = vi; vi != vi_end; vi = next) {
				++next;
				get_vertex(*vi).cycle_parent = nullptr;
				get_vertex(*vi).is_in_spanning_tree = false;
			}
		}


		GRuint collapse_edges_shorter_than(GRfloat min_length) {
			std::vector<EdgeDescriptor> edges_to_collapse;
			std::pair<EdgeIterator, EdgeIterator> e_it;

			for (e_it = boost::edges(internal_graph_); e_it.first != e_it.second; ++e_it.first) {

				GRuint source_degree
					= degree(boost::source(*e_it.first, internal_graph_));

				GRuint target_degree
					= degree(boost::target(*e_it.first, internal_graph_));

				if (get_edge(*e_it.first).curve.length() < min_length && source_degree != 1 && target_degree != 1) {
					edges_to_collapse.push_back(*e_it.first);
				}
			}


			//collapse the edges at midpoints and gather the vertices to remove
			std::vector<VertexDescriptor> vertices_to_remove;
			//std::cout << "found " << edges_to_collapse.size() << " edges to collapse " << std::endl;
			for (auto edge : edges_to_collapse) {
				try {
					VertexDescriptor vertex = collapse_edge(edge, MIDPOINT).first.first;
					if (vertex != InternalBoostGraph::null_vertex()) {
						vertices_to_remove.push_back(vertex);
					}
				}
				catch (std::invalid_argument e) {
					std::cerr << e.what() << std::endl;
				}
			}

			//and remove the now-alone vertices
			for (auto vertex : vertices_to_remove) {
				remove_vertex(vertex);
			}

			return (GRuint)vertices_to_remove.size();
		}


		void collapse_edges_with_less_than_n_splines(GRuint n) {
			std::vector<EdgeDescriptor> edges_to_collapse;
			std::pair<EdgeIterator, EdgeIterator> e_it;
			for (e_it = boost::edges(internal_graph_); e_it.first != e_it.second; ++e_it.first) {

				GRuint source_degree
					= degree(boost::source(*e_it.first, internal_graph_));

				GRuint target_degree
					= degree(boost::target(*e_it.first, internal_graph_));

				if (get_edge(*e_it.first).curve.size() < n && source_degree != 1 && target_degree != 1) {
					edges_to_collapse.push_back(*e_it.first);
				}
			}

			//collapse the edges at midpoints and gather the vertices to remove
			std::vector<VertexDescriptor> vertices_to_remove;
			//std::cout << "found " << edges_to_collapse.size() << " edges to collapse " << std::endl;
			for (auto edge : edges_to_collapse) {
				VertexDescriptor vertex = collapse_edge(edge, MIDPOINT).first.first;

				if (vertex != InternalBoostGraph::null_vertex()) {
					vertices_to_remove.push_back(vertex);
					//std::cout << " will remove vertex at " << internal_graph_[vertex].position.to_string() << std::endl;
				}
			}

			//and remove the now-alone vertices
			for (auto vertex : vertices_to_remove) {
				remove_vertex(vertex);
			}
		}


		/** collapse all edges that contain an empty curve (only the start and end points)*/
		void collapse_simple_edges() {
			collapse_edges_with_less_than_n_splines(3);
		}


		/** remove all edges of degree k*/
		void remove_vertices_of_degree(InternalBoostGraph::degree_size_type k) {
			VertexIterator vi, vi_end, next;
			boost::tie(vi, vi_end) = vertices();
			for (next = vi; vi != vi_end; vi = next) {
				++next;
				InternalBoostGraph::degree_size_type degree = boost::in_degree(*vi, internal_graph_) + boost::out_degree(*vi, internal_graph_);
				if(degree == k){
				remove_vertex(*vi);
				}
			}
		}

		//returns the merged edge and the pair of removed edges
		//NOTE : only works if the vertex is of degree 2 and has one in-edge and one out-edge
		std::pair<EdgeDescriptor, EdgePair> remove_degree_2_vertex_and_merge_edges(VertexDescriptor vertex_to_remove) {


			VertexDescriptor new_source;
			VertexDescriptor new_target;
			EdgeProperties new_curve;

			//first we gather the two edges's curves

					//those should vary in size between 0 and 2 only
			std::vector<VertexDescriptor> sources;
			std::vector<VertexDescriptor> targets;

			//same
			std::vector<EdgeProperties> in_curves;
			std::vector<EdgeProperties> out_curves;

			std::pair<InEdgeIterator, InEdgeIterator> in_edges = boost::in_edges(vertex_to_remove, internal_graph_);
			std::pair<OutEdgeIterator, OutEdgeIterator> out_edges = boost::out_edges(vertex_to_remove, internal_graph_);

			for (InEdgeIterator e_it(in_edges.first); e_it != in_edges.second; e_it++) {
				in_curves.push_back(get_edge(*e_it));
				sources.push_back(boost::source(*e_it, internal_graph_));
			}
			for (OutEdgeIterator e_it(out_edges.first); e_it != out_edges.second; e_it++) {
				out_curves.push_back(get_edge(*e_it));
				targets.push_back(boost::target(*e_it, internal_graph_));
			}

			//and merge them together in a way that depends on their direction (->*<- ,  <-*->, ->*-> or <-*<-)

			//in case the directions are the same we add the out curve to the incident curve
			if (in_curves.size() == 1 && out_curves.size() == 1) {

				//copy the in curve
				EdgeProperties new_prop = in_curves[0];

				//update the tangent (we don't take the first PointTangent since it's the same as 
				//the last of the incident curve. i.e. the removed vertex' position)
				new_prop.curve.back().second = (out_curves[0].curve[1].first - new_prop.curve.back().first).normalize();

				//add the out curve (same comment)
				for (GRuint i(1); i < out_curves[0].curve.size(); i++) {
					new_prop.curve.push_back(out_curves[0].curve[i]);
				}

				//and add the new curve
				new_curve = new_prop;

				//and add the new source and target
				new_source = sources[0];
				new_target = targets[0];

			}
			//and if the directions are not the same we further check if it's two out or two in edges
			else if (in_curves.size() == 2 && out_curves.size() == 0) {//case ->*<-
				//copy the first in curve
				EdgeProperties new_prop = in_curves[0];

				GRuint second_curve_size((GRuint)in_curves[1].curve.size());

				//update the tangent
				new_prop.curve.back().second = (in_curves[1].curve[second_curve_size - 2].first - new_prop.curve.back().first).normalize();

				//add the second in curve in reverse order (and with the tangents in opposite direction
				for (GRuint i(1); i < second_curve_size; i++) {
					new_prop.curve.push_back(
						PointTangent(
							in_curves[1].curve[second_curve_size - 1 - i].first,
							in_curves[1].curve[second_curve_size - 1 - i].second * (-1.f)
						)
					);
				}

				//add the new curve
				new_curve = new_prop;

				//and add the new source and target
				new_source = sources[0];
				new_target = sources[1];

			}
			else if (in_curves.size() == 0 && out_curves.size() == 2) {//case <-*->
				GRuint first_curve_size((GRuint)out_curves[0].curve.size());

				std::vector<PointTangent> first_half;
				//add the first in curve in reverse order (and with the tangents in opposite direction
				for (GRuint i(0); i < first_curve_size; i++) {
					first_half.push_back(
						PointTangent(
							out_curves[0].curve[first_curve_size - 1 - i].first,
							out_curves[0].curve[first_curve_size - 1 - i].second * (-1.f)
						)
					);
				}

				EdgeProperties new_prop;
				new_prop.curve = first_half;

				//update the tangent
				new_prop.curve.back().second = (out_curves[1].curve[1].first - new_prop.curve.back().first).normalize();

				GRuint second_curve_size((GRuint)out_curves[1].curve.size());

				//add the second in curve in standard order (except the first element)
				for (GRuint i(1); i < second_curve_size; i++) {
					new_prop.curve.push_back(
						PointTangent(
							out_curves[1].curve[i].first,
							out_curves[1].curve[i].second
						)
					);
				}

				//add the new curve
				new_curve = new_prop;

				//and add the new source and target
				new_source = targets[0];
				new_target = targets[1];

			}
			else {
				throw std::runtime_error("Could not create new edge to replace vertex of degree 2");
			}

			if (new_source != VertexDescriptor() && new_target != VertexDescriptor()) {
				//add an edge from the in-edge's source to the out-edge's target
				std::pair<EdgeDescriptor, bool> new_edge = add_edge(new_source, new_target, { new_curve });

				//remove the vertex (and both in and out-edges)
				EdgeVector removed_edges = remove_vertex(vertex_to_remove);

				//std::cout << "edge count : " << edge_count() << std::endl;
				//std::cout << " new edge length : " << new_curve.size() << std::endl;
				if (new_edge.second && removed_edges.size() == 2) {
					return std::pair<EdgeDescriptor, EdgePair>(new_edge.first, EdgePair(removed_edges[0], removed_edges[1]));
				}
			}

			throw std::runtime_error("Could not create new edge to replace vertex of degree 2");

#if 0
			if (boost::in_degree(vertex_to_remove, internal_graph_) != 1 || boost::out_degree(vertex_to_remove, internal_graph_) != 1) {
				throw std::invalid_argument("Cannot remove vertex of degree 2");
			}

			EdgeDescriptor in_edge = *(boost::in_edges(vertex_to_remove, internal_graph_).first);
			EdgeDescriptor out_edge = *(boost::out_edges(vertex_to_remove, internal_graph_).first);

			//std::cout << "in edge curve : " << &(internal_graph_[edge].curve) << std::endl;
			//copy the in curve
			DeformableSplineCurve new_curve = internal_graph_[in_edge].curve;
			DeformableSplineCurve right_curve = internal_graph_[out_edge].curve;

			//std::cout << " new curve : " << &new_curve << std::endl;
			//update the tangent (we don't take the first PointTangent since it's the same as 
			//the last of the incident curve. i.e. the removed vertex' position)
			new_curve.back().second = (right_curve.after_front().first - new_curve.before_back().first).normalize();

			//add the out curve (start from 1 because the front() one is the same as new_curve.back()
			for (GRuint i(1); i < right_curve.size(); i++) {
				new_curve.push_back(right_curve[i]);
			}

			//set the original shape
		//	new_curve.set_original_shape();


			//add an edge from the in-edge's source to the out-edge's target
			std::pair<EdgeDescriptor, bool> new_edge = add_edge(boost::source(in_edge, internal_graph_), boost::target(out_edge, internal_graph_), { new_curve });

			//remove the vertex (and both in and out-edges)
			std::vector<EdgeDescriptor> removed_edges = remove_vertex(vertex_to_remove);

			//std::cout << "edge count : " << edge_count() << std::endl;
			//std::cout << " new edge length : " << new_curve.size() << std::endl;
			if (new_edge.second && removed_edges.size() == 2) {
				return std::pair<EdgeDescriptor, EdgePair> (new_edge.first, EdgePair(removed_edges[0], removed_edges[1]));
			}
			else {
				throw std::invalid_argument("Could not create new edge to replace vertex of degree 2");
			}

#endif
		}



		/** NOTE : this will crash for chained degree-2 vertices !*/
		void remove_vertices_of_degree_2_and_merge_edges() {

			std::vector<VertexDescriptor> new_sources;
			std::vector<VertexDescriptor> new_targets;
			std::vector<EdgeProperties> new_props;

			GRuint iteration_count(0);

			//NOTE : this is a two-step process because we can't remove vertices during the loop
			//indeed this would invalidate the vertex iterator of the loop
			VertexIterator vi, vi_end, next;
			boost::tie(vi, vi_end) = vertices();
			for (next = vi; vi != vi_end; vi = next) {
				++next;
				InternalBoostGraph::degree_size_type degree = this->degree(*vi);
				std::cout << " vertex " << iteration_count << std::endl;
				std::cout << "in degree : " << boost::in_degree(*vi, internal_graph_) << std::endl;
				std::cout << "out degree : " << boost::out_degree(*vi, internal_graph_) << std::endl;
				std::cout << "degree : " << degree << std::endl << std::endl;;
				iteration_count++;
				if (degree == 2) {


					//first we gather the two edges's curves

					//those should vary in size between 0 and 2 only
					std::vector<VertexDescriptor> sources;
					std::vector<VertexDescriptor> targets;

					//same
					std::vector<EdgeProperties> in_curves;
					std::vector<EdgeProperties> out_curves;

					std::pair<InEdgeIterator, InEdgeIterator> in_edges = boost::in_edges(*vi, internal_graph_);
					std::pair<OutEdgeIterator, OutEdgeIterator> out_edges = boost::out_edges(*vi, internal_graph_);

					for (InEdgeIterator e_it(in_edges.first); e_it != in_edges.second; e_it++) {
						in_curves.push_back(get_edge(*e_it));
						sources.push_back(boost::source(*e_it, internal_graph_));
					}
					for (OutEdgeIterator e_it(out_edges.first); e_it != out_edges.second; e_it++) {
						out_curves.push_back(get_edge(*e_it));
						targets.push_back(boost::target(*e_it, internal_graph_));
					}

					//and merge them together in a way that depends on their direction (->*<- ,  <-*->, ->*-> or <-*<-)

					//in case the directions are the same we add the out curve to the incident curve
					if (in_curves.size() == 1 && out_curves.size() == 1) {
						
						//copy the in curve
						EdgeProperties new_prop = in_curves[0];

						//update the tangent (we don't take the first PointTangent since it's the same as 
						//the last of the incident curve. i.e. the removed vertex' position)
						new_prop.curve.back().second = (out_curves[0].curve[1].first - new_prop.curve.back().first).normalize();

						//add the out curve (same comment)
						for (GRuint i(1); i < out_curves[0].curve.size(); i++) {
							new_prop.curve.push_back(out_curves[0].curve[i]);
						}

						//and add the new curve
						new_props.push_back(new_prop);

						//and add the new source and target
						new_sources.push_back(sources[0]);
						new_targets.push_back(targets[0]);

					}
					//and if the directions are not the same we further check if it's two out or two in edges
					else if (in_curves.size() == 2 && out_curves.size() == 0) {//case ->*<-
						//copy the first in curve
						EdgeProperties new_prop = in_curves[0];

						GRuint second_curve_size((GRuint)in_curves[1].curve.size());

						//update the tangent
						new_prop.curve.back().second = (in_curves[1].curve[second_curve_size-2].first - new_prop.curve.back().first).normalize();

						//add the second in curve in reverse order (and with the tangents in opposite direction
						for(GRuint i(1); i< second_curve_size; i++){
							new_prop.curve.push_back(
								PointTangent(
									in_curves[1].curve[second_curve_size-1-i].first, 
									in_curves[1].curve[second_curve_size-1-i].second * (-1.f)
								)
							);
						}

						//add the new curve
						new_props.push_back(new_prop);

						//and add the new source and target
						new_sources.push_back(sources[0]);
						new_targets.push_back(sources[1]);

					}
					else if (in_curves.size() == 0 && out_curves.size() == 2) {//case <-*->
						GRuint first_curve_size((GRuint)out_curves[0].curve.size());

						std::vector<PointTangent> first_half;
						//add the first in curve in reverse order (and with the tangents in opposite direction
						for (GRuint i(0); i < first_curve_size; i++) {
							first_half.push_back(
								PointTangent(
									out_curves[0].curve[first_curve_size - 1 - i].first,
									out_curves[0].curve[first_curve_size - 1 - i].second * (-1.f)
								)
							);
						}

						EdgeProperties new_prop;
						new_prop.curve = first_half;

						//update the tangent
						new_prop.curve.back().second = (out_curves[1].curve[1].first - new_prop.curve.back().first).normalize();

						GRuint second_curve_size((GRuint)out_curves[1].curve.size());

						//add the second in curve in standard order (except the first element)
						for (GRuint i(1); i < second_curve_size; i++) {
							new_prop.curve.push_back(
								PointTangent(
									out_curves[1].curve[i].first,
									out_curves[1].curve[i].second 
								)
							);
						}

						//add the new curve
						new_props.push_back(new_prop);

						//and add the new source and target
						new_sources.push_back(targets[0]);
						new_targets.push_back(targets[1]);

					}else{
						std::cerr << " THIS SHOULDN'T HAVE HAPPENED. EXITING" << std::endl;
						exit(EXIT_FAILURE);

					}

					remove_vertex(*vi);
				}
			}

			//std::cout << "adding " << new_sources.size() << " new edges " << std::endl;

			//return;
			if (new_sources.size() != new_targets.size() || new_sources.size() != new_props.size()) {
				std::cerr << " THIS SHOULDN'T HAVE HAPPENED. EXITING" << std::endl;
				exit(EXIT_FAILURE);
			}

			for (GRuint i(0); i < new_sources.size(); i++) {
				add_edge(new_sources[i], new_targets[i], new_props[i]);
			}
		}





		/** cleans the graph by :
		- collapsing the simple edges*/
		/*void clean() {
			collapse_simple_edges(); 
			remove_vertices_of_degree_2_and_merge_edges();
		}*/


		void move_and_scale(Vector3f displacement, GRfloat scale_factor) {

			std::pair<VertexIterator, VertexIterator> vp;

			for (vp = boost::vertices(internal_graph_); vp.first != vp.second; ++vp.first) {
				VertexDescriptor v = *vp.first;
				internal_graph_[v].position = (internal_graph_[v].position + displacement)*scale_factor;
			}

			std::pair<EdgeIterator, EdgeIterator> ep;
			for (ep = boost::edges(internal_graph_); ep.first != ep.second; ++ep.first) {
				EdgeDescriptor e = *ep.first;
				for (auto& point_tangent : internal_graph_[e].curve) {
					point_tangent.first = (point_tangent.first + displacement)*scale_factor;
				}
				internal_graph_[e].curve.update_tangents();
			}
		}

		
		/******************************************************************************************** Print stuff */


		std::string to_string() {
			std::stringstream msg;

			msg << " SkeletalGraph contains " << internal_graph_.m_vertices.size() << " vertices and " << internal_graph_.m_edges.size() << " edges :" << std::endl;


			std::pair<VertexIterator, VertexIterator> vp;

			GRuint iteration_count(0);

			msg << "------ vertices ------" << std::endl;
			for (vp = boost::vertices(internal_graph_); vp.first != vp.second; ++vp.first) {
				VertexDescriptor v = *vp.first;
				msg << iteration_count << " : " << std::endl <<v<<" : "<< internal_graph_[v].position.to_string()<<", radius : "<<internal_graph_[v].radius << std::endl;

				iteration_count++;
			}
			msg << std::endl << std::endl;

			std::pair<EdgeIterator, EdgeIterator> ep;

			iteration_count = 0;

			msg << "------- edges -------" << std::endl;
			msg << "note : the first and last point should be doubled" << std::endl;
			for (ep = boost::edges(internal_graph_); ep.first != ep.second; ++ep.first) {
				EdgeDescriptor e = *ep.first;

				msg << iteration_count << " : "<<e<<std::endl
					<< " |" << internal_graph_[boost::source(e, internal_graph_)].position.to_string() << "| -> "
					<< std::endl << internal_graph_[e].curve.to_string() << std::endl
					<<" -> |"<<internal_graph_[boost::target(e, internal_graph_)].position.to_string()<<"| " << std::endl<<std::endl;

				iteration_count++;
			}
			msg << std::endl;

			return msg.str();
		}

		void print(std::queue<VertexDescriptor> q) {
			GRuint size = (GRuint)q.size();
			for (GRuint i(0); i < size; i++) {
				std::cout<<" " << get_vertex(q.front()).position.to_string();
				q.pop();
			}
			std::cout << std::endl;
		}

	};
}