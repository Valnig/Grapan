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

//TODO : add sizes on each edge_to_collapse and vertex

namespace grapholon {

	struct VertexProperties {
		Vector3f position;
		bool visited = false;
	};

	struct EdgeProperties {
		DeformableSplineCurve curve;
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

	class SkeletalGraph {
	private:

		GRuint edge_spline_count_ = 0;

		InternalBoostGraph internal_graph_;

		/** Vertices and edges */

	public:

		typedef enum { SOURCE, TARGET, MIDPOINT } COLLAPSE_OPTION;


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
			clear_vertex(vertex);
			boost::remove_vertex(vertex, internal_graph_);
		}

		void clear_vertex(VertexDescriptor vertex) {
			std::pair<InEdgeIterator, InEdgeIterator> in_edges = boost::in_edges(vertex, internal_graph_);
			std::pair<OutEdgeIterator, OutEdgeIterator> out_edges = boost::out_edges(vertex, internal_graph_);

			//in theory we only need to iterate through in_edges since it's an undirected graph
			for (InEdgeIterator e_it(in_edges.first); e_it != in_edges.second; e_it++) {
				edge_spline_count_ -= (GRuint)internal_graph_[*e_it].curve.size();
			}
			for (OutEdgeIterator e_it(out_edges.first); e_it != out_edges.second; e_it++) {
				edge_spline_count_ -= (GRuint)internal_graph_[*e_it].curve.size();
			}

			boost::clear_vertex(vertex, internal_graph_);
		}

		const VertexProperties& get_vertex(VertexDescriptor vertex) const {
			return internal_graph_[vertex];
		}

		std::pair<VertexIterator, VertexIterator> vertices() {
			return boost::vertices(internal_graph_);
		}

		bool update_vertex_position(VertexDescriptor vertex, Vector3f new_position) {

			internal_graph_[vertex] = { new_position };

			std::pair<InEdgeIterator, InEdgeIterator> in_edges = boost::in_edges(vertex, internal_graph_);
			std::pair<OutEdgeIterator, OutEdgeIterator> out_edges = boost::out_edges(vertex, internal_graph_);

			//in theory we only need to iterate through in_edges since it's an undirected graph
			for (InEdgeIterator e_it(in_edges.first); e_it != in_edges.second; e_it++) {
				if (!internal_graph_[*e_it].curve.pseudo_elastic_deform(false, new_position)) {
					return false;
				}
			}
			for (OutEdgeIterator e_it(out_edges.first); e_it != out_edges.second; e_it++) {
				if (!internal_graph_[*e_it].curve.pseudo_elastic_deform(true, new_position)) {
					return false;
				}
			}

			return true;
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
		
		VertexDescriptor collapse_edge(EdgeDescriptor edge_to_collapse, COLLAPSE_OPTION option = SOURCE) {

			//first attach the edge_to_collapse's source's edges to the edge_to_collapse's target
			VertexDescriptor source = boost::source(edge_to_collapse, internal_graph_);
			VertexDescriptor target = boost::target(edge_to_collapse, internal_graph_);

			//std::cout << "collapsing edge from " << internal_graph_[source].position.to_string() << " to " << internal_graph_[target].position.to_string() << std::endl;

			//check if edge exists
			if (source == InternalBoostGraph::null_vertex() 
				|| target == InternalBoostGraph::null_vertex()
				|| !boost::edge(source, target, internal_graph_).second) {
				std::cout << "edge doesn't exist" << std::endl;
				return InternalBoostGraph::null_vertex();
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
			clear_vertex(to_remove);

			//add the new edges
			for (GRuint i(0); i < sources_to_add.size(); i++) {
				add_edge(sources_to_add[i], to_keep, source_props_to_add[i]);
			}

			for (GRuint i(0); i < targets_to_add.size(); i++) {
				add_edge(to_keep, targets_to_add[i], target_props_to_add[i]);
			}

			//and update the vertex to keep's position
			internal_graph_[to_keep].position = new_position;

			return to_remove;
		}




		/** General operations*/

		/** collapse all edges that contain an empty curve (only the start and end points)*/
		void collapse_simple_edges() {

			std::vector<EdgeDescriptor> edges_to_collapse;
			std::pair<EdgeIterator, EdgeIterator> e_it;
			for (e_it = boost::edges(internal_graph_); e_it.first != e_it.second; ++e_it.first) {
				if (is_simple_edge(*e_it.first)) {
					edges_to_collapse.push_back(*e_it.first);
				}
			}

			//collapse the edges at midpoints and gather the vertices to remove
			std::vector<VertexDescriptor> vertices_to_remove;
			std::cout << "found " << edges_to_collapse.size() << " edges to collapse " << std::endl;
			for (auto edge : edges_to_collapse) {
				VertexDescriptor vertex = collapse_edge(edge, MIDPOINT);
				if (vertex != InternalBoostGraph::null_vertex()) {
					vertices_to_remove.push_back(vertex);
					std::cout << " will remove vertex at " << internal_graph_[vertex].position.to_string() << std::endl;
				}
			}

			//and remove the now-alone vertices
			for (auto vertex : vertices_to_remove) {
				remove_vertex(vertex);
			}
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


		void remove_vertices_of_degree_2_and_merge_edges() {

			std::vector<VertexDescriptor> new_sources;
			std::vector<VertexDescriptor> new_targets;
			std::vector<EdgeProperties> new_props;

			VertexIterator vi, vi_end, next;
			boost::tie(vi, vi_end) = vertices();
			for (next = vi; vi != vi_end; vi = next) {
				++next;
				InternalBoostGraph::degree_size_type degree = boost::in_degree(*vi, internal_graph_) + boost::out_degree(*vi, internal_graph_);
				if (degree == 2) {

					//std::cout << " vertex at " << get_vertex(*vi).position.to_string() << " has degree 2" << std::endl;
					//merge the two edges into a new one

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

					/*std::cout << " sources : " << std::endl;
					for (auto vertex : sources) {
						std::cout << get_vertex(vertex).position.to_string() << " ";
					}
					std::cout << std::endl;

					std::cout << " targets : " << std::endl;
					for (auto vertex : targets) {
						std::cout << get_vertex(vertex).position.to_string() << " ";
					}
					std::cout << std::endl;*/


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


		/** collapse all edges in to a single point (at the center) and	
		creates an edge from this center to each vertices surrounding the edges*/
		void collapse_edges_at_center(std::vector<EdgeDescriptor>& edges) {

		}

		//MAYBE TODO (goes allong with collapse_edges_at_center
		void collapse_simple_edges_at_centers() {

		}

		/** cleans the graph by :
		- collapsing the simple edges*/
		void clean() {
			collapse_simple_edges(); 
			remove_vertices_of_degree_2_and_merge_edges();
		}


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
			msg << "note : the first and last point should be doubled" << std::endl;
			for (ep = boost::edges(internal_graph_); ep.first != ep.second; ++ep.first) {
				EdgeDescriptor e = *ep.first;

				msg << iteration_count << " : |"
					<< internal_graph_[boost::source(e, internal_graph_)].position.to_string() << "| -> "
					<< std::endl << internal_graph_[e].curve.to_string() << std::endl
					<<" -> |"<<internal_graph_[boost::target(e, internal_graph_)].position.to_string()<<"| " << std::endl<<std::endl;

				iteration_count++;
			}
			msg << std::endl;

			return msg.str();
		}


	};
}