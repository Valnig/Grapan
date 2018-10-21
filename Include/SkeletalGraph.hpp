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

#include <vector>
#include <list>

#include "GrapholonTypes.hpp"

struct Position3d {
	GRfloat X;
	GRfloat Y;
	GRfloat Z;
};

typedef std::vector<Position3d> CurvePoints;

class VertexProperties {
	Position3d position;
};

class EdgeProperties {
	CurvePoints curve;
};


typedef std::pair<GRuint, GRuint> Edge;

typedef std::list<Edge> OutEdgeList;

typedef boost::adjacency_list<
	boost::listS, boost::listS, boost::undirectedS,
	VertexProperties, EdgeProperties>
	InternalBoostGraph;

class SkeletalGraph {
private:
	InternalBoostGraph internal_graph_;

public:
	SkeletalGraph(GRuint vertex_count, OutEdgeList out_edge_list) :
		internal_graph_(out_edge_list.begin(), out_edge_list.end(), vertex_count) {

		std::cout << "created SkeletalGraph with " << vertex_count << " vertices and " << out_edge_list.size() << " edges : " << std::endl;
	}

	SkeletalGraph(GRuint vertex_count) :internal_graph_(vertex_count) {
		std::cout << "created SkeletalGraph with " << vertex_count << " vertices and no edges : " << std::endl;

	}


};