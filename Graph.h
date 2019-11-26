#ifndef GRAPH_H_
#define GRAPH_H_

#include <vector> // std::vector
#include <utility> // std::pair
#include "Vertex.h"
#include "Edge.h"
#include "Node.h"
#include "XY.h"

using std::vector ;
using std::pair ;
using namespace easymath ;

typedef unsigned long int ULONG ;

// Graph class to create and store graph structure
class Graph
{
	public:
	  typedef pair<int, int> edge ;
	  Graph(vector<XY> &locations, vector<edge> &edge_array, vector< vector<double> > &cost_distribution)
	  {
	    itsVertices = GenerateVertices(locations) ;
	    itsEdges = GenerateEdges(edge_array, cost_distribution) ;
    }
    
		~Graph()
		{
			for (unsigned i = 0; i < numVertices; i++){
				delete itsVertices[i] ;
				itsVertices[i] = 0 ;
			}
	    delete [] itsVertices ;
	    itsVertices = 0 ;
	    for (unsigned i = 0; i < numEdges; i++){
		  	delete itsEdges[i] ;
		  	itsEdges[i] = 0 ;
    	}
	    delete [] itsEdges ;
	    itsEdges = 0 ;
    }
		
		Vertex ** GetVertices() const {return itsVertices ;}
		Edge ** GetEdges() const {return itsEdges ;}
		ULONG GetNumVertices() const {return numVertices ;}
		ULONG GetNumEdges() const {return numEdges ;}
		
		vector<Edge *> GetNeighbours(XY v) ;
		vector<Edge *> GetNeighbours(Vertex * v) ;
		vector<Edge *> GetNeighbours(XY v, XY v0) ;
		vector<Edge *> GetNeighbours(Vertex * v, Vertex * v0) ;
		vector<Edge *> GetNeighbours(Node * n) ;
		
	private:
		Vertex ** itsVertices ;
		Edge ** itsEdges ;
		ULONG numVertices ;
		ULONG numEdges ;
		
		Vertex ** GenerateVertices(vector<XY> &vertices) ;
		Edge ** GenerateEdges(vector<edge> &edges, vector< vector<double> > &cost_distribution) ;
} ;

vector<Edge *> Graph::GetNeighbours(XY v)
{
  Vertex * pV ;
  for (ULONG i = 0; i < numVertices; i++)
    if ( v.x == itsVertices[i]->GetX() && v.y == itsVertices[i]->GetY() )
      pV = itsVertices[i] ;

  return GetNeighbours(pV) ;
}

vector<Edge *> Graph::GetNeighbours(Vertex * v)
{
  vector<Edge *> neighbours(numEdges) ;
  ULONG k = 0 ;

  for (ULONG i = 0; i < numEdges; i++){
	  double x1 = itsEdges[i]->GetVertex1()->GetX() ;
	  double y1 = itsEdges[i]->GetVertex1()->GetY() ;
	  if (x1 == v->GetX() && y1 == v->GetY()){
		  neighbours[k] = itsEdges[i] ;
		  k++ ;
	  }
  }

  neighbours.resize(k) ;
  return neighbours ;
}

vector<Edge *> Graph::GetNeighbours(XY v, XY v0) // Do not include parent vertex in list of neighbours
{
  Vertex * pV ;
  Vertex * pV0 ;
  for (ULONG i = 0; i < numVertices; i++){
    if ( v.x == itsVertices[i]->GetX() && v.y == itsVertices[i]->GetY() )
      pV = itsVertices[i] ;
    if ( v0.x == itsVertices[i]->GetX() && v0.y == itsVertices[i]->GetY() )
      pV0 = itsVertices[i] ;
  }

  return GetNeighbours(pV, pV0) ;
}

vector<Edge *> Graph::GetNeighbours(Vertex * v, Vertex * v0) // Do not include parent vertex in list of neighbours
{
  vector<Edge *> neighbours ;

  for (ULONG i = 0; i < numEdges; i++){
	  double x1 = itsEdges[i]->GetVertex1()->GetX() ;
	  double y1 = itsEdges[i]->GetVertex1()->GetY() ;
	  double x2 = itsEdges[i]->GetVertex2()->GetX() ;
	  double y2 = itsEdges[i]->GetVertex2()->GetY() ;
	  if (x1 == v->GetX() && y1 == v->GetY() && x2 != v0->GetX() && y2 != v0->GetY()){
		  neighbours.push_back(itsEdges[i]) ;
	  }
  }

  return neighbours ;
}

vector<Edge *> Graph::GetNeighbours(Node * n) // Do not include parent vertex in list of neighbours
{
  vector<Edge *> neighbours ;
  Vertex * v = n->GetVertex() ;

  for (ULONG i = 0; i < numEdges; i++){
	  double x1 = itsEdges[i]->GetVertex1()->GetX() ;
	  double y1 = itsEdges[i]->GetVertex1()->GetY() ;
	  double x2 = itsEdges[i]->GetVertex2()->GetX() ;
	  double y2 = itsEdges[i]->GetVertex2()->GetY() ;
  	if (x1 == v->GetX() && y1 == v->GetY()){
		  bool isNeighbour = true ;
		  Node * n0 = n ;
		  while (n0->GetParent()){
				n0 = n0->GetParent() ;
		  	Vertex * v0 = n0->GetVertex() ;
				if (x2 == v0->GetX() && y2 == v0->GetY()){
					isNeighbour = false ;
					break ;
				}
			}
			if (isNeighbour)
				neighbours.push_back(itsEdges[i]) ;
	  }
  }

  return neighbours ;
}

Vertex ** Graph::GenerateVertices(vector<XY> &vertices)
{
  numVertices = (ULONG)vertices.size() ;
  Vertex ** allVertices = new Vertex * [numVertices] ;
	
  for (ULONG i = 0; i < numVertices; i++)
  {
    double x = vertices[i].x ;
    double y = vertices[i].y ;

    allVertices[i] = new Vertex(x,y) ;
  }
  
  return allVertices ;
}

Edge ** Graph::GenerateEdges(vector<edge> &edges, vector< vector<double> > &cost_distribution)
{
  numEdges = (ULONG)edges.size() ;
  Edge ** allEdges = new Edge * [numEdges] ;

  for (ULONG i = 0; i < numEdges; i++)
  {
    Vertex * v1 = itsVertices[(ULONG)edges[i].first] ;
    Vertex * v2 = itsVertices[(ULONG)edges[i].second] ;
    double cost = cost_distribution[i][0] ;
    double sig = cost_distribution[i][1] ;

    allEdges[i] = new Edge(v1, v2, cost, sig) ;
  }

  return allEdges ;
}

#endif // GRAPH_H_
