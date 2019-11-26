#ifndef EDGE_H_
#define EDGE_H_

#include <random> // std::default_random_engine, std::normal distribution
#include <math.h> // pow, sqrt
#include "Vertex.h"

using std::default_random_engine ;
using std::normal_distribution ;

// Edge class to contain mean and standard deviation of cost along an edge
class Edge
{
	public:
		Edge(Vertex * v1, Vertex * v2, double cost, double sig):
		  itsVertex1(v1), itsVertex2(v2), itsMeanCost(cost), itsSigCost(sig), itsMeanSearch(cost), itsSigSearch(sig) {}
		~Edge(){}
		
		Vertex * GetVertex1() const {return itsVertex1 ;}
		Vertex * GetVertex2() const {return itsVertex2 ;}
		double GetMeanCost() const {return itsMeanCost ;}
		void SetMeanCost(double cost) {itsMeanCost = cost ;}
		double GetSigCost() const {return itsSigCost ;}
		void SetSigCost(double sig) {itsSigCost = sig ;}
		double GetTrueCost() {return itsTrueCost ;}
		void SetTrueCost(double cost) {itsTrueCost = cost ;}
		double GetMeanSearch() const {return itsMeanSearch ;}
		void SetMeanSearch(double cost) {itsMeanSearch = cost ;}
		double GetSigSearch() const {return itsSigSearch ;}
		void SetSigSearch(double sig) {itsSigSearch = sig ;}
		void SetTrueCost(default_random_engine generator) ;
	private:
		Vertex * itsVertex1 ;
		Vertex * itsVertex2 ;
		double itsMeanCost ;
		double itsSigCost ;
		double itsTrueCost ;
		double itsMeanSearch ; // Actual value used in search
		double itsSigSearch ; // Actual value used in search
} ;

void Edge::SetTrueCost(default_random_engine generator)
{
  double diff_x = itsVertex1->GetX() - itsVertex2->GetX() ;
  double diff_y = itsVertex1->GetY() - itsVertex2->GetY() ;
  double diff = sqrt(pow(diff_x,2) + pow(diff_y,2)) ;
  normal_distribution<double> distribution(itsMeanCost,itsSigCost) ;
  itsTrueCost = distribution(generator) ;
  if (itsTrueCost < diff)
    itsTrueCost = diff ;
}

#endif // EDGE_H_
