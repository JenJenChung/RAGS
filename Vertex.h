#ifndef VERTEX_H_
#define VERTEX_H_

#include <vector> // std::vector

using std::vector ;

// Vertex class to contain location of vertices
class Node ;
class Vertex
{
	public:
		Vertex(double x, double y):
		  itsX(x), itsY(y) {}
		~Vertex() {}
		
		double GetX() const {return itsX ;}
		void SetX(double x) {itsX = x ;}
		double GetY() const {return itsY ;}
		void SetY(double y) {itsY = y ;}
		double GetCTC()const{return itsCTC ;}
		void SetCTC(double cost_to_come) {itsCTC = cost_to_come ;}
		void SetNodes(vector<Node *> NewNodes) {itsNodes = NewNodes ;}
		vector<Node *> GetNodes() {return itsNodes ;}
		void SetActualCost(double ac) {itsActualCost = ac;}
		double GetActualCost() const {return itsActualCost ;}
		
		friend bool operator==(const Vertex &lhs, const Vertex &rhs){
			return lhs.GetX()==rhs.GetX() && lhs.GetY()==rhs.GetY();
		}

	private:
		double itsActualCost ;
		double itsX ;
		double itsY ;
		double itsCTC ;
		vector<Node *> itsNodes ;
};

#endif // VERTEX_H_
