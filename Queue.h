#ifndef QUEUE_H_
#define QUEUE_H_

#include <vector> // std::vector
#include <queue> // std::priority_queue
#include <boost/math/special_functions/erf.hpp> // erf_inv
#include "Node.h"
#include "Search.h"

using namespace boost::math ; // erf_inv
using std::vector ;
using std::priority_queue ;

typedef unsigned long int ULONG ;

#ifndef PTHRESH
#define PTHRESH 0.50
#endif

double pthresh = 0.5 ;
heuristic COMPARE = RAGSCOMPARE ;

class CompareNode{
  double eConst ;
  public:
    CompareNode(){eConst = erf_inv(1.0-2.0*pthresh) ;}
    ~CompareNode(){}
    bool operator() (const Node * n1, const Node * n2) const{
      if (COMPARE == RAGSCOMPARE){
        double n1Cost = n1->GetMeanCost() ;
        double n2Cost = n2->GetMeanCost() ;
        double thresh = n1Cost + sqrt(2.0)*sqrt(pow(n1->GetSigCost(),2) + pow(n2->GetSigCost(),2))*eConst ;
        return (n2Cost < thresh) ;
      }
      else{
        double n1Cost = n1->GetMeanCost()+n1->GetHeuristic() ;
        double n2Cost = n2->GetMeanCost()+n2->GetHeuristic() ;
        return (n2Cost < n1Cost) ;
      }
    }
} ;

// Custom queue type to perform priority queue updates
class Queue
{
	public:
  	typedef priority_queue<Node *, vector<Node *>, CompareNode> QUEUE ;
		Queue(Node * source, pathOut pType){
		  if (pType == ALL)
		    pthresh = PTHRESH ;
	    else
	      COMPARE = HEURISTIC ;
	    itsPQ = new QUEUE ;
      eConst = erf_inv(1.0-2.0*pthresh) ;
	    itsPQ->push(source) ;
    }
    
		~Queue(){
			while (!itsPQ->empty()){
				Node * temp = itsPQ->top() ;
				delete temp ;
				temp = 0 ;
				itsPQ->pop() ;
			}
	    delete itsPQ ;
	    itsPQ = 0 ;
	    for (ULONG i = 0; i < closed.size(); i ++){
		    delete closed[i] ;
		    closed[i] = 0 ;
	    }
    }
		
		vector<Node *> GetClosed() const {return closed ;}
		bool EmptyQueue() const {return itsPQ->empty() ;}
		ULONG SizeQueue() const {return (ULONG)itsPQ->size() ;}
		void UpdateQueue(Node * newNode) ;
		Node * PopQueue() ;
    
	private:
		QUEUE * itsPQ ;
		vector<Node *> closed ;
		double eConst ;
		
		bool CompareNodes(const Node * n1, const Node * n2) const ;
} ;

void Queue::UpdateQueue(Node * newNode)
{
  // Compare newNode to nodes in closed set
  // if closed contains node with same vertex, compare their costs
  // choose whether or not to create a new node
  bool dom = false ;
  for (ULONG i = 0; i < closed.size(); i++){
    if (closed[i]->GetVertex() == newNode->GetVertex()){
	    dom = CompareNodes(newNode, closed[i]) ;
	    if (dom){
	    	delete newNode ;
	    	newNode = 0 ;
		    return ;
	    }
    }
  }
  itsPQ->push(newNode) ;
}

Node * Queue::PopQueue()
{
  // Check if next node is already dominated by existing node in closed set
  Node * newNode = itsPQ->top() ;
  bool dom = false ;
  for (ULONG i = 0; i < closed.size(); i++){
    if (closed[i]->GetVertex() == newNode->GetVertex()){
      dom = CompareNodes(newNode, closed[i]) ;
      if (dom){
      	delete newNode ;
      	newNode = 0 ;
	      itsPQ->pop() ;
	      return 0 ;
      }
    }
  }
  closed.push_back(itsPQ->top()) ;
  itsPQ->pop() ;
  return closed[(ULONG)closed.size()-1] ;
}

bool Queue::CompareNodes(const Node * n1, const Node * n2) const
{
	// out: Is n1 worse than or equal to n2? Does n2 dominate n1?
  double n1Cost = n1->GetMeanCost() ;
  double n2Cost = n2->GetMeanCost() ;
//  return (n1Cost >= n2Cost && n1->GetSigCost() >= n2->GetSigCost()) ; // old domination metric
  /*	if (n1Cost > n2Cost && n1->GetSigCost() > n2->GetSigCost())
    return true ;
  else if (n2Cost > n1Cost && n2->GetSigCost() > n1->GetSigCost())
    return false ;
  else
    return (n1Cost > n2Cost) ;*/
  double thresh = n1Cost + sqrt(2.0)*sqrt(pow(n1->GetSigCost(),2) + pow(n2->GetSigCost(),2))*eConst ;
  return (n2Cost < thresh) ; // does n2 dominate n1?
}

#endif //QUEUE_H_
