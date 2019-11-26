#ifndef RAGS_H_
#define RAGS_H_

#include <iostream> // std::cout
#include <fstream> // std::fstream::fstream
#include <vector> // std::vector
#include <algorithm> // std::sort
#include <utility> // std::pair
#include <math.h> // erfc, pow, sqrt
#include <stdio.h> // printf
#include "Vertex.h"
#include "Node.h"
#include "Graph.h"
#include "Search.h"
#include "XY.h"

using namespace easymath ;
using std::cout ;
using std::vector ;
using std::sort ;
using std::pair ;
using std::ofstream ;
using std::ifstream ;

const double PI = 3.14159265358979323846264338328 ;

// RAGS class for interfacing with sector agents
class RAGS
{
  public:
    typedef pair<int, int> edge ;
    RAGS(vector<XY> &locations, vector<edge> &edge_array, vector< vector<double> > &cost_distribution, pathOut POUT = ALL): 
    itsLocations(locations), itsEdgeArray(edge_array), PSET(POUT){
    	itsGraph = new Graph(locations, edge_array, cost_distribution) ;
    	itsSearch = 0 ;
    }
    
    ~RAGS()
    {
      if (itsGraph){
        delete itsGraph ;
        itsGraph = 0 ;
      }
      if (itsSearch){
        delete itsSearch ;
        itsSearch = 0 ;
      }
    	for (unsigned i = 0; i < itsNDSet.size(); i++){
    		Node * hN ;
    		Node * pN = itsNDSet[i]->GetParent() ;
    		while (pN){
    			hN = pN->GetParent() ;
    			delete pN ;
    			pN = hN ;
  			}
    		delete itsNDSet[i] ;
    		itsNDSet[i] = 0 ;
  		}
    }
    
    Graph * GetGraph() const {return itsGraph ;}
    Vertex * GetVert() const {return itsVert ;}
    
    void SetInitialVert(XY start) ;
    void SetInitialVert(Vertex * start) ;
    bool InitialiseNDSet(XY start, XY goal, XY &s) ;
    XY SearchGraph(XY start, XY goal, vector<double> &costs) ;
    XY SearchGraphGreedy(XY start, XY goal, vector<double> &costs) ;
    int GetEdgeIndex(XY start, XY goal) ;
    ULONG GetNDSetSize(){return itsNDSet.size() ;}
    vector<Node *> GetNDSet(){return itsNDSet ;}
    
    void NDSetWriteBinary(ofstream &fs) ;
    void NDSetReadBinary(ifstream &fs) ;
    
  private:
  	vector<XY> itsLocations ;
    vector<edge> itsEdgeArray ;
    Graph * itsGraph ;
    Search * itsSearch ;
    Vertex * itsVert ;
    vector<Node *> itsNDSet ;
    pathOut PSET ;
    
    XY SearchGraph(Vertex * start, Vertex * goal, vector<double> &costs) ;
    XY SearchGraphGreedy(Vertex * start, Vertex * goal, vector<double> &costs) ;
    bool InitialiseNDSet(Vertex* start, Vertex* goal, XY &s) ;
    void AssignCurrentEdgeCosts(vector<double> &costs) ;
    void AssignCTC(Vertex * A, Vertex * B, vector<double> &costs) ;
    static bool IsABetterThanB(Vertex * A, Vertex * B) ;
    static double ComputeImprovementProbability(Vertex * A, Vertex * B) ;
    static vector<double> linspace(double a, double b, int n) ;
    static bool GreedyComparison(Vertex * A, Vertex * B) ;
    
    void NodeWriteBinary(ofstream &fs, Node * n, int nodeNum) ;
    ULONG GetVertexIndex(Vertex * v) ;
    Vertex * GetVertex(ULONG i) ;
} ;

void RAGS::SetInitialVert(XY start){
	Vertex ** allVerts = itsGraph->GetVertices() ;
  Vertex * sVert ;
  bool sFound = false ;
  for (ULONG i = 0; i < itsGraph->GetNumVertices(); i++)
  {
    if ( start.x == allVerts[i]->GetX() && start.y == allVerts[i]->GetY() ){
      sVert = allVerts[i] ;
      sFound = true ;
    }
  }
  if (!sFound){
  	printf("ERROR: start vertex (%f,%f) not found. Exiting.\n",start.x,start.y) ;
  	exit(1) ;
  }
  return SetInitialVert(sVert) ;
}

void RAGS::SetInitialVert(Vertex * start)
{
  Vertex ** allVerts = itsGraph->GetVertices() ;
  for (ULONG i = 0; i < itsGraph->GetNumVertices(); i++)
  {
    if ( start == allVerts[i] )
      itsVert = allVerts[i] ;
  }
}

bool RAGS::InitialiseNDSet(XY start, XY goal, XY &s){
	Vertex ** allVerts = itsGraph->GetVertices() ;
  Vertex * sVert ;
  Vertex * gVert ;
  bool sFound = false ;
  bool gFound = false ;
  for (ULONG i = 0; i < itsGraph->GetNumVertices(); i++)
  {
    if ( start.x == allVerts[i]->GetX() && start.y == allVerts[i]->GetY() ){
      sVert = allVerts[i] ;
      sFound = true ;
    }
  }
  for (ULONG i = 0; i < itsGraph->GetNumVertices(); i++)
  {
    if ( goal.x == allVerts[i]->GetX() && goal.y == allVerts[i]->GetY() ){
      gVert = allVerts[i] ;
      gFound = true ;
    }
  }
  if (!sFound)
  	printf("ERROR: start vertex (%f,%f) not found. Exiting.\n",start.x,start.y) ;
  if (!gFound)
  	printf("ERROR: goal vertex (%f,%f) not found. Exiting.\n",goal.x,goal.y) ;
	if (!sFound || !gFound)
		exit(1) ;
  return InitialiseNDSet(sVert, gVert, s) ;
}

bool RAGS::InitialiseNDSet(Vertex* start, Vertex* goal, XY &s){
  bool pFound = true ;
  itsSearch = new Search(itsGraph, start, goal) ;
  vector<Node *> GSPaths = itsSearch->PathSearch(PSET) ;
  if (GSPaths.empty()){
    s = XY(start->GetX(),start->GetY()) ; // no paths found, stay where you are
    pFound = false ;
    return pFound ;
  }
  
  for (ULONG i = 0; i < (ULONG)GSPaths.size(); i++)
    itsNDSet.push_back(GSPaths[i]->ReverseList(0)) ;
	
  for (ULONG i = 0; i < (ULONG)itsNDSet.size(); i++)
    itsNDSet[i]->SetCTG(GSPaths[i]->GetMeanCost(),GSPaths[i]->GetSigCost()) ;
  SetInitialVert(itsNDSet[0]->GetVertex()) ;
  
  return pFound ;
}

XY RAGS::SearchGraph(XY start, XY goal, vector<double> &costs)
// Create function that converts from XY to Vertex *
{
	Vertex ** allVerts = itsGraph->GetVertices() ;
  Vertex * sVert ;
  Vertex * gVert ;
  bool sFound = false ;
  bool gFound = false ;
  for (ULONG i = 0; i < itsGraph->GetNumVertices(); i++)
  {
    if ( start.x == allVerts[i]->GetX() && start.y == allVerts[i]->GetY() ){
      sVert = allVerts[i] ;
      sFound = true ;
    }
  }
  for (ULONG i = 0; i < itsGraph->GetNumVertices(); i++)
  {
    if ( goal.x == allVerts[i]->GetX() && goal.y == allVerts[i]->GetY() ){
      gVert = allVerts[i] ;
      gFound = true ;
    }
  }
  if (!sFound)
  	printf("ERROR: start vertex (%f,%f) not found. Exiting.\n",start.x,start.y) ;
  if (!gFound)
  	printf("ERROR: goal vertex (%f,%f) not found. Exiting.\n",goal.x,goal.y) ;
	if (!sFound || !gFound)
		exit(1) ;
  return SearchGraph(sVert, gVert, costs) ;
}

XY RAGS::SearchGraph(Vertex * start, Vertex * goal, vector<double> &costs)
{
  AssignCurrentEdgeCosts(costs) ;
  
  // Initialise non-dominated path set
  if (itsNDSet.empty()){
    XY s ;
    bool pFound = InitialiseNDSet(start, goal, s) ;
    if (!pFound)
      return s ;
  }
  
  // Flag and return if current vertex does not match start vertex
  if (!(itsVert == start)){
    printf("\nERROR: input start vertex (%f,%f) does not match stored vertex (%f,%f)", 
    	start->GetX(), start->GetY(), itsVert->GetX(), itsVert->GetY()) ; 
    XY s = XY(start->GetX(),start->GetY()) ;
    return s ;
  }
	
  vector<Node *> newNodes = itsNDSet ;
  itsVert->SetNodes(newNodes) ;
  vector<Node *> tmpNodes ;
  vector<Node *> noNodes ; // keep track of unvisited nodes
  vector<Vertex *> nextVerts ;
  
  // Identify next vertices
  for (unsigned i = 0; i < newNodes.size(); i++){
	  bool newVert = true ;
	  for (unsigned j = 0; j < nextVerts.size(); j++){
		  if (nextVerts[j] == newNodes[i]->GetParent()->GetVertex() || nextVerts[j] == itsVert){
			  newVert = false ;
			  break ;
		  }
	  }
	  if (newVert){
		  nextVerts.push_back(newNodes[i]->GetParent()->GetVertex()) ;
	    AssignCTC(itsVert, newNodes[i]->GetParent()->GetVertex(), costs) ;
    }
  }
  
  // Identify next vertex path nodes
  for (unsigned i = 0; i < nextVerts.size(); i++){
	  tmpNodes.clear() ;
	  for (unsigned j = 0; j < newNodes.size(); j++)
		  if (nextVerts[i] == newNodes[j]->GetParent()->GetVertex())
			  tmpNodes.push_back(newNodes[j]->GetParent()) ;
		  else
		  	noNodes.push_back(newNodes[j]->GetParent()) ;
	  nextVerts[i]->SetNodes(tmpNodes) ;
  }
	
	// Rank next vertices according to probability of improvement
	sort(nextVerts.begin(),nextVerts.end(),IsABetterThanB) ;
	itsVert = nextVerts[0] ;
	
	// Delete nodes of vertices that will not be considered along entire link list
	for (unsigned i = 1; i < nextVerts.size(); i++){
		vector<Node *> nN = nextVerts[i]->GetNodes() ;
		for (unsigned j = 0; j < nN.size(); j++){
			Node * hN ;
			Node * pN = nN[j]->GetParent() ;
			while (pN){
				hN = pN->GetParent() ;
				delete pN ;
				pN = hN ;
			}
			delete nN[j] ;
			nN[j] = 0 ;
		}
  }
	
	// Delete old NDSet
	for (unsigned i = 0; i < itsNDSet.size(); i++){
		delete itsNDSet[i] ;
		itsNDSet[i] = 0 ;
	}
	itsNDSet.clear() ;
	itsNDSet = itsVert->GetNodes() ;
	XY vertXY = XY(itsVert->GetX(),itsVert->GetY()) ;
	return vertXY ;
}

XY RAGS::SearchGraphGreedy(XY start, XY goal, vector<double> &costs)
// Create function that converts from XY to Vertex *
{
	Vertex ** allVerts = itsGraph->GetVertices() ;
  Vertex * sVert ;
  Vertex * gVert ;
  bool sFound = false ;
  bool gFound = false ;
  for (ULONG i = 0; i < itsGraph->GetNumVertices(); i++)
  {
    if ( start.x == allVerts[i]->GetX() && start.y == allVerts[i]->GetY() ){
      sVert = allVerts[i] ;
      sFound = true ;
    }
  }
  for (ULONG i = 0; i < itsGraph->GetNumVertices(); i++)
  {
    if ( goal.x == allVerts[i]->GetX() && goal.y == allVerts[i]->GetY() ){
      gVert = allVerts[i] ;
      gFound = true ;
    }
  }
  if (!sFound)
  	printf("ERROR: start vertex (%f,%f) not found. Exiting.\n",start.x,start.y) ;
  if (!gFound)
  	printf("ERROR: goal vertex (%f,%f) not found. Exiting.\n",goal.x,goal.y) ;
	if (!sFound || !gFound)
		exit(1) ;
  return SearchGraphGreedy(sVert, gVert, costs) ;
}

XY RAGS::SearchGraphGreedy(Vertex * start, Vertex * goal, vector<double> &costs)
{
  AssignCurrentEdgeCosts(costs) ;
  
  // Initialise non-dominated path set
  if (itsNDSet.empty()){
    XY s ;
    bool pFound = InitialiseNDSet(start, goal, s) ;
    if (!pFound)
      return s ;
  }
    
  // Flag and return if current vertex does not match start vertex
  if (!(itsVert == start)){
    printf("\nERROR: input start vertex (%f,%f) does not match stored vertex (%f,%f)", 
    	start->GetX(), start->GetY(), itsVert->GetX(), itsVert->GetY()) ; 
    XY s = XY(start->GetX(),start->GetY()) ;
    return s ;
  }
	
  vector<Node *> newNodes = itsNDSet ;
  itsVert->SetNodes(newNodes) ;
  vector<Node *> tmpNodes ;
  vector<Node *> noNodes ; // keep track of unvisited nodes
  vector<Vertex *> nextVerts ;
  
  // Identify next vertices
  for (unsigned i = 0; i < newNodes.size(); i++){
	  bool newVert = true ;
	  for (unsigned j = 0; j < nextVerts.size(); j++){
		  if (nextVerts[j] == newNodes[i]->GetParent()->GetVertex() || nextVerts[j] == itsVert){
			  newVert = false ;
			  break ;
		  }
	  }
	  if (newVert){
		  nextVerts.push_back(newNodes[i]->GetParent()->GetVertex()) ;
	    AssignCTC(itsVert, newNodes[i]->GetParent()->GetVertex(), costs) ;
	  }
  }
  
  // Identify next vertex path nodes
  for (unsigned i = 0; i < nextVerts.size(); i++){
	  tmpNodes.clear() ;
	  for (unsigned j = 0; j < newNodes.size(); j++)
		  if (nextVerts[i] == newNodes[j]->GetParent()->GetVertex())
			  tmpNodes.push_back(newNodes[j]->GetParent()) ;
		  else
		  	noNodes.push_back(newNodes[j]->GetParent()) ;
	  nextVerts[i]->SetNodes(tmpNodes) ;
  }
	
	// Rank next vertices according to true immediate transition cost
	sort(nextVerts.begin(),nextVerts.end(),GreedyComparison) ;
	itsVert = nextVerts[0] ;
	
	// Delete nodes of vertices that will not be considered along entire link list
	for (unsigned i = 1; i < nextVerts.size(); i++){
		vector<Node *> nN = nextVerts[i]->GetNodes() ;
		for (unsigned j = 0; j < nN.size(); j++){
			Node * hN ;
			Node * pN = nN[j]->GetParent() ;
			while (pN){
				hN = pN->GetParent() ;
				delete pN ;
				pN = hN ;
			}
			delete nN[j] ;
			nN[j] = 0 ;
		}
  }
	
	// Delete old NDSet
	for (unsigned i = 0; i < itsNDSet.size(); i++){
		delete itsNDSet[i] ;
		itsNDSet[i] = 0 ;
	}
	itsNDSet.clear() ;
	itsNDSet = itsVert->GetNodes() ;
	XY vertXY = XY(itsVert->GetX(),itsVert->GetY()) ;
	return vertXY ;
}

void RAGS::AssignCurrentEdgeCosts(vector<double> &costs)
{
  ULONG n = itsGraph->GetNumEdges() ;
  Edge ** e = itsGraph->GetEdges() ;
  
  for (ULONG i = 0; i < n; i++)
    e[i]->SetTrueCost(costs[i]) ;
}

void RAGS::AssignCTC(Vertex * A, Vertex * B, vector<double> &costs)
{
  ULONG n = itsGraph->GetNumEdges() ;
  Edge ** e = itsGraph->GetEdges() ;
  
  bool vFound = false ;
  for (ULONG i = 0; i < n; i++){
    if (e[i]->GetVertex1() == A && e[i]->GetVertex2() == B){
      B->SetCTC(costs[i]) ;
      vFound = true ;
    }
  }
  if (!vFound)
    cout << "ERROR: Edge not found!\n" ;
}

int RAGS::GetEdgeIndex(XY start, XY goal)
{
	int startID ;
	int goalID ;
	bool foundStart = false ;
	bool foundGoal = false ;
	for (unsigned i = 0; i < itsLocations.size(); i++){
		if (start == itsLocations[i]){
			startID = i ;
			foundStart = true ;
		}
		if (goal == itsLocations[i]){
			goalID = i ;
			foundGoal = true ;
		}
		if (foundStart && foundGoal)
			break ;
	}
	if (!foundStart){
		cout << "ERROR: Did not find current vertex index.\n" ;
		exit(1) ;
	}
	if (!foundGoal){
		cout << "ERROR: Did not find next vertex index.\n" ;
		exit(1) ;
	}
	for (unsigned i = 0; i < itsEdgeArray.size(); i++){
		if (itsEdgeArray[i].first == startID && itsEdgeArray[i].second == goalID)
			return i ;
	}
	cout << "Error: Did not find current edge index. Exiting.\n" ;
	exit(1) ;
}

// Read and write ND to binary file
void RAGS::NDSetWriteBinary(ofstream &fs){
  int numPaths = itsNDSet.size() ;
  cout << "Number of paths to be written to file: " << numPaths << "\n" ;
  char * p_numPaths = reinterpret_cast<char *>(&numPaths) ;
  fs.write(p_numPaths,sizeof(numPaths)) ; // record size of non-dominated set
  for (size_t i = 0; i < itsNDSet.size(); i++)
    NodeWriteBinary(fs,itsNDSet[i],0) ;
}

void RAGS::NodeWriteBinary(ofstream &fs, Node * n, int nodeNum){
  if (n->GetParent()) // write from goal node first
    NodeWriteBinary(fs, n->GetParent(), nodeNum+1) ;
  
  // Get vertex index
  ULONG vInd = GetVertexIndex(n->GetVertex()) ;
  
  // Set parent index
  int pInd ;
  if (!n->GetParent())
    pInd = -1 ; // goal node
  else
    pInd = nodeNum+1 ;
  
  // Get all member variable sizes
  double mC = n->GetMeanCost() ;
  double sC = n->GetSigCost() ;
  ULONG d = n->GetDepth() ;
  double h = n->GetHeuristic() ;
  double mCTG = n->GetMeanCTG() ;
  double sCTG = n->GetSigCTG() ;
  
  // Convert all to const char *
  char * p_nodeNum = reinterpret_cast<char *>(&nodeNum) ;
  char * p_vInd = reinterpret_cast<char *>(&vInd) ;
  char * p_pInd = reinterpret_cast<char *>(&pInd) ;
  char * p_mC = reinterpret_cast<char *>(&mC) ;
  char * p_sC = reinterpret_cast<char *>(&sC) ;
  char * p_d = reinterpret_cast<char *>(&d) ;
  char * p_h = reinterpret_cast<char *>(&h) ;
  char * p_mCTG = reinterpret_cast<char *>(&mCTG) ;
  char * p_sCTG = reinterpret_cast<char *>(&sCTG) ;
  
  // Write all variables to binary
  fs.write(p_nodeNum,sizeof(nodeNum)) ;
  fs.write(p_vInd,sizeof(vInd)) ;
  fs.write(p_pInd,sizeof(pInd)) ;
  fs.write(p_mC,sizeof(mC)) ;
  fs.write(p_sC,sizeof(sC)) ;
  fs.write(p_d,sizeof(d)) ;
  fs.write(p_h,sizeof(h)) ;
  fs.write(p_mCTG,sizeof(mCTG)) ;
  fs.write(p_sCTG,sizeof(sCTG)) ;
}

void RAGS::NDSetReadBinary(ifstream &fs){
  int numPaths ;
  char * p_numPaths = reinterpret_cast<char *>(&numPaths) ;
  fs.read(p_numPaths,sizeof(numPaths)) ;
  
  int curPath = 0 ;
  while (curPath < numPaths){
    
    Node * pNode ; // track parent node
    while (true){
      int nodeNum ;
      char * p_nodeNum = reinterpret_cast<char *>(&nodeNum) ;
      fs.read(p_nodeNum,sizeof(nodeNum)) ; // current node number
  
      ULONG vInd ;
      char * p_vInd = reinterpret_cast<char *>(&vInd) ;
      fs.read(p_vInd,sizeof(vInd)) ;
      Node * n = new Node(GetVertex(vInd)) ; // Initialise node at vertex
      int pInd ;
      char * p_pInd = reinterpret_cast<char *>(&pInd) ;
      fs.read(p_pInd,sizeof(pInd)) ;
      if (pInd < 0) // goal node
        pNode = 0 ; // reset parent node
      
      // Allocate memory
      double mC, sC, mCTG, sCTG, h ;
      ULONG d  ;
      char * p_mC = reinterpret_cast<char *>(&mC) ;
      char * p_sC = reinterpret_cast<char *>(&sC) ;
      char * p_d = reinterpret_cast<char *>(&d) ;
      char * p_h = reinterpret_cast<char *>(&h) ;
      char * p_mCTG = reinterpret_cast<char *>(&mCTG) ;
      char * p_sCTG = reinterpret_cast<char *>(&sCTG) ;
      fs.read(p_mC,sizeof(mC)) ;
      fs.read(p_sC,sizeof(sC)) ;
      fs.read(p_d,sizeof(d)) ;
      fs.read(p_h,sizeof(h)) ;
      fs.read(p_mCTG,sizeof(mCTG)) ;
      fs.read(p_sCTG,sizeof(sCTG)) ;
      
      n->SetMeanCost(mC) ;
      n->SetSigCost(sC) ;
      n->SetDepth(d) ;
      n->SetHeuristic(h) ;
      n->SetMeanCTG(mCTG) ;
      n->SetSigCTG(sCTG) ;
      n->SetParent(pNode) ;
      
      pNode = n ; // assign as new parent node
      if (nodeNum == 0){
        itsNDSet.push_back(n) ;
        curPath++ ;
        break ;
      }
    }
  }
}

ULONG RAGS::GetVertexIndex(Vertex * v){
  Vertex ** vAll = itsGraph->GetVertices() ;
  ULONG numV = itsGraph->GetNumVertices() ;
  for (ULONG i = 0; i < numV; i++)
    if (v == vAll[i])
      return i ;
  std::cerr << "ERROR: Vertex not found! Exiting.\n" ;
  return 0 ;
}

Vertex * RAGS::GetVertex(ULONG i){
  Vertex ** vAll = itsGraph->GetVertices() ;
  return vAll[i] ;
}

// Function to compare vertices: returns TRUE if vertex A is better than vertex B
bool RAGS::IsABetterThanB(Vertex * A, Vertex * B)
{
  double pImprove = ComputeImprovementProbability(A,B) ;

  return (pImprove<=0.5);
}

// Function to compare vertices: returns TRUE if vertex A is better than vertex B
double RAGS::ComputeImprovementProbability(Vertex * A, Vertex * B)
{
  vector<Node *> ANodes = A->GetNodes();
  vector<Node *> BNodes = B->GetNodes();

  double c_A0 = A->GetCTC() ;
  double c_B0 = B->GetCTC() ;
  double max_3sig = ANodes[0]->GetMeanCTG() + 3*ANodes[0]->GetSigCTG() ;
  double min_3sig = ANodes[0]->GetMeanCTG() - 3*ANodes[0]->GetSigCTG() ;
  for (unsigned i = 0; i < ANodes.size(); i++)
  {
    if (max_3sig < ANodes[i]->GetMeanCTG()+3*ANodes[i]->GetSigCTG())
      max_3sig = ANodes[i]->GetMeanCTG()+3*ANodes[i]->GetSigCTG() ;
    if (min_3sig > ANodes[i]->GetMeanCTG()-3*ANodes[i]->GetSigCTG())
      min_3sig = ANodes[i]->GetMeanCTG()-3*ANodes[i]->GetSigCTG() ;
  }
  for (unsigned i = 0; i < BNodes.size(); i++)
  {
    if (max_3sig < BNodes[i]->GetMeanCTG()+3*BNodes[i]->GetSigCTG())
      max_3sig = BNodes[i]->GetMeanCTG()+3*BNodes[i]->GetSigCTG() ;
    if (min_3sig > BNodes[i]->GetMeanCTG()-3*BNodes[i]->GetSigCTG())
      min_3sig = BNodes[i]->GetMeanCTG()-3*BNodes[i]->GetSigCTG() ;
  }

  // If code is taking too long, change n to a smaller value
  // Note that since this is the numerical integration discretisation
  // smaller n will provide coarser approximation to solution
  int n = 100 ;
  vector<double> x = linspace(min_3sig,max_3sig,n) ;
  double dx = x[1]-x[0] ;
  double pImprove = 0.0 ;
  for (unsigned k = 0; k < x.size(); k++)
  {
    double p_cAi = 0.0 ;
    for (unsigned i = 0; i < ANodes.size(); i++)
    {
      double mu_Ai = ANodes[i]->GetMeanCTG() ;
      double sig_Ai = ANodes[i]->GetSigCTG() ;
      double p_cA1 = (1/(sig_Ai*sqrt(2*PI)))*exp(-(pow(x[k]-mu_Ai,2))/(2*pow(sig_Ai,2))) ;
      double p_cA2 = 1.0 ;
      for (unsigned j = 0; j < ANodes.size(); j++)
      {
        double mu_Aj = ANodes[j]->GetMeanCTG() ;
        double sig_Aj = ANodes[j]->GetSigCTG() ;
        if (j != i)
	        p_cA2 *= 0.5*erfc((x[k]-mu_Aj)/(sig_Aj*sqrt(2))) ;
      }
      p_cAi += p_cA1*p_cA2 ;
    }
    double p_cBi = 1.0 ;
    for (unsigned i = 0; i < BNodes.size(); i++)
    {
      double mu_Bi = BNodes[i]->GetMeanCTG() ;
      double sig_Bi = BNodes[i]->GetSigCTG() ;
      p_cBi *= 0.5*erfc((x[k]-(c_B0-c_A0)-mu_Bi)/(sig_Bi*sqrt(2))) ;
    }
    pImprove += (p_cAi)*(1-p_cBi)*dx ;
  }

  return pImprove ;
}

vector<double> RAGS::linspace(double a, double b, int n)
{
  vector<double> array ;
  double step = (b-a)/(n-1) ;
  while (a<=b)
  {
    array.push_back(a) ;
    a += step ;
  }
  return array ;
}

// Function to compare vertices: returns TRUE if vertex A is better than vertex B
bool RAGS::GreedyComparison(Vertex * A, Vertex * B)
{
  return (A->GetCTC() < B->GetCTC()) ;
}

#endif // RAGS_H_
