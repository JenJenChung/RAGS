#include <iostream> // std::cout
#include <utility> // std::pair
#include <fstream> // std::ifstream, std::ofstream
#include <sstream> // std::stringstream
#include <string> // std::string
#include <random> // std::default_random_engine, std::uniform_real_distribution
#include <chrono> // std::chrono_system_clock::now
#include <algorithm> // std::count, std::find
#include <time.h> // clock, clock_t, CLOCKS_PER_SEC
#include <math.h> // pow, sqrt
#include <stdlib.h> // atof, atol, rand
#include "RAGS.h"
#include "XY.h"

using namespace easymath ;
using std::cout ;
using std::cin ;
using std::pair ;
using std::ifstream ;
using std::ofstream ;
using std::stringstream ;
using std::string ;
using std::default_random_engine ;
using std::uniform_real_distribution ;
using std::normal_distribution ;
using std::count ;
using std::find ;

typedef pair<int, int> edge ;
typedef unsigned long int ULONG ;

int main(){
  int trialNum ;
	cout << "Trial number: " ;
	cin >> trialNum ;
	
	// Initialising graph parameters *****************************************************************
	// Load vertex locations from txt file
  cout << "Reading vertices from file..." ;
  ifstream verticesFile("test_config/vertices0.txt") ;
	vector<XY> vertices ;
	vector<double> v(2) ;
	string line ;
	while (getline(verticesFile,line))
	{
		stringstream lineStream(line) ;
		string cell ;
		int i = 0 ;
		while (getline(lineStream,cell,','))
		{
			v[i++] = atof(cell.c_str()) ;
		}
		vertices.push_back(XY(v[0],v[1])) ;
	}
	cout << "complete.\n" ;
	
	// Load edge connections from txt file
  cout << "Reading edges from file..." ;
  ifstream edgesFile("test_config/edges0.txt") ;
	vector<edge> edges ;
	vector<long int> e ;
	while (getline(edgesFile,line))
	{
		stringstream lineStream(line) ;
		string cell ;
		e.clear() ;
		while (getline(lineStream,cell,','))
			e.push_back(atol(cell.c_str())) ;
		edge temp(e[0],e[1]) ;
		edges.push_back(temp) ;
	}
	cout << "complete.\n" ;
	
	// Randomly generate edge cost distributions
	cout << "Generating edge cost distribution parameter values..." ;
  // Write to txt file
  stringstream cdFileName ;
  cdFileName << "test_config/cost_distributions" << trialNum << ".txt" ;
  ofstream costsFile ;
  costsFile.open(cdFileName.str().c_str()) ;
  
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  default_random_engine generator(seed);
  std::uniform_real_distribution<double> mean_distribution(0.0,100.0);
  std::uniform_real_distribution<double> std_distribution(0.0,10.0);
  
  vector< vector<double> > cost_distributions ;
  for (ULONG i = 0; i < edges.size(); i++){
    vector<double> cost ;
    double diffx = vertices[edges[i].first].x - vertices[edges[i].second].x ;
    double diffy = vertices[edges[i].first].y - vertices[edges[i].second].y ;
    double dist = sqrt(pow(diffx,2)+pow(diffy,2)) ;
    bool repeat = true ;
    while (repeat){
      cost.clear() ;
      cost.push_back(mean_distribution(generator)) ;
      cost.push_back(std_distribution(generator)) ;
      if (cost[1] < cost[0]){ // do not allow std > mean
        repeat = false ;
        cost[0] += dist ;
      }
    }
    cost_distributions.push_back(cost) ;
    costsFile << cost[0] << "," << cost[1] << "\n" ;
  }
	costsFile.close() ;
	cout << "complete.\n" ;
	// END: Initialising graph parameters ************************************************************
	
	// Writing configuration to file *****************************************************************
	// Write true edge costs to file
  stringstream tcFileName ;
  tcFileName << "test_config/true_costs" << trialNum << ".txt" ;
  ofstream trueCostsFile ;
  trueCostsFile.open(tcFileName.str().c_str()) ;
    
  // Query for true edge costs and write to txt file
  vector<double> true_costs ;
  for (ULONG i = 0; i < edges.size(); i++){
    normal_distribution<double> distribution(cost_distributions[i][0], cost_distributions[i][1]) ;
    double c ;
    while (true){
      c = distribution(generator) ;
      if (c > 0) // do not allow negative costs
        break ;
    }
    true_costs.push_back(c) ;
    trueCostsFile << true_costs[i] << "\n" ;
  }
  trueCostsFile << "\n" ;
	trueCostsFile.close() ;
	// END: Writing configuration to file ************************************************************
	
	// Define planning task
//	XY start = vertices[3] ; // with UTM graph: vertices.txt, edges.txt
//	XY goal = vertices[14] ; // with UTM graph: vertices.txt, edges.txt
	XY start = vertices[0] ; // with random 100 graph: vertices0.txt, edges0.txt
	XY goal = vertices[99] ; // with random 100 graph: vertices0.txt, edges0.txt
	
	// RAGS planner **********************************************************************************
	cout << "***** RAGS planner *****\n" ;
	// Create RAGS object
	RAGS * testRAGS = new RAGS(vertices, edges, cost_distributions) ;
	
	// Write true path costs to file
  stringstream pcRAGSFileName ;
  pcRAGSFileName << "test_config/path_cost_RAGS" << trialNum << ".txt" ;
  ofstream RAGSPathCostFile ;
  RAGSPathCostFile.open(pcRAGSFileName.str().c_str()) ;
  
  // Initialise current location
	XY curLoc = start ;
	
  // Execute RAGS path
	double cumulativeCost = 0 ;
  clock_t t_start = clock() ;
	while (true){
	  cout << "Current location: (" << curLoc.x << "," << curLoc.y << ")..." ;
    cout << "ND-set size: " << testRAGS->GetNDSetSize() << "..." ;
	  if (curLoc == goal){ // Exit loop if goal is reached
	    cout << "Goal reached!\n" ;
	    break ;
    }
    
    // Execute RAGS transition
    XY nextLoc = testRAGS->SearchGraph(curLoc,goal,true_costs) ;
    
    // Log transition cost
    int i = testRAGS->GetEdgeIndex(curLoc, nextLoc) ;
    cumulativeCost += true_costs[i] ;
    RAGSPathCostFile << true_costs[i] << "," << cumulativeCost << "\n" ;
    
    cout << "transitioning to next location...\n" ;
    curLoc = nextLoc ;
	}
  double rags_time = (double)(clock() - t_start)/CLOCKS_PER_SEC ; // log RAGS computation time
  cout << "RAGS planning and execution time: " << rags_time << "s.\n" ;
	
	delete testRAGS ;
	
	RAGSPathCostFile.close() ;
	// END: RAGS planner *****************************************************************************
	
	// Greedy planner ********************************************************************************
	cout << "***** Greedy planner *****\n" ;
	// Create RAGS object
	RAGS testGreedy(vertices, edges, cost_distributions) ;
	
	// Write true path costs to file
  stringstream pcGreedyFileName ;
  pcGreedyFileName << "test_config/path_cost_Greedy" << trialNum << ".txt" ;
  ofstream GreedyPathCostFile ;
  GreedyPathCostFile.open(pcGreedyFileName.str().c_str()) ;
  
  // Initialise current location
	curLoc = start ;
	
	// Execute greedy path
	cumulativeCost = 0 ;
	while (true){
	  cout << "Current location: (" << curLoc.x << "," << curLoc.y << ")..." ;
    cout << "ND-set size: " << testGreedy.GetNDSetSize() << "..." ;
	  if (curLoc == goal){ // Exit loop if goal is reached
	    cout << "Goal reached!\n" ;
	    break ;
    }
    
    // Execute greedy transition
    XY nextLoc = testGreedy.SearchGraphGreedy(curLoc,goal,true_costs) ;
    
    // Log transition cost
    int i = testGreedy.GetEdgeIndex(curLoc, nextLoc) ;
    cumulativeCost += true_costs[i] ;
    GreedyPathCostFile << true_costs[i] << "," << cumulativeCost << "\n" ;
    
    cout << "transitioning to next location...\n" ;
    curLoc = nextLoc ;
  }
	  
	GreedyPathCostFile.close() ;
	// END: Greedy planner ***************************************************************************
	
	// Astar planner *********************************************************************************
	cout << "***** A* planner *****\n" ;
	
	// For A* search
	vector< vector<double> > Astar_costs = cost_distributions ;
	for (ULONG i = 0; i < Astar_costs.size(); i++)
	  Astar_costs[i][1] = 0.0 ; // standard deviation not considered in Astar planner
	
	// Create RAGS object
	pathOut pOut = BEST ;
	RAGS testAstar(vertices, edges, Astar_costs, pOut) ;
	
	// Write true path costs to file
  stringstream pcAstarFileName ;
  pcAstarFileName << "test_config/path_cost_Astar" << trialNum << ".txt" ;
  ofstream AstarPathCostFile ;
  AstarPathCostFile.open(pcAstarFileName.str().c_str()) ;
  
  // Reset current location
  curLoc = start ;
  
  // Execute A* path
  cumulativeCost = 0 ;
	while (true){
	  cout << "Current location: (" << curLoc.x << "," << curLoc.y << ")..." ;
	  if (curLoc == goal){ // Exit loop if goal is reached
	    cout << "Goal reached!\n" ;
	    break ;
    }
    
    // Execute A* transition
    XY nextLoc = testAstar.SearchGraph(curLoc,goal,true_costs) ;
    
    // Log transition cost
    int i = testAstar.GetEdgeIndex(curLoc, nextLoc) ;
    cumulativeCost += true_costs[i] ;
    AstarPathCostFile << true_costs[i] << "," << cumulativeCost << "\n" ;
    
    cout << "transitioning to next location...\n" ;
    curLoc = nextLoc ;
	}
	
	AstarPathCostFile.close() ;
	// END: Astar planner ****************************************************************************
	
	// Sampled Astar planner *************************************************************************
	cout << "***** Sampled A* planner *****\n" ;
	
	// Write true path costs to file
  stringstream pcSampledAstarFileName ;
  pcSampledAstarFileName << "test_config/path_cost_SampledAstar" << trialNum << ".txt" ;
  ofstream SampledAstarPathCostFile ;
  SampledAstarPathCostFile.open(pcSampledAstarFileName.str().c_str()) ;
  
  // Manage sampling time
  t_start = clock() ; // reset clock timer
	double t_elapse = 0.0 ;
	
  // Log sampled paths
  vector< vector<XY> > sampledPaths ;
  
  int k = 0 ;
  while (t_elapse <= rags_time){
    // Sample edge costs
    vector< vector<double> > sampled_costs_with_std ;
    vector<double> sampled_costs ;
    for (ULONG i = 0; i < edges.size(); i++){
      vector<double> cc ;
      normal_distribution<double> distribution(cost_distributions[i][0], cost_distributions[i][1]) ;
      double c ;
      while (true){
        c = distribution(generator) ;
        if (c > 0)
          break ;
      }
      sampled_costs.push_back(c) ;
      cc.push_back(c) ;
      cc.push_back(0.0) ;
      sampled_costs_with_std.push_back(cc) ; // for RAGS object initialisation
    }
    
    // Create RAGS object
    RAGS sampledAstar(vertices, edges, sampled_costs_with_std, pOut) ;
    
    // Reset current location
    curLoc = start ;
    
    // Log current path
    vector<XY> curPath ;
    
    // Execute sampled A* path
	  while (true){
      curPath.push_back(curLoc) ;
	    if (curLoc == goal)
	      break ;
      // Execute A* transition
      XY nextLoc = sampledAstar.SearchGraph(curLoc,goal,sampled_costs) ;
      curLoc = nextLoc ;
	  }
	  sampledPaths.push_back(curPath) ;
    t_elapse = (double)(clock() - t_start)/CLOCKS_PER_SEC ;
    k++ ;
  }
  
  // Compute most frequent path
  vector<int> path_set ;
  int max_count = 0;
  for(ULONG i = 0 ; i < sampledPaths.size(); i++){
    int mycount = count(sampledPaths.begin(), sampledPaths.end(), sampledPaths[i]);
    if(mycount > max_count){
      path_set.clear() ;
      path_set.push_back(i);
      max_count = mycount;
    }
    else if (mycount == max_count){
      path_set.push_back(i) ;
    }
  }
  cout << "Most frequent path was traversed  " << max_count 
    << " times out of " << k << " total samplings.\n" ;
  
  // If multiple paths are sampled the same number of times, pick a random path from the subset
  int path ;
  if (path_set.size() > 1)
    path = path_set[rand() % path_set.size()] ;
  else
    path = path_set[0] ;
  
  // Execute sampled A* path
  cumulativeCost = 0 ;
  for (ULONG i = 0; i < sampledPaths[path].size(); i++){
    curLoc = sampledPaths[path][i] ;
    XY nextLoc ;
    
	  cout << "Current location: (" << curLoc.x << "," << curLoc.y << ")..." ;
	  if (curLoc == goal)
	    cout << "Goal reached!\n" ;
    else {
      nextLoc = sampledPaths[path][i+1] ;
      if (i == sampledPaths[path].size()-1)
        cout << "nextLoc: (" << nextLoc.x << "," << nextLoc.y << ")\n" ;
      int j = testAstar.GetEdgeIndex(curLoc, nextLoc) ; // borrow A* RAGS object for edge indices
    
      cumulativeCost += true_costs[j] ;
      SampledAstarPathCostFile << true_costs[j] << "," << cumulativeCost << "\n" ;
    
      cout << "transitioning to next location...\n" ;
      curLoc = nextLoc ;
    }
	}
	
	SampledAstarPathCostFile.close() ;
	// END: Sampled Astar planner ********************************************************************
	
	// Hindsight optimal plan ************************************************************************
	cout << "***** Hindsight optimal plan *****\n" ;
	
	// For A* search
	vector< vector<double> > true_costs_with_std ;
	for (ULONG i = 0; i < true_costs.size(); i++){
	  vector<double> c ;
	  c.push_back(true_costs[i]) ;
	  c.push_back(0.0) ; // standard deviation not considered in Astar planner
	  true_costs_with_std.push_back(c) ;
  }
  
	// Create RAGS object
	RAGS testOptimal(vertices, edges, true_costs_with_std, pOut) ;
	
	// Write true path costs to file
  stringstream optimalFileName ;
  optimalFileName << "test_config/path_cost_Optimal" << trialNum << ".txt" ;
  ofstream optimalPathCostFile ;
  optimalPathCostFile.open(optimalFileName.str().c_str()) ;
  
  // Reset current location
  curLoc = start ;
  
  // Execute hindsight optimal path
  cumulativeCost = 0 ;
	while (true){
	  cout << "Current location: (" << curLoc.x << "," << curLoc.y << ")..." ;
	  if (curLoc == goal){ // Exit loop if goal is reached
	    cout << "Goal reached!\n" ;
	    break ;
    }
    
    // Execute A* transition
    XY nextLoc = testOptimal.SearchGraph(curLoc,goal,true_costs) ;
    
    // Log transition cost
    int i = testOptimal.GetEdgeIndex(curLoc, nextLoc) ;
    cumulativeCost += true_costs[i] ;
    optimalPathCostFile << true_costs[i] << "," << cumulativeCost << "\n" ;
    
    cout << "transitioning to next location...\n" ;
    curLoc = nextLoc ;
	}
	
	optimalPathCostFile.close() ;
	// END: Hindsight optimal plan *******************************************************************
	
	return 0 ;
}
