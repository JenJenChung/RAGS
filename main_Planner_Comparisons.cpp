#ifndef PTHRESH
#define PTHRESH 0.70
#endif

enum searchType {ASTAR, DIJKSTRA} ; // BREADTH, DEPTH
enum heuristic {ZERO, MANHATTAN, EUCLIDEAN, RAGSCOMPARE} ;
enum pathOut {BEST,ALL} ;

searchType SEARCH_TYPE = ASTAR ;
heuristic HEURISTIC = ZERO ;

#include <iostream> // std::cout
#include <utility> // std::pair
#include <fstream> // std::ifstream, std::ofstream
#include <sstream> // std::stringstream
#include <string> // std::string
#include <random> // std::default_random_engine, std::uniform_real_distribution
#include <chrono> // std::chrono_system_clock::now
#include <algorithm> // std::count, std::find
#include <time.h> // time, clock, clock_t, CLOCKS_PER_SEC
#include <math.h> // pow, sqrt
#include <stdlib.h> // atof, atol, rand, srand
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

#include "ExecutePlanners.h"
#include "GraphGeneration.h"

int main(){
  int numGraphs = 100 ;
  int trialNum = 0 ;
  int varMax = 20 ;
	
  int buffSize = 100 ;
  char fileDir[buffSize] ;
  sprintf(fileDir,"logs/maxVar_%d/pThresh_%.2f",varMax,PTHRESH) ;
  char configDir[buffSize] ;
  sprintf(configDir,"config_files/maxVar_%d",varMax) ;
  char mkdir[buffSize] ;
  sprintf(mkdir,"mkdir -p %s",fileDir) ;
  system(mkdir) ;
  sprintf(mkdir,"mkdir -p %s",configDir) ;
  system(mkdir) ;
  
  // Generate new graphs ***************************************************************************
  bool newGraphs = false ;
  bool newEdgeVarsOnly = false ;
  if (newGraphs){
    // Graph parameters
    int numVerts = 100 ;
    double x = 100 ;
    double y = 100 ;
    int meanMax = 100 ;
    int i = 0 ;
    while (i < numGraphs){
      if (newEdgeVarsOnly){
        char eFile[buffSize] ;
        sprintf(eFile,"config_files/maxVar_5/edges%d.txt",i) ;
        ifstream eFileRead(eFile) ;
        if (!eFileRead.is_open()){
          cout << "\nFile: " << eFile << " not found, skipping.\n" ;
          i++ ;
          continue ;
        }
        vector< vector<double> > ee ;
        vector<double> e(4) ;
        string line ;
        while (getline(eFileRead,line)){
	        stringstream lineStream(line) ;
	        string cell ;
	        int j = 0 ;
	        while (getline(lineStream,cell,','))
	        {
		        e[j++] = atof(cell.c_str()) ;
	        }
	        ee.push_back(e) ;
        }
        
	      stringstream eFileName ;
	      eFileName << configDir << "/edges" << i << ".txt" ;
	      ofstream eFileWrite ;
	      eFileWrite.open(eFileName.str().c_str()) ;
        
	      for (size_t j = 0; j < ee.size(); j++){
				  ee[j][3] = (double)(rand() % (varMax*100))/100.0 ; // 0.0 ; *** EDGE COST VARIANCE ***
				  eFileWrite << ee[j][0] << "," << ee[j][1] << "," << ee[j][2] << "," << ee[j][3] << "\n" ;
			  }
			  eFileWrite.close() ;
      }
      else{
        GenerateGraph(configDir, i, numVerts, x, y, meanMax, varMax) ;
      }
      i++ ;
    }
  }
  // END: Generate new graphs **********************************************************************
  
  while (trialNum < numGraphs){ 
    // Initialising graph parameters ***************************************************************
    // Load vertex locations from txt file
    cout << "Reading vertices from file: " ;
    char vFile[buffSize] ;
    sprintf(vFile,"%s/vertices%d.txt",configDir,trialNum) ;
    ifstream verticesFile(vFile) ;
    cout << vFile << "..." ;
    if (!verticesFile.is_open()){
      cout << "\nFile: " << vFile << " not found, skipping.\n" ;
      trialNum++ ;
      continue ;
    }
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
    cout << "Reading edges from file: " ;
    char eFile[buffSize] ;
    sprintf(eFile,"%s/edges%d.txt",configDir,trialNum) ;
    ifstream edgesFile(eFile) ;
    cout << eFile << "..." ;
    if (!edgesFile.is_open()){
      cout << "\nFile: " << eFile << " not found, skipping.\n" ;
      trialNum++ ;
      continue ;
    }
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
    cdFileName << fileDir << "/cost_distributions" << trialNum << ".txt" ;
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
    // END: Initialising graph parameters **********************************************************
    
    // START: Execute planners *********************************************************************
    // Define planning task
    XY start = vertices[0] ; // with random 100 graph: vertices0.txt, edges0.txt
    XY goal = vertices[99] ; // with random 100 graph: vertices0.txt, edges0.txt
    size_t loops = 100 ;
    ExecutePlanners(fileDir, trialNum, vertices, edges, cost_distributions, start, goal, generator, loops) ;
    // END: Execute planners ***********************************************************************
    
    trialNum++ ;
  }
  
  return 0 ;
}
