#ifndef PTHRESH
#define PTHRESH 0.60
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
  
  // Initialising graph parameters *****************************************************************
  // Load vertex locations from txt file
  cout << "Reading vertices from file: " ;
  char vFile[buffSize] ;
  sprintf(vFile,"%s/vertices%d.txt",configDir,trialNum) ;
  ifstream verticesFile(vFile) ;
  cout << vFile << "..." ;
  if (!verticesFile.is_open()){
    cout << "\nFile: " << vFile << " not found, skipping.\n" ;
    exit(1) ;
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
    exit(1) ;
  }
  vector<edge> edges ;
  vector<double> e ;
  vector< vector<double> > cost_distributions ;
  vector<double> c(2) ;
  while (getline(edgesFile,line))
  {
    stringstream lineStream(line) ;
    string cell ;
    e.clear() ;
    while (getline(lineStream,cell,','))
	    e.push_back(atof(cell.c_str())) ;
    edge temp((int)e[0],(int)e[1]) ;
    c[0] = e[2] ;
    c[1] = e[3] ;
    edges.push_back(temp) ;
    cost_distributions.push_back(c) ;
  }
  cout << "complete.\n" ;

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  default_random_engine generator(seed);
  // END: Initialising graph parameters ************************************************************
  
  // START: Compute RAGS ND-set ********************************************************************
  // Define planning task
  XY start = vertices[0] ; // with random graph: vertices0.txt, edges0.txt
  XY goal = vertices[vertices.size()-1] ; // with random graph: vertices0.txt, edges0.txt
  
  // Write ND set to file
  stringstream ndFileName ;
  ndFileName << fileDir << "/ND_set" << trialNum << ".txt" ;
  ofstream ndSetFile ;
  ndSetFile.open(ndFileName.str().c_str(),std::ifstream::app) ;

  // Create RAGS object
  RAGS * testRAGS = new RAGS(vertices, edges, cost_distributions) ;
  
  // Compute ND set
  XY s ;
  testRAGS->InitialiseNDSet(start,goal,s) ;
  vector<Node *> NDSet = testRAGS->GetNDSet() ;
  
  for (size_t i = 0; i < NDSet.size(); i++){
    ndSetFile << NDSet[i]->GetVertex()->GetX() << "," << NDSet[i]->GetVertex()->GetY() << "\n" ;
    Node * n = NDSet[i]->GetParent() ;
    while (n){
      ndSetFile << n->GetVertex()->GetX() << "," << n->GetVertex()->GetY() << "\n" ;
      n = n->GetParent() ;
    }
  }
  
  ndSetFile.close() ;
  
//  // Display ND set
//  for (size_t i = 0; i < testRAGS->GetNDSetSize(); i++)
//    testRAGS->GetNDSet()[i]->DisplayPath() ;
  
  // Write ND set to binary file
  cout << "Writing ND-set to binaries...\n" ;
  stringstream ndBinFileName ;
  ndBinFileName << fileDir << "/ND_set" << trialNum << ".dat" ;
  ofstream binaryNDSetFile ;
  binaryNDSetFile.open(ndBinFileName.str().c_str(), std::ios::out | std::ios::binary) ;
  testRAGS->NDSetWriteBinary(binaryNDSetFile) ;
  binaryNDSetFile.close() ;
  cout << "Writing complete.\n" ;
  
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
  }
  
  // Write RAGS path file
  cout << "Executing RAGS path on original RAGS object...\n" ;
  stringstream pFileName ;
  pFileName << fileDir << "/RAGS_path" << trialNum << ".txt" ;
  ofstream pFile ;
  pFile.open(pFileName.str().c_str(),std::ifstream::app) ;
  
  // Initialise current location
  XY curLoc = start ;

  // Execute RAGS path
  while (true){
    pFile << curLoc.x << "," << curLoc.y << "\n" ;
    cout << "Current vertex: (" << curLoc.x << "," << curLoc.y << ")\n" ;
    if (curLoc == goal){
      break ;
    }
    
    // Execute RAGS transition
    XY nextLoc = testRAGS->SearchGraph(curLoc,goal,true_costs) ;
    
    // Check transition was successful
    if (curLoc == nextLoc){
      cout << "Transition unsuccessful! Graph #" << trialNum << "\n" ;
      exit(1) ;
    }
    
    curLoc = nextLoc ;
  }
  
  pFile.close() ;

  delete testRAGS ;
  
  // Create new RAGS object and read in stored ND set
  cout << "Reading in ND-set to new RAGS object from binary files...\n" ;
  std::ifstream readNDSetFile ;
  readNDSetFile.open(ndBinFileName.str().c_str(), std::ios::in | std::ios::binary) ;
  
  RAGS * newRAGS = new RAGS(vertices, edges, cost_distributions) ;
  newRAGS->NDSetReadBinary(readNDSetFile) ; // read in ND set from binary file and write to itsNDSet
  readNDSetFile.close() ;
  cout << "Reading complete.\n" ;
  
  // Write new RAGS path file
  cout << "Executing RAGS path on new RAGS object...\n" ;
  stringstream npFileName ;
  npFileName << fileDir << "/RAGS_path_new" << trialNum << ".txt" ;
  ofstream npFile ;
  npFile.open(npFileName.str().c_str(),std::ifstream::app) ;
  
//  // Display ND set
//  for (size_t i = 0; i < newRAGS->GetNDSetSize(); i++)
//    newRAGS->GetNDSet()[i]->DisplayPath() ;
  
  // Initialise current location
  curLoc = start ;
  newRAGS->SetInitialVert(start) ;
  
  // Execute RAGS path
  while (true){
    npFile << curLoc.x << "," << curLoc.y << "\n" ;
    cout << "Current vertex: (" << curLoc.x << "," << curLoc.y << ")\n" ;
    if (curLoc == goal){
      break ;
    }
    
    // Execute RAGS transition
    XY nextLoc = newRAGS->SearchGraph(curLoc,goal,true_costs) ;
    
    // Check transition was successful
    if (curLoc == nextLoc){
      cout << "Transition unsuccessful! Graph #" << trialNum << "\n" ;
      exit(1) ;
    }
    
    curLoc = nextLoc ;
  }
  
  npFile.close() ;

  delete newRAGS ;
  

  // END: Compute RAGS ND-set **********************************************************************
    
  return 0 ;
}
