#ifndef EXECUTE_PLANNERS_H_
#define EXECUTE_PLANNERS_H_
/* Function to execute RAGS, greedy, A* and sampled A*
Logs path data and computation times
Compares against the hindsight optimal path */

void ExecutePlanners(char * fileDir, int trialNum, vector<XY> vertices, vector<edge> edges, vector< vector<double> > cost_distributions, XY start, XY goal, default_random_engine generator, size_t loops){
  // START: Initialising path log files ************************************************************
  // Write true edge costs to file
  stringstream tcFileName ;
  tcFileName << fileDir << "/true_costs" << trialNum << ".txt" ;
  ofstream trueCostsFile ;
  trueCostsFile.open(tcFileName.str().c_str(),std::ifstream::app) ;
  
  // Write planning data to txt file
  stringstream pcFileName ;
  pcFileName << fileDir << "/path_costs" << trialNum << ".txt" ;
  ofstream costFile ;
  costFile.open(pcFileName.str().c_str()) ;
  
  stringstream tFileName ;
  tFileName << fileDir << "/computation_times" << trialNum << ".txt" ;
  ofstream timeFile ;
  timeFile.open(tFileName.str().c_str()) ;
  
  if (!costFile.is_open() || !timeFile.is_open()){
    cout << "Error opening file/s! Exiting.\n" ;
    return ;
  }
  // END: Initialising log files *******************************************************************
    
	size_t i = 0 ;
	while (i < loops){
	  // START: Writing configuration to file ********************************************************
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
      trueCostsFile << true_costs[i] << "," ;
    }
    trueCostsFile << "\n" ;
	  // END: Writing configuration to file **********************************************************

    // RAGS planner ********************************************************************************
//	  cout << "***** RAGS planner *****\n" ;
	  // Create RAGS object
	  RAGS * testRAGS = new RAGS(vertices, edges, cost_distributions) ;
    
    // Initialise current location
	  XY curLoc = start ;
	
    // Execute RAGS path
	  double cumulativeCost = 0 ;
    clock_t t_start = clock() ;
	  while (true){
//	    cout << "Current location: (" << curLoc.x << "," << curLoc.y << ")..." ;
//      cout << "ND-set size: " << testRAGS->GetNDSetSize() << "..." ;
	    if (curLoc == goal){ // Exit loop if goal is reached
//	      cout << "Goal reached!\n" ;
	      break ;
      }
      
      // Execute RAGS transition
      XY nextLoc = testRAGS->SearchGraph(curLoc,goal,true_costs) ;
      
      // Check transition was successful
      if (curLoc == nextLoc){
        cout << "Transition unsuccessful! Graph #" << trialNum << "\n" ;
        exit(1) ;
      }
      
      // Accumulate transition cost
      int i = testRAGS->GetEdgeIndex(curLoc, nextLoc) ;
      cumulativeCost += true_costs[i] ;
      
//      cout << "transitioning to next location...\n" ;
      curLoc = nextLoc ;
	  }
    double rags_time = (double)(clock() - t_start)/CLOCKS_PER_SEC ; // log RAGS computation time
//    cout << "RAGS planning and execution time: " << rags_time << "s.\n" ;
    timeFile << rags_time << "," ;
    costFile << cumulativeCost << "," ;
	
	  delete testRAGS ;
	
	  // END: RAGS planner ***************************************************************************
	
	  // Greedy planner ******************************************************************************
//	  cout << "***** Greedy planner *****\n" ;
	  // Create RAGS object
	  RAGS * testGreedy = new RAGS(vertices, edges, cost_distributions) ;
    
    // Initialise current location
	  curLoc = start ;
	
	  // Execute greedy path
	  cumulativeCost = 0 ;
    t_start = clock() ; // reset clock timer
	  while (true){
//	    cout << "Current location: (" << curLoc.x << "," << curLoc.y << ")..." ;
//      cout << "ND-set size: " << testGreedy->GetNDSetSize() << "..." ;
	    if (curLoc == goal){ // Exit loop if goal is reached
//	      cout << "Goal reached!\n" ;
	      break ;
      }
      
      // Execute greedy transition
      XY nextLoc = testGreedy->SearchGraphGreedy(curLoc,goal,true_costs) ;
      
      // Check transition was successful
      if (curLoc == nextLoc){
        cout << "Transition unsuccessful! Graph #" << trialNum << "\n" ;
        exit(1) ;
      }
      
      // Accumulate transition cost
      int i = testGreedy->GetEdgeIndex(curLoc, nextLoc) ;
      cumulativeCost += true_costs[i] ;
      
//      cout << "transitioning to next location...\n" ;
      curLoc = nextLoc ;
    }
    double greedy_time = (double)(clock() - t_start)/CLOCKS_PER_SEC ; // log greedy computation time
    timeFile << greedy_time << "," ;
    costFile << cumulativeCost << "," ;
	
	  delete testGreedy ;
    
	  // END: Greedy planner *************************************************************************
//	
//	  // Astar planner *******************************************************************************
////	  cout << "***** A* planner *****\n" ;
//	
//	  // For A* search
//	  vector< vector<double> > Astar_costs = cost_distributions ;
//	  for (ULONG i = 0; i < Astar_costs.size(); i++)
//	    Astar_costs[i][1] = 0.0 ; // variance not considered in Astar planner
//	
//	  // Create RAGS object
	  pathOut pOut = BEST ;
//	  RAGS * testAstar = new RAGS(vertices, edges, Astar_costs, pOut) ;
//    
//    // Reset current location
//    curLoc = start ;
//    
//    // Execute A* path
//    cumulativeCost = 0 ;
//    t_start = clock() ; // reset clock timer
//	  while (true){
////	    cout << "Current location: (" << curLoc.x << "," << curLoc.y << ")..." ;
//	    if (curLoc == goal){ // Exit loop if goal is reached
////	      cout << "Goal reached!\n" ;
//	      break ;
//      }
//      
//      // Execute A* transition
//      XY nextLoc = testAstar->SearchGraph(curLoc,goal,true_costs) ;
//      
//      // Check transition was successful
//      if (curLoc == nextLoc){
//        cout << "Transition unsuccessful! Graph #" << trialNum << "\n" ;
//        exit(1) ;
//      }
//      
//      // Accumulate transition cost
//      int i = testAstar->GetEdgeIndex(curLoc, nextLoc) ;
//      cumulativeCost += true_costs[i] ;
//      
////      cout << "transitioning to next location...\n" ;
//      curLoc = nextLoc ;
//	  }
//    double astar_time = (double)(clock() - t_start)/CLOCKS_PER_SEC ; // log A* computation time
//    timeFile << astar_time << "," ;
//    costFile << cumulativeCost << "," ;
//	
//	  // END: Astar planner **************************************************************************
//	
//	  // Sampled Astar planner ***********************************************************************
////	  cout << "***** Sampled A* planner *****\n" ;
//    
//    // Manage sampling time
//    t_start = clock() ; // reset clock timer
//	  double t_elapse = 0.0 ;
//	
//    // Log sampled paths
//    vector< vector<XY> > sampledPaths ;
//    
//    int k = 0 ;
//    RAGS * sampledAstar ;
//    while (t_elapse <= rags_time){
//      // Sample edge costs
//      vector< vector<double> > sampled_costs_with_std ;
//      vector<double> sampled_costs ;
//      for (ULONG i = 0; i < edges.size(); i++){
//        vector<double> cc ;
//        normal_distribution<double> distribution(cost_distributions[i][0], cost_distributions[i][1]) ;
//        double c ;
//        while (true){
//          c = distribution(generator) ;
//          if (c > 0)
//            break ;
//        }
//        sampled_costs.push_back(c) ;
//        cc.push_back(c) ;
//        cc.push_back(0.0) ;
//        sampled_costs_with_std.push_back(cc) ; // for RAGS object initialisation
//      }
//      
//      // Create RAGS object
//      sampledAstar = new RAGS(vertices, edges, sampled_costs_with_std, pOut) ;
//      
//      // Reset current location
//      curLoc = start ;
//      
//      // Log current path
//      vector<XY> curPath ;
//      
//      // Execute sampled A* path
//	    while (true){
//        curPath.push_back(curLoc) ;
//	      if (curLoc == goal)
//	        break ;
//        // Execute A* transition
//        XY nextLoc = sampledAstar->SearchGraph(curLoc,goal,sampled_costs) ;
//      
//        // Check transition was successful
//        if (curLoc == nextLoc){
//          cout << "Transition unsuccessful! Graph #" << trialNum << "\n" ;
//          exit(1) ;
//        }
//        curLoc = nextLoc ;
//	    }
//	    sampledPaths.push_back(curPath) ;
//      t_elapse = (double)(clock() - t_start)/CLOCKS_PER_SEC ;
//      k++ ;
//      delete sampledAstar ;
//    }
//    
//    // Compute most frequent path
//    vector<int> path_set ;
//    int max_count = 0;
//    for(ULONG i = 0 ; i < sampledPaths.size(); i++){
//      int mycount = count(sampledPaths.begin(), sampledPaths.end(), sampledPaths[i]);
//      if(mycount > max_count){
//        path_set.clear() ;
//        path_set.push_back(i);
//        max_count = mycount;
//      }
//      else if (mycount == max_count){
//        path_set.push_back(i) ;
//      }
//    }
////    cout << "Most frequent path was traversed  " << max_count 
////      << " times out of " << k << " total samplings.\n" ;
//    timeFile << k << "\n" ; // log total A* samplings
//    
//    // If multiple paths are sampled the same number of times, pick a random path from the subset
//    int path ;
//    if (path_set.size() > 1)
//      path = path_set[rand() % path_set.size()] ;
//    else
//      path = path_set[0] ;
//    
//    // Execute sampled A* path
//    cumulativeCost = 0 ;
//    for (ULONG i = 0; i < sampledPaths[path].size(); i++){
//      curLoc = sampledPaths[path][i] ;
//      XY nextLoc ;
//      
////	    cout << "Current location: (" << curLoc.x << "," << curLoc.y << ")..." ;
//	    if (curLoc == goal){
////	      cout << "Goal reached!\n" ;
//      }
//      else {
//        nextLoc = sampledPaths[path][i+1] ;
//        if (i == sampledPaths[path].size()-1){
////          cout << "nextLoc: (" << nextLoc.x << "," << nextLoc.y << ")\n" ;
//        }
//        int j = testAstar->GetEdgeIndex(curLoc, nextLoc) ; // borrow A* RAGS object for edge indices
//      
//        cumulativeCost += true_costs[j] ;
//      
////        cout << "transitioning to next location...\n" ;
//        curLoc = nextLoc ;
//      }
//	  }
//    costFile << cumulativeCost << "," ;
//	
//	  delete testAstar ;
//	
//	  // END: Sampled Astar planner ******************************************************************
	
	  // Hindsight optimal plan **********************************************************************
//	  cout << "***** Hindsight optimal plan *****\n" ;
	
	  // For A* search
	  vector< vector<double> > true_costs_with_std ;
	  for (ULONG i = 0; i < true_costs.size(); i++){
	    vector<double> c ;
	    c.push_back(true_costs[i]) ;
	    c.push_back(0.0) ; // variance not considered in Astar planner
	    true_costs_with_std.push_back(c) ;
    }
    
	  // Create RAGS object
	  RAGS * testOptimal = new RAGS(vertices, edges, true_costs_with_std, pOut) ;
    
    // Reset current location
    curLoc = start ;
    
    // Execute hindsight optimal path
    cumulativeCost = 0 ;
	  while (true){
//	    cout << "Current location: (" << curLoc.x << "," << curLoc.y << ")..." ;
	    if (curLoc == goal){ // Exit loop if goal is reached
//	      cout << "Goal reached!\n" ;
	      break ;
      }
      
      // Execute A* transition
      XY nextLoc = testOptimal->SearchGraph(curLoc,goal,true_costs) ;
      
      // Check transition was successful
      if (curLoc == nextLoc){
        cout << "Transition unsuccessful! Graph #" << trialNum << "\n" ;
        exit(1) ;
      }
      
      // Accumulate transition cost
      int i = testOptimal->GetEdgeIndex(curLoc, nextLoc) ;
      cumulativeCost += true_costs[i] ;
      
//      cout << "transitioning to next location...\n" ;
      curLoc = nextLoc ;
	  }
    costFile << cumulativeCost << "\n" ;
	
	  delete testOptimal ;
	
	  // END: Hindsight optimal plan *****************************************************************
	  
	  i++ ;
  }
	
  // START: Closing log files **********************************************************************
	costFile.close() ;
	timeFile.close() ;
	trueCostsFile.close() ;
  // END: Closing log files ************************************************************************
}
#endif // EXECUTE_PLANNERS_H_
