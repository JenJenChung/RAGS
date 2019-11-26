#ifndef GRAPH_GEN_H_
#define GRAPH_GEN_H_

double EuclideanDistance(vector<double> v1, vector<double> v2)
{
	double diff_x = v1[0] - v2[0] ;
	double diff_y = v1[1] - v2[1] ;
	double diff = sqrt(pow(diff_x,2) + pow(diff_y,2)) ;
	
	return diff ;
}

// Connect vertices within specified radius
void RadiusConnect(char * cDir, int graphNum, vector< vector<double> > vertices, double radius, int meanMax, int sigMax)
{
	srand(graphNum+1);
	vector< vector<double> > edges(pow(vertices.size(),2), vector<double>(4)) ;
	ULONG k = 0 ;
	
	for (ULONG i = 0; i < (ULONG)vertices.size(); i++)
	{
		for (ULONG j = 0; j < (ULONG)vertices.size(); j++)
		{
			double diff = EuclideanDistance(vertices[i], vertices[j]) ;
			
			if (diff <= radius && i != j)
			{
				edges[k][0] = (double)i ;
				edges[k][1] = (double)j ;
				edges[k][2] = diff + (double)(rand() % (meanMax*100))/100.0 ;
				edges[k][3] = (double)(rand() % (sigMax*100))/100.0 ; // 0.0 ; *** EDGE COST STANDARD DEVIATION ***
				k++ ;
			}
		}
	}
	
	edges.resize(k) ;
	
	// Write edges to txt file
	stringstream eFileName ;
	eFileName << cDir << "/edges" << graphNum << ".txt" ;
	
	ofstream edgesFile ;
	edgesFile.open(eFileName.str().c_str()) ;
	
	for (ULONG i = 0; i < edges.size(); i++)
	{
		edgesFile << edges[i][0] << "," << edges[i][1] << "," << edges[i][2] << "," << edges[i][3] << "\n" ;
	}
	edgesFile.close() ;
}

void GenerateVertices(char * cDir, int graphNum, int numVerts, double x, double y, double &radius, vector< vector<double> > &vertices){
	srand (graphNum+1);
	double vertx, verty;
	int xx = x;
	int yy = y;
  
  int i = 0 ;
  while (i < numVerts){
		bool fDbl = false ;
		if (i == 0)
		{
			vertices[i][0] = 0 ;
			vertices[i][1] = 0 ;
		}
		else if (i == numVerts-1)
		{
			vertices[i][0] = x ;
			vertices[i][1] = y ;
		}
		else
		{
			vertx = rand() % xx;
			verty = rand() % yy;
			if ((vertx == 0 && verty == 0) || (vertx == x && verty == y)){
			  fDbl = true ;
			  break ;
		  }
		  else {
			  for (int j = 0; j < i; j++){
			    if (vertx == vertices[j][0] && verty == vertices[j][1]){
			      fDbl = true ;
			      break ;
		      }
	      }
      }
	    if (!fDbl){ // only add to list if unique
			  vertices[i][0] = vertx ;
			  vertices[i][1] = verty ;
		  }
		}
		
		if (!fDbl)
		  i++ ;
	}

	// Write vertices to txt file
	stringstream vFileName ;
	vFileName << cDir << "/vertices" << graphNum << ".txt" ;

	ofstream vertsFile ;
	vertsFile.open(vFileName.str().c_str()) ;

	for (ULONG i = 0; i < vertices.size(); i++)
	{
		vertsFile << vertices[i][0] << "," << vertices[i][1] << "\n" ;
	}
	vertsFile.close() ;
	
	radius = sqrt((6.0/PI)*x*y*(log((double)numVerts)/(double)numVerts)) ;
}

void GenerateGraph(char * cDir, int graphNum, int numVerts, double x, double y, int meanMax, int sigMax){
//	cout << "Generating random vertices in " << x << " by " << y << "\n" ;
	double radius ;
  vector< vector< double > > vertices(numVerts, vector<double>(2));

  GenerateVertices(cDir, graphNum, numVerts, x, y, radius, vertices) ;
//	cout << "Connecting with radius " << radius << endl;
	RadiusConnect(cDir, graphNum, vertices, radius, meanMax, sigMax);
}

#endif // GRAPH_GEN_H_
