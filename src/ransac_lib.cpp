#include "ransac_lib.hpp"


vector_double bisectrixLine(vector_double l1, vector_double l2)
{
	vector_double bisectrix(3), V1(2), V2(2), VY(2);
	double a1, b1, c1, a2, b2, c2;
	double a1k, b1k, c1k, a2k, b2k, c2k, k1, k2;
	double bi1_a, bi1_b, bi1_c, bi2_a, bi2_b, bi2_c, theta1, theta2, modV1, modV2, modVY;
		
	a1 = l1[0];
	b1 = l1[1];
	c1 = l1[2];
	a2 = l2[0];
	b2 = l2[1];
	c2 = l2[2];
	
	k1 = sqrt(a1*a1 + b1*b1);
	k2 = sqrt(a2*a2 + b2*b2);
	
	a1k = a1/k1;
	b1k = b1/k1;
	c1k = c1/k1;
	a2k = a2/k2;
	b2k = b2/k2;
	c2k = c2/k2;
	
	bi1_a = (a1k - a2k);
	bi1_b = (b1k - b2k);
	bi1_c = (c1k - c2k);
	bi2_a = (a1k + a2k);
	bi2_b = (b1k + b2k);
	bi2_c = (c1k + c2k);

/*
	m1 = -(bi1_a/bi1_b);
	m2 = -(bi2_a/bi2_b);
	
	if(abs(m1)>abs(m2)){
		bisectrix[0] = bi1_a;
		bisectrix[1] = bi1_b;
		bisectrix[2] = bi1_c;
	}
	else{
		bisectrix[0] = bi2_a;
		bisectrix[1] = bi2_b;
		bisectrix[2] = bi2_c;
	}
*/

	
	// seleciona a bissetriz com menor angulo em relacao ao eixo Y
	// evitando divisoes por zero
	
	if(bi1_b != 0){
		V1[0] = 10;
		V1[1] = ((-bi1_c - V1[0]*bi1_a)/bi1_b) - (-bi1_c/bi1_b);
	}
	else if(bi1_a != 0){
		V1[1] = 10;
		V1[0] = ((-bi1_c - V1[1]*bi1_b)/bi1_a) - (-bi1_c/bi1_a);
	}
	
	if(bi2_b != 0){
		V2[0] = 10;
		V2[1] = ((-bi2_c - V2[0]*bi2_a)/bi2_b) - (-bi2_c/bi2_b);
	}
	else if(bi2_a != 0){
		V2[1] = 10;
		V2[0] = ((-bi2_c - V2[1]*bi2_b)/bi2_a) - (-bi2_c/bi2_a);
	}
	
	VY[0] = 0;
	VY[1] = 10;
	
	modV1 = sqrt(V1[0]*V1[0] + V1[1]*V1[1]);
	modV2 = sqrt(V2[0]*V2[0] + V2[1]*V2[1]);
	modVY = sqrt(VY[0]*VY[0] + VY[1]*VY[1]);
	
	theta1 = acos(abs(V1[0]*VY[0] + V1[1]*VY[1])/abs(modV1*modVY));
	theta2 = acos(abs(V2[0]*VY[0] + V2[1]*VY[1])/abs(modV2*modVY));

	if(abs(theta1)>=abs(theta2)){
		bisectrix[0] = bi1_a;
		bisectrix[1] = bi1_b;
		bisectrix[2] = bi1_c;
	}
	else{
		bisectrix[0] = bi2_a;
		bisectrix[1] = bi2_b;
		bisectrix[2] = bi2_c;
	}
  
	return bisectrix;
}

vector<float> intersectionPoint(vector<float> l1, vector<float> l2)
{
	vector<float> intersec(2);
	float x1, y1, x2, y2, x3, y3, x4, y4;
	x1 = l1[0];
	x2 = l1[1];
	y1 = l1[2];
	y2 = l1[3];
	x3 = l2[0];
	x4 = l2[1];
	y3 = l2[2];
	y4 = l2[3];

	intersec[0] = (((x1*y2-y1*x2)*(x3-x4)-(x3*y4-y3*x4)*(x1-x2))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)));
	intersec[1] = (((x1*y2-y1*x2)*(y3-y4)-(x3*y4-y3*x4)*(y1-y2))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)));

	return intersec;
}

vector_float ransac2Dline(vector_float x, vector_float y, float threshold, float p_inliers)
{
	vector<pair<size_t,TLine2D > >   detectedLines;
	const int n_inliers = int (p_inliers*x.size());
	vector_float line;

	CTicTac	tictac;

	ransac_detect_2D_lines(x,y,detectedLines,threshold,n_inliers);

	//cout << "RANSAC method: ransac_detect_2D_lines" << endl;
	//cout << " Computation time: " << tictac.Tac()*1000.0 << " ms" << endl;
	//cout << " " << detectedLines.size() << " lines detected." << endl;	

	if(detectedLines.size() == 0)
		return line;
	
	vector<pair<size_t,TLine2D> >::iterator p=detectedLines.begin();

	line.resize(4);

	line[0] = -15;
	line[1] = 15;
	
	for (vector_float::Index q=0;q<2;q++)
		line[q+2] = -(p->second.coefs[2]+p->second.coefs[0]*line[q])/p->second.coefs[1];

	return line;
}

void plotLine(mrpt::gui::CDisplayWindowPlots &win, vector<float> line, string format, string name)
{
/*
	vector_double lx(2), ly(2);
	lx[0]=line[0];
	lx[1]=line[1];
	ly[0]=line[2];
	ly[1]=line[3];
*/
	vector_double lx(21), ly(21);
	for(int i=0; i<21; i++)
	{
		lx[i] = i-10;
		ly[i] = (-lx[i]*line[0] - line[2])/line[1];
	}
	
	win.plot(lx,ly,format,name);
}

void plotPoints(mrpt::gui::CDisplayWindowPlots &win, vector<float> x,vector<float> y, string format, string name)
{
	
	vector_double lx(x.size()), ly(y.size());
	
	for (unsigned int i=0; i<x.size();i++) {
		lx[i]=x[i];
		ly[i]=y[i];
	}
	
	win.plot(lx,ly,format,name);
}
void plotPoints(mrpt::gui::CDisplayWindowPlots &win, vector_float x,vector_float y, string format, string name)
{
	win.plot(x,y,format,name);
}



