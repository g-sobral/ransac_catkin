#include "ros/ros.h"
#include <math.h>
#include <iostream>
#include <vector>
#include <signal.h>
#include <time.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "ransac_project/BorderLines.h"
#include "ransac_project/CarCommand.h"
#include "ransac_project/Bisectrix.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include <string.h>
#include <tr1/functional>
#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/gui.h>
#include <mrpt/utils.h>
#include <mrpt/gui/CDisplayWindowPlots.h>

using namespace ros;
using namespace std;
using namespace std::tr1;
using namespace tf;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::random;

#define PI 3.14159265
#define DURATION 0.5

/* bisectrixLine - calcula a linha bissetriz entre duas linhas, cada uma delas
					definida por dois pontos
	
	Entradas
	l1 	: vetor contendo os pontos da primeira linha (x1, x2, y1, y2)
	l2	: vetor contendo os pontos da segunda linha (x3, x4, y3, y4)
	
	Saidas
	bisectrix	: vetor contendo os pontos que definem a linha bissetriz (xb1,xb2,yb1,yb2)
		
*/
vector_double bisectrixLine(vector_double l1, vector_double l2);

/* intersectionPoint - calcula o ponto de interseccao de duas linhas, cada uma delas
					definida por dois pontos
	
	Entradas
	l1 	: vetor contendo os pontos da primeira linha (x1, x2, y1, y2)
	l2	: vetor contendo os pontos da segunda linha (x3, x4, y3, y4)
	
	Saidas
	intersec	: vetor contendo o ponto de interseccao (xi, yi)
		
*/
vector<float> intersectionPoint(vector<float> l1, vector<float> l2);


/* ransac2Dline - aproxima linhas 2D a partir de uma nuvem de pontos
	
	Entradas
	x 	: vetor contendo as coordenadas x da nuvem de pontos
	y 	: vetor contendo as coordenadas y da nuvem de pontos
	threshold: erro máximo para que um ponto seja considerado inlier
	p_inliers: porcentagem mínima de inliers
	
	Saidas
	line	: vetor contendo os pontos da linha encontrada por RANSAC (x1, x2, y1, y2)
		
*/
vector_float ransac2Dline(vector_float x, vector_float y, float threshold, float p_inliers);

/* plotLine - plota uma linha
	
	Entradas
	win		: ponteiro para a janela onde a linha deve ser plotada
	line 	: vetor contendo os pontos da linha a ser plotada (x1, x2, y1, y2)
	format	: string com o formato da linha a ser plotada
	name	: string com o nome da linha a ser plotada
		
*/
void plotLine(mrpt::gui::CDisplayWindowPlots &win, vector<float> line, string format, string name);

void plotPoints(mrpt::gui::CDisplayWindowPlots &win, vector<float> x,vector<float> y, string format, string name);

void plotPoints(mrpt::gui::CDisplayWindowPlots &win, vector_float x,vector_float y, string format, string name);

