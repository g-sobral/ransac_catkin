#include <vector>
#include <math.h>
#include "ros/ros.h"
#include <time.h>
#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/gui.h>
#include <mrpt/utils.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::random;

#define PI 3.14159265

/* LineTracking - Função de controle para seguir uma trajetória dada por uma reta
 *  Entradas: 
 *
 *	line 	: vetor contendo os pontos que definem a trajetória (x1, x2, y1, y2)
 *  currentVelocity: velocidade atual do veículo
 *  currentPose: posição atual do veículo
 *
 *  Saída:
 *
 *  velocidade linear em m/s e angular em rad/s

 */

class Controle
{
	private:
		static double errori_[1];
	public:
		static double LineTracking( vector_double line, const double v_linear, const double v_angular, double dt, double KPT, double KIT, double KRT, double KVT);
};

