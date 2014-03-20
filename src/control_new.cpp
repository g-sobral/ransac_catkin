#include "control.hpp"

double Controle::errori_[1] = {0};

double Controle::LineTracking( vector_double line, const double v_linear, const double v_angular, double dt, double KPT, double KIT, double KRT, double KVT)
{

  double rudder, v_perp, alpha, epsilon, dist, psi_ref;
  float TSAMPLETRAJ = ros::Time::now().toSec() - dt;
  //float KPT=20;
  //float KIT=0.1;
  //float KRT=1;
  //float KVT=0.1;
  
  /* ********************NOTAS*******************************
  
  currentVelocity (linear e angular) deve ser obtida a partir 
  da leitura do estado do veículo.
  
  A variável Controle::errori_, que armazena o erro integral, deve ser declarada
  fora desta função.
  
  */ 

  alpha = atan(-line[0]/line[1]);
  alpha = alpha < 0 ? alpha + PI : alpha;
  
  epsilon = PI/2 - alpha;
  
  v_perp = v_linear * sin(epsilon);
    
  dist = abs(line(2))/sqrt(line[0]*line[0] + line[1]*line[1]);
  
  Controle::errori_[0] = Controle::errori_[0]+dist;
  Controle::errori_[0] = Controle::errori_[0] > 27 ? 27 : Controle::errori_[0];
  Controle::errori_[0] = Controle::errori_[0] < -27 ? -27 : Controle::errori_[0];

  ROS_INFO(" alpha = %lf\t epsilon = %lf\t v_perp = %lf\n dist = %lf\t errori = %lf \n", alpha, epsilon, v_perp, dist, Controle::errori_[0]);

  psi_ref = (-1 * KPT * dist) + (-1 * KIT * Controle::errori_[0]) + (-1 * KVT * v_perp * TSAMPLETRAJ);

  // velocidade angular em radianos/seg
  rudder = KRT * psi_ref;
  
  ROS_INFO(" psi_ref = %lf\t rudder(deg) = %lf\n", psi_ref, rudder);
  
  rudder = DEG2RAD(rudder);
  
  ROS_INFO(" v_linear = %lf\t rudder(rad) = %lf\n", v_linear, rudder);
  
  return rudder;
}
