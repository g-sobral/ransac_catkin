#include "control.hpp"

double Controle::errori_[1] = {0};

double Controle::LineTracking( vector_double line, const double v_linear, const double v_angular, double dt, double KPT, double KIT, double KRT, double KVT)
{
  double trans[2],dist[2];
  double dist_val,head,rudder;

  double vel[2],desv_vel[2]; //,mod_desv;
  double dd, dd1,ddv;
  double psirefc;
  float TSAMPLETRAJ = ros::Time::now().toSec() - dt;
  //float KPT=20;
  //float KIT=0.1;
  //float KRT=1;
  //float KVT=0.1;

  stringstream ss;
  //ss << "Alg" << std::endl;
  
  /* ********************NOTAS*******************************
  
  currentVelocity (linear e angular) deve ser obtida a partir 
  da leitura do estado do veículo.
  
  A variável Controle::errori_, que armazena o erro integral, deve ser declarada
  fora desta função.
  
  */ 

  // convert heading (pose, ins) and velocity (odometry) into NORTHVEL and EASTVEL
  float alfa;
  alfa = atan2(line[3]-line[2], line[1]-line[0]);
  float northv = v_linear * sin(alfa);
  float eastv = v_linear * cos(alfa);
  // the yaw rate in odometry is in rads
  float YAW_RATE = RAD2DEG(v_angular);  // getting angular velocity from odometry - exchange for INS


  
  //ss << "currV l "<< currentVelocity.lin() << " r " << YAW_RATE << " nv= " << northv << " ev= " << eastv << std::endl;

  

  //determina o vetor para transicao de trajetoria
  trans[0] = line[1] - line[0];
  trans[1] = line[3] - line[2];
  //normaliza o vetor trans
  float n = sqrt(trans[0]*trans[0]+trans[1]*trans[1]);
  trans[0]/=n;trans[1]/=n;

  //ss << " trans " << trans[0] << " " << trans[1] << " n " << n << std::endl;
  ROS_INFO("trans normalizado (%lf , %lf) \n",trans[0],trans[1]);

  // obtem distancias
  dist[0] =  - line[1];
  dist[1] =  - line[3];


  dd = dist[1]*trans[0] - dist[0]*trans[1];
  //ss << " dist " << dist[0] << " " << dist[1] << " dd " << dd << std::endl;
  ROS_INFO("dist (%lf , %lf) dd %lf \n",dist[0],dist[1],dd);
  // projecao de p1_patual na direcao da reta
  dist_val = dist[0]*trans[0]+dist[1]*trans[1];

  // Calcula o angulo da direcao da reta
  vel[0] = eastv; vel[1] = northv;

  // Obtem o deslocamemto devido a velocidade ...
  desv_vel[0] = vel[0]* TSAMPLETRAJ; desv_vel[1] = vel[1]* TSAMPLETRAJ;

  //ss << " dist_val " << dist_val << " vel [" << vel[0] << " " << vel[1] << "]" << std::endl;
  ROS_INFO("dist_val %lf vel (%lf , %lf) \n",dist_val,vel[0],vel[1]);

  // Obtem a distancia deste deslocamento a direcao
  dd1 =  desv_vel[1]*trans[0] - desv_vel[0]*trans[1];
  //ss << " desv_vel [" << desv_vel[0] << " " << desv_vel[1] << "] dd1" << dd1 << std::endl;
  ROS_INFO("desv_vel (%lf , %lf) dd1 %lf \n",desv_vel[0],desv_vel[1],dd1);

  // Obtem a velocidade perpendicular
  ddv = vel[1]*trans[0] - vel[0]*trans[1];

  //ss << " ddv " << ddv << std::endl;
  ROS_INFO("ddv %lf \n",ddv);
  
  //Mudando para PID comum
  Controle::errori_[0]=Controle::errori_[0]+dd;

  if (Controle::errori_[0] > 27) Controle::errori_[0] = 27;
  else if(Controle::errori_[0] < -27) Controle::errori_[0] = -27;

  //ss << "errori [" << Controle::errori_[0] << " " << Controle::errori_[1] << " " << Controle::errori_[2] << "] " << std::endl;
  ROS_INFO("errori %lf \n",Controle::errori_[0]);

  psirefc =   ( (-1) * KPT * dd )  +  ( (-1) *  KVT * ddv )   +  (  (-1) * KIT * Controle::errori_[0]);
  //          (   PROPORCIONAL  )     (    DERIVATIVO     )      (       INTEGRAL        )

  psirefc = psirefc>= 90? 89:psirefc;
  psirefc = psirefc<=-90?-89:psirefc;


  head = RAD2DEG(atan2(trans[1],trans[0]));

  rudder = psirefc + head;
  rudder = rudder> 180.0?rudder-360.0:rudder;
  rudder = rudder<-180.0?rudder+360.0:rudder;

  //ss << " psirefc " << psirefc << " head " << head <<" rudder " << rudder <<  std::endl;
  ROS_INFO("psirefc %lf head %lf rudder %lf \n",psirefc,head,rudder);

  rudder = rudder > 180.0 ? rudder - 360.0 : rudder;
  rudder = rudder < -180.0 ? rudder + 360.0 : rudder;

  //BRUNO: G_RUD_TRAJ agora eh KRT
  rudder = KRT * rudder;

  //rudder = rudder + (1.5) * YAW_RATE; // rudder is deg, suppose YAW_RATE must be deg too?

  rudder = (rudder>20?20:(rudder<-20?-20:rudder));
  
  // velocidade angular em radianos/seg
  rudder = rudder*PI/180;

  //ss << " finalv l" << v.lin() << " r " << v.rot() << std::endl;
  ROS_INFO("V linear %lf V angular %lf \n",v_linear,rudder);
  //return v;
  
  return rudder;
} // approachGoalCommandAurora
