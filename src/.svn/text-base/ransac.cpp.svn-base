#include "ransac_lib.hpp"
#include "ransac_classes.hpp"

/*
*	parametros:
*	threshold:
*	p_inliers:
*	dataWidth: define a que distancia pontos lidos pelo Hokuyo sao considerados sendo que esta sendo considerado
*	uma largura de 1 dataWidht para esquerda e para a direita e 3x para frente
*
*	laserCallback: recebe pontos vindos ho Hokuyo, os transforma em coordenadas cartesianas eos filtra com o parametro
*	dataWidth e os separa em pontos da esquerda e direita para que a funçao ransac2Dlines possa achar retas do lado esquerdo
*	e direito.
*	Entrada:
*	mensagem do Hokuyo que contem distancias e angulos da sua leitura formando pontos em coordenadas polares
*	Saida:
*	mensagem definida no pacote ransac_project::BorderLines que contem as linhas encontradas e os pontos que foram
*	utilzados no formato (x,y)
*
*
*/
	
void laser::laserCallback(const sensor_msgs::LaserScan& msg)
{
	watchdog->IsAlive();
	
	vector_float x_left, x_right, y_left, y_right, line_left, line_right;
	vector<float> x_l, y_l, x_r, y_r, line_r, line_l, intersec, intersec_line(4);
	float theta;
	
	geometry_msgs::PointStamped laser_point;
	laser_point.header.stamp = Time();
	laser_point.header.frame_id = "hokuyo";
	laser_point.point.z = 1; /*valor arbitrario para teste sem o carro*/
	
	for(unsigned int i = 0; i < msg.ranges.size(); i++){
		
		if(msg.ranges[i] < msg.range_max && msg.ranges[i] > msg.range_min){
			theta = msg.angle_min + i * msg.angle_increment;
			
			/*polar -> cartesian*/
			laser_point.point.x = msg.ranges[i] * cos(theta);
			laser_point.point.y = msg.ranges[i] * sin(theta);
			
			try{
				geometry_msgs::PointStamped vero_point;
    				listener->transformPoint("vero", laser_point, vero_point);
    				
    				if(laser_point.point.x < (3 * dataWidth) && abs(laser_point.point.y) < (dataWidth)){
    					if(theta > 0){
    						x_left.push_back(vero_point.point.x);
    						y_left.push_back(vero_point.point.y);
    					}
    				
    					else if(theta < 0){
    						x_right.push_back(vero_point.point.x);
    						y_right.push_back(vero_point.point.y);
    					}
    				}
    				
    				/*ROS_INFO("hokuyo: (%.2f, %.2f. %.2f) -----> vero: (%.2f, %.2f, %.2f) at time %.2f", laser_point.point.x, laser_point.point.y, 					laser_point.point.z,vero_point.point.x, vero_point.point.y, vero_point.point.z, vero_point.header.stamp.toSec());*/
			}
			
			catch(TransformException& ex){
				ROS_ERROR("%s", ex.what());
			}
		}
		
	}
	
	/*funcao esta pegando a primeira reta do vetor de retas encontradas*/
	line_left = ransac2Dline(x_left, y_left, threshold, p_inliers);
	line_right = ransac2Dline(x_right, y_right, threshold, p_inliers);
	
	for(int j = 0; j < line_left.size(); j ++){
		line_l.push_back(line_left[j]);
	}
	
	for(int j = 0; j < line_right.size(); j ++){
		line_r.push_back(line_right[j]);
	}
	
	for(int j = 0; j < x_left.size(); j ++){
		x_l.push_back(x_left[j]);
		y_l.push_back(y_left[j]);
	}
	
	for(int j = 0; j < x_right.size(); j ++){
		x_r.push_back(x_right[j]);
		y_r.push_back(y_right[j]);
	}
	
	if(window){
		plotLine(*window,line_l,"r-2","ransac: left side");
		plotLine(*window,line_r,"b-2","ransac: right side");
		plotPoints(*window,x_l,y_l,"g.2","scan: left side");
		plotPoints(*window,x_r,y_r,"m.2","scan: right side");
	}
	
	
	/*x_l, y_l, x_r, y_r sao pontos apenas para a visualização utilizados pelo node dataplot*/
	ransac_project::BorderLines msg_pub;
	msg_pub.line_left = line_l;
	msg_pub.line_right = line_r;
	msg_pub.x_left = x_l;
	msg_pub.y_left = y_l;
	msg_pub.x_right = x_r;
	msg_pub.y_right = y_r;
	pub->publish(msg_pub);
}

int main(int argc,char **argv){

	init(argc, argv, "ransac");
	NodeHandle laser_node;
	NodeHandle node("~");
	
	bool wp = false;
	double threshold, p_inliers, dataWidth;
	
	/*parametros fundamentais para a execução do software*/
	if(!node.getParam("threshold", threshold) || !node.getParam("p_inliers", p_inliers) || !node.getParam("dataWidth", dataWidth)){
		
		ROS_ERROR("parameters not specified");
		exit(0);
	}
	
	node.getParam("windowPlot", wp);
	
	laser* ls = laser::uniqueInst(laser_node.advertise<ransac_project::BorderLines>("ransac_lines", 1), wp, laser_node);
	
	ls->setthreshold(threshold);
	ls->setp_inliers(p_inliers);
	ls->setdataWidth(dataWidth);
	
	Subscriber sub = laser_node.subscribe("scan", 1, &laser::laserCallback, ls);
	spin();
	
	return 0;
}
