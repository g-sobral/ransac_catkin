#include "ransac_lib.hpp"

CDisplayWindowPlots* window;

void linesCallback(const ransac_project::BorderLines& msg)
{
	vector<float> lineR(4), lineL(4);
/*
	lineR[0] = -15;
	lineR[1] = 15;
	lineR[2] = -(msg.line_right[2] + msg.line_right[0]*lineR[0])/msg.line_right[1];
	lineR[3] = -(msg.line_right[2] + msg.line_right[0]*lineR[1])/msg.line_right[1];
	
	lineL[0] = -15;
	lineL[1] = 15;
	lineL[2] = -(msg.line_left[2] + msg.line_left[0]*lineL[0])/msg.line_left[1];
	lineL[3] = -(msg.line_left[2] + msg.line_left[0]*lineL[1])/msg.line_left[1];

	plotLine(*window,lineL,"r-2","ransac: left side");
	plotLine(*window,lineR,"b-2","ransac: right side");
*/
	plotLine(*window,msg.line_left,"r-2","ransac: left side");
	plotLine(*window,msg.line_right,"b-2","ransac: right side");
	plotPoints(*window,msg.x_left,msg.y_left,"g.2","scan: left side");
	plotPoints(*window,msg.x_right,msg.y_right,"m.2","scan: right side");
}

void bisecCallback(const ransac_project::Bisectrix& msg)
{
	plotLine(*window,msg.bisectrix,"g-2","bisectrix");
}

int main(int argc, char** argv)
{
	init(argc, argv, "dataplot");
	NodeHandle node;
	
	window = new CDisplayWindowPlots("RANSAC", 800, 600);
	window->axis(-5, 5, -3, 13, true);
	
	Subscriber sub1 = node.subscribe("ransac_lines", 1, linesCallback);
	Subscriber sub2 = node.subscribe("bisec_data", 1, bisecCallback);
	
	spin();
	
	return 0;
}
