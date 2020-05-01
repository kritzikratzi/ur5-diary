#include <rtde_control_interface.h>
#include <thread> // only needed for the delay
#include <vector>
#include <chrono>
#define GLM_PRECISION_HIGHP_FLOAT
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include "WavFile.h"
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include "TimeMapper.h"

using namespace ur_rtde;
using namespace std; 

double clamp(double val, double a, double b){
	if(a>b) swap(a,b);
	if(val<a) return a;
	else if(val>b) return b;
	else return val;
}

double scale_lin(double val, double from_a, double from_b, double to_a, double to_b){
	double res = (val-from_a)/(from_b-from_a)*(to_b-to_a) + to_a;
	return clamp(res, to_a, to_b);
}
int64_t current_millis(){
	using namespace std::chrono;
	auto time = system_clock::now(); // get the current time
	auto since_epoch = time.time_since_epoch(); // get the duration since epoch
	auto millis = duration_cast<milliseconds>(since_epoch);
	return millis.count();
	
}
double len(const vector<double> & p1, const vector<double> & p2){
	if(p1.size() != 6 || p2.size() != 6){
		cerr << "error, vector sizes are not 6" << endl;
		exit(1);
		return 0;
	}
	else{
		double l = 0;
		for(int i = 0; i < 3; i++){
			double dist = p2[i]-p1[i];
			l += dist*dist;
		}
		
		return sqrt(l);
	}
}

ostream& operator<<(ostream & out, const vector<double> & v){
	cout << "[";
	bool first = false;
	for(double d : v){
		cout << d << ", ";
	}
	cout << "]";
	return out;
}

vector<double> with_z(const vector<double> & pts, double z){
	if(pts.size()<3) return;
	vector<double> res = pts;
	res[2] = z;
	return res;
}

glm::vec3 camera_rotated(float rx_deg, float ry_deg, float rz_deg){
	glm::vec3 kress_rot{2.191,2.226,0.005};
	const static float deg_to_rad = M_PI/180;
	
	float angle = glm::length(kress_rot);
	if(abs(angle)>0.001){
		glm::vec3 dir = kress_rot / (float)angle;
		
		auto q1 = glm::angleAxis(angle, dir);
		auto qy = glm::angleAxis(ry_deg*deg_to_rad, glm::vec3(1,0,0));
		auto qx = glm::angleAxis(rx_deg*deg_to_rad, glm::vec3(0,1,0));
		auto qz = glm::angleAxis(rz_deg*deg_to_rad, glm::vec3(0,0,1));

		auto q = q1*qy*qx*qz;
		
		// back into rot-vec form
		glm::vec3 res = glm::axis(q);
		angle = glm::angle(q);
		return angle*res;
	}
	else{
		cerr << "ouch" << endl;
		std::terminate();
	}

}

int main(int argc, char* argv[])
{
	RTDEControlInterface rtde_control("192.168.0.100");
	double acceleration = 0.1;
	double velocity = 0.1;

	auto mv = [&](glm::vec3 pos, glm::vec3 angle){
		vector<double> p0 = {pos.x, pos.y, pos.z, angle.x, angle.y, angle.z};
		//rtde_control.moveL(p0, velocity, acceleration);
	};

//	mv({0,-0.4,0}, {0.0,0.0,0.0});
//	mv({0,-0.4,0}, {0.3,0.0,0.0});
//	mv({0,-0.4,0}, {0.3,0.3,0.0});
//	mv({0,-0.4,0}, {0.0,0.3,0.0});
	glm::vec3 kress_pos{-0.137, -0.338, 0.100};
	glm::vec3 kress_rot{2.191,2.226,0.005};
	//glm::vec3 kress_rot{2,1.58,0};
	//glm::vec3 tool_rot{-0.0968,-2.2597,2.1154};
	
	for(int i =0; i < 5*100; i+=5){
		float time_days = i/100.0;
		
		float angle = glm::length(kress_rot);
		if(abs(angle)>0.001){
			float rz = TimeMapper(time_days,0,2).easeOut(45, 0);
			float ry = TimeMapper(time_days,1,2.5).easeInOut(0, 75);
			float dz = TimeMapper(time_days,2,3).easeInOut(0,
														   0.02);
			
			glm::vec3 pos = kress_pos + glm::vec3(0,0,dz);
			glm::vec3 rot = camera_rotated(0, ry, rz);
			
			cout << "POS " << pos.x << "," << pos.y << "," << pos.z << endl;
			cout << "ROT " << rot.x << "," << rot.y << "," << rot.z;
			cout << " [enter]" << endl;
			
			//getchar();
			vector<double> p = {pos.x, pos.y, pos.z, rot.x, rot.y, rot.z};
			rtde_control.moveL(p, velocity, acceleration);
		}
		else{
			cerr << "fixme" << endl;
			std::terminate();
		}
		
	}
	
	//mv({-0.137, -0.338, 0.100},{2.0,2.0,0.3});
}
