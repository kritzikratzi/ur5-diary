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


struct Pose{
	glm::vec3 pos;
	glm::vec3 rot;
	
	std::vector<double> to_vec(){
		return {pos.x, pos.y, pos.z, rot.x, rot.y, rot.z};
	}
	
};

std::ostream & operator<<(std::ostream & out, const Pose & p){
	out << "[" << p.pos.x << "," << p.pos.y << "," << p.pos.z << "] @";
	out << "[" << p.rot.x << "," << p.rot.y << "," << p.rot.z << "]";
	return out;
}


Pose camera_rotated(float rx_deg, float ry_deg, float rz_deg){
	//tcp camx
	//const glm::vec3 kress_pos{-0.137, -0.338, 0.085}; // "zero" position, ie position of lense hovering over seed ...
	//const glm::vec3 kress_rot{2.191,2.226,0.005}; // ... and default rotation
	//const glm::vec3 kress_offset{0,0,0.02}; // distance between surface and lense
	//tcp camy
	const glm::vec3 kress_pos{-0.13768, -0.34907, 0.082}; // "zero" position, ie position of lense hovering over seed ...
	const glm::vec3 kress_rot{0,0,0.143}; // ... and default rotation
	const glm::vec3 kress_offset{0,0,0.001+0.002}; // distance between surface and lense
	
	const static float deg_to_rad = M_PI/180;
	
	float angle = glm::length(kress_rot);
	if(abs(angle)>0.001){
		glm::vec3 dir = kress_rot / (float)angle;
		
		auto q1 = glm::angleAxis(angle, dir);
		auto qx = glm::angleAxis(rx_deg*deg_to_rad, glm::vec3(1,0,0));
		auto qy = glm::angleAxis(ry_deg*deg_to_rad, glm::vec3(0,1,0));
		auto qz = glm::angleAxis(rz_deg*deg_to_rad, glm::vec3(0,0,1));

		auto q = q1*qx*qy*qz;
		
		// back into rot-vec form
		glm::vec3 r_axis = glm::axis(q);
		float r_angle = glm::angle(q);
		
		glm::vec3 off = kress_offset;
		off = glm::rotate(off, rx_deg*deg_to_rad, glm::vec3(1,0,0));
		off = glm::rotate(off, ry_deg*deg_to_rad, glm::vec3(0,1,0));
		off = glm::rotate(off, rz_deg*deg_to_rad, glm::vec3(0,0,1));
		//off.x *= -1; // WHY?
		//cout << off.x << endl;
		
		glm::vec3 pos = kress_pos + off;
		return {pos, r_axis*r_angle};
	}
	else{
		cerr << "ouch" << endl;
		std::terminate();
	}
}

int main(int argc, char* argv[])
{
	RTDEControlInterface rtde_control("192.168.0.100");
	double acceleration = 0.02;
	double velocity = 0.05;

//	mv({0,-0.4,0}, {0.0,0.0,0.0});
//	mv({0,-0.4,0}, {0.3,0.0,0.0});
//	mv({0,-0.4,0}, {0.3,0.3,0.0});
//	mv({0,-0.4,0}, {0.0,0.3,0.0});
//	glm::vec3 kress_pos{-0.137, -0.338, 0.100};
//	glm::vec3 kress_rot{2.191,2.226,0.005};
	//glm::vec3 kress_rot{2,1.58,0};
	//glm::vec3 tool_rot{-0.0968,-2.2597,2.1154};
	
	int N = 1000;
	for(int i =0; i < N; i++){
		double time_days = 6*i/(double)N;
	
		float rz = TimeMapper(time_days,0,4.5).easeOut(-45, 0);
		float ry = TimeMapper(time_days,2,6).easeInOut(0, 40);
		float dz = TimeMapper(time_days,2,6).easeInOut(0, 0.002);
		dz = 0;
		
		rz = TimeMapper(time_days,0,4.5).easeOut(0, 0);
		ry = TimeMapper(time_days,0,6).easeInOut(0, 40);
		dz = TimeMapper(time_days,2,6).easeInOut(0, 0.001);
		
		Pose p = camera_rotated(0, ry, rz);
		p.pos.z += dz;
		
		cout << "POS " << i << "/" << 595 << ": " << p;
		cout << " [enter]" << endl;
		
		//getchar();
		rtde_control.moveL(p.to_vec(), velocity, acceleration);
		//camy: -27.84, -463.66, 109.33, -1.0334, 1.3136, 1.2999
		std::this_thread::sleep_for(chrono::milliseconds(1500));
		system("cd /Users/hansi/Sync/projects/2020/comemak/code/test_10_kresse/ && ./gphoto2_capture.sh 2>&1 1>/dev/null");
		std::this_thread::sleep_for(chrono::milliseconds(100));
	}
	
	//mv({-0.137, -0.338, 0.100},{2.0,2.0,0.3});
}
