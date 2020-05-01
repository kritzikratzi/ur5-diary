#include <rtde_control_interface.h>
#include <thread> // only needed for the delay
#include <vector>
#include <chrono>
#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include "WavFile.h"

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

class Trajectory{
public:
	Trajectory(RTDEControlInterface & rtde_control) : rtde_control(rtde_control){
	}
	
	void add(const vector<double> & p){
		points.push_back(p);
	}
	
	void follow(){
		if(points.size()==0) return;
		
		double lookahead_time = 0.05;
		double gain = 300;
		double acceleration = 0.1;
		double velocity = 0.05;
		double safe_z = 0.3;

		rtde_control.moveL(with_z(points[0],safe_z), 0.5, 0.1);
		rtde_control.moveL(points[0], 0.5, 0.1);

		for(int i = 1; i < points.size(); i++){
			cout << "res0=" << points[i] << endl;
			auto & p = points[i];
			
			double d = len(points[i], points[i-1]);

			
			if(d<=0.00001) continue;
			double time = 0.05+10*d/max(0.00001,velocity);
			cout << "i=" << i << " d=" << d << " t = " << time << endl;
			cout << p << endl;
			auto ta = current_millis();
			rtde_control.servoL(p, velocity, acceleration, time, lookahead_time, gain);
			//rtde_control.moveJ(q, velocity, acceleration);
			auto tb = current_millis();
			cout << "DT = " << (tb-ta) << endl;
			int ms = max(1, (int)(time*1000-(tb-ta)-1000*lookahead_time));
			std::this_thread::sleep_for(std::chrono::milliseconds(ms));
		}
		// loop
		rtde_control.servoStop();
		rtde_control.moveL(with_z(points.back(),safe_z), 0.5, 0.1);
	}
	
private:
	RTDEControlInterface & rtde_control;
	
	vector<vector<double>> points;
};



vector<double> tcp_from_paper_pos(glm::vec3 paper_pos, glm::vec3 hand_rot){
//	const double pen_length = 0.113; // meters
	const double pen_length = 0.080; // meters
	glm::vec3 off(0,0,-pen_length);
	off = glm::rotate(off, hand_rot.x, glm::vec3(1,0,0));
	off = glm::rotate(off, hand_rot.y, glm::vec3(0,1,0));
	off = glm::rotate(off, hand_rot.z, glm::vec3(0,0,1));
	
//	cout << "-> off=" << off.x << "," << off.y << "," << off.z << endl;
	
	glm::vec3 tcp = -off + paper_pos;
//	cout << "-> tcp=" << tcp.x << "," << tcp.y << "," << tcp.z << endl;
	return {tcp.x, tcp.y, tcp.z, hand_rot.x, hand_rot.y, hand_rot.z};
}

int main(int argc, char* argv[])
{
	RTDEControlInterface rtde_control("192.168.0.100");
	rtde_control.setTcp({0,0,0.157,M_PI,0,0,0});
	//vector<double> p1 = {-0.143, -0.435, 0.25, 0.2, 0, 0};
	//vector<double> p2 = {-0.143, -0.435, 0.30, 0, 0, 0};

	double acceleration = 0.05;
	double velocity = 1;

	auto mv = [&](glm::vec3 pos, glm::vec3 angle){
		auto p0 = tcp_from_paper_pos(pos, angle);
		cout << "res0=" << p0 << endl;
		rtde_control.moveL(p0, velocity, acceleration);
	};

	/*mv({0,-0.4,0}, {0.0,0.0,0.0});
	mv({0,-0.4,0}, {0.3,0.0,0.0});
	mv({0,-0.4,0}, {0.3,0.3,0.0});
	mv({0,-0.4,0}, {0.0,0.3,0.0});*/
	
	/*int N = 8;
	for(int i = 0; i < N; i++){
		double alpha = 2*M_PI*i/(double)N;
		double ca = cos(alpha);
		double sa = sin(alpha);
		
		mv({0,-0.4,0}, {-sa/4,ca/4,0.0});
	}*/
	
	/*glm::vec3 c(0,-0.4,0);
	mv(c + glm::vec3{0,0,+0.1}, {0,0,0});
	mv(c + glm::vec3{0,0, 0.0}, {0,0.25,0});
	mv(c + glm::vec3{0.1,0, 0.0}, {0.25,0,0});
	mv(c + glm::vec3{0.1,-0.1, 0.0}, {0.25,0.0,0});
	mv(c + glm::vec3{0.1,-0.1, 0.1}, {0.0,0.0,0});*/
	
	Trajectory tr{rtde_control};
	WavFile wav;
	wav.load("shuttle-table.wav");
	int stride = 5;
	int N = wav.getLength()/stride;
	cout << "n pts = " << N << endl;
	auto wav_pt = [&](int i){ return glm::vec2(wav.data[wav.channels*i*stride+0], wav.data[wav.channels*i*stride+1]);};
	
	glm::vec2 last_diff(0,0);
	glm::vec2 last_pt = wav_pt(0);
	for(int i = 1; i < N; i++){
		glm::vec2 pt = wav_pt(i);
		glm::vec2 diff = (pt - last_pt)*glm::vec2(4,4);
		last_pt = pt;
		
		glm::vec2 use_diff = glm::mix(last_diff, diff, 0.9);
		last_diff = use_diff;
		use_diff.x = clamp(use_diff.x, -0.3, 0.3);
		use_diff.y = clamp(use_diff.y, -0.3, 0.3);
		//cout << "diff = " << use_diff.x << ", " << use_diff.y << endl;
		double scale = 0.2;
		auto z_fix = scale_lin(pt.y,-1,1,0.001,-0.002);
		glm::vec3 paper_pos =
			glm::vec3(0,-0.4,z_fix) +
			glm::vec3(pt.x*scale,pt.y*scale,0);
		
		glm::vec3 hand_rot(-use_diff.y, use_diff.x, 0);
		
		auto res = tcp_from_paper_pos(paper_pos, hand_rot);
		cout << i << ": " <<pt.x << " / " << pt.y << "   " << res << endl;
		tr.add(res);
	}
	cout << "follow" << endl;
	
	tr.follow();
	rtde_control.servoStop();
}
