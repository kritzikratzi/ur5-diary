#include <rtde_control_interface.h>
#include <rtde_receive_interface.h>

#include <thread> // only needed for the delay
#include <vector>
#include <chrono>
#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>

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
		
		double lookahead_time = 0.03;
		double gain = 300;
		double acceleration = 0.1;
		double velocity = 0.25;

		double dt = 0.3; // millis
		rtde_control.moveJ(points[0], 0.5, 0.05);
		for(int i = 1; i < points.size(); i++){
			cout << "res0=" << points[i] << endl;
			auto & q = points[i];
			double time = dt;
			cout << "i=" << i << "/" << points.size() << " t = " << time << endl;
			auto ta = current_millis();
			rtde_control.servoJ(q, velocity, acceleration, time, lookahead_time, gain);
			//rtde_control.moveJ(q, velocity, acceleration);
			auto tb = current_millis();
			cout << "DT = " << (tb-ta) << endl;
			int ms = max(1, (int)(time*1000-50));
			std::this_thread::sleep_for(std::chrono::milliseconds(ms));
		}
		// loop
		rtde_control.servoStop();
	}
	
	
	void reverse(){
		std::reverse(points.begin(),points.end());
	}
	
	size_t npts(){
		return points.size();
	}
	
	void trim(size_t start){
		if(start>0) points.erase(points.begin()+start, points.end());
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
	cout << "starting ..." << endl; 
	RTDEControlInterface rtde_control("192.168.0.100");
	RTDEReceiveInterface rtde_receive("192.168.0.100");
	
	rtde_control.setTcp({0,0,0.157,M_PI,0,0,0});
	//vector<double> p1 = {-0.143, -0.435, 0.25, 0.2, 0, 0};
	//vector<double> p2 = {-0.143, -0.435, 0.30, 0, 0, 0};

	double acceleration = 0.05;
	double velocity = 0.5;

	auto mv = [&](glm::vec3 pos, glm::vec3 angle){
		auto p0 = tcp_from_paper_pos(pos, angle);
		cout << "res0=" << p0 << endl;
		rtde_control.moveL(p0, velocity, acceleration);
	};
	
	auto max_force = [&](){
		auto force = rtde_receive.getActualTCPForce();
		auto m = max(abs(force[0]), max(abs(force[1]), abs(force[2])));
		return m;
	};
	
	
	rtde_control.teachMode();
	system("say x");
	auto now = current_millis();
	
	Trajectory tr{rtde_control};
	
	bool did_move = false;
	size_t erase_from = 0;
	while(true){
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		double m = max_force();
		cout << "m=" << m << endl;
		if(m>2.5){
			now = current_millis();
			did_move = true;
			erase_from = tr.npts();
		}

		tr.add(rtde_receive.getActualQ());
		
		if(current_millis()-now>2000){
			rtde_control.endTeachMode();
			cout << "erase " << erase_from << " / " << tr.npts() << endl;
			system("say aufnahme fertig");
			break;
		}
	}
	
	tr.trim(erase_from);
	tr.reverse();
	tr.follow();
}
