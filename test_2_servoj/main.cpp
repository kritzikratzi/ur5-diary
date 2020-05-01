#include <rtde_control_interface.h>
#include <thread> // only needed for the delay
#include <vector>
#include <chrono>

using namespace ur_rtde;
using namespace std; 

int64_t current_millis(){
	using namespace std::chrono;
	auto time = system_clock::now(); // get the current time
	auto since_epoch = time.time_since_epoch(); // get the duration since epoch
	auto millis = duration_cast<milliseconds>(since_epoch);
	return millis.count();
	
}
double len(const vector<double> & p1, const vector<double> & p2){
	if(p1.size() != p2.size() || p1.size() == 0){
		cerr << "error, vector sizes don't match or are zero" << endl;
		exit(1);
		return 0;
	}
	else{
		double l = 0;
		for(int i = 0; i < p1.size(); i++){
			double dist = p2[i]-p1[i];
			l += dist*dist;
		}
		
		return sqrt(l);
	}
}

class Trajectory{
public:
	Trajectory(RTDEControlInterface & rtde_control) : rtde_control(rtde_control){
	}
	
	void add(const vector<double> & p){
		auto q = rtde_control.getInverseKinematics(p,{0,0,0,0,0,0});
		if(q.size() != 6){
			cerr << "inverse kinematics does not have 6 points, wtf!" << endl;
			exit(1);
			return;
		}
		points.push_back({p,q});
	}
	
	void follow(){
		if(points.size()==0) return;
		
		double lookahead_time = 0.05;
		double gain = 300;
		double acceleration = 0.5;
		double velocity = 0.2;

		rtde_control.moveL(points[0].p, 0.1, 0.1);
		
		for(int i = 1; i < points.size(); i++){
			auto & pt = points[i];
			
			double d = len(points[i].p, points[i-1].p);
			if(d<=0.00001) continue;
			double time = 0.05+10*d/max(0.00001,velocity);
			cout << "i=" << i << " d=" << d << " t = " << time << endl;

			auto ta = current_millis();
			rtde_control.servoJ(pt.q, velocity, acceleration, time, lookahead_time, gain);
			auto tb = current_millis();
			cout << "DT = " << (tb-ta) << endl;
			int ms = max(1, (int)(time*1000-(tb-ta)-1000*lookahead_time));
			std::this_thread::sleep_for(std::chrono::milliseconds(ms));
		}
		// loop
		rtde_control.servoStop();
		rtde_control.moveL(points.front().p, 0.1, 0.1);
	}
	
private:
	struct Pt{
		vector<double> p;
		vector<double> q;
	};
	
	RTDEControlInterface & rtde_control;
	
	vector<Pt> points;
};

int main(int argc, char* argv[])
{
  RTDEControlInterface rtde_control("192.168.0.100");
	//vector<double> p1 = {-0.143, -0.435, 0.25, 0, 0, 0};
	//vector<double> p2 = {-0.143, -0.435, 0.30, 0, 0, 0};
	//	vector<double> joint_q1 = rtde_control.getInverseKinematics(p1,{0,0,0,0,0,0});
	//	vector<double> joint_q2 = rtde_control.getInverseKinematics(p2,{0,0,0,0,0,0});

	Trajectory tr{rtde_control};
	int N = 200;
	for(int i = 0; i <= N; i++){
		/*vector<double> p = {
			-0.143+0.4*i/(double)N,
			-0.435,
			0.25+0.05*sin(2*i*M_PI*2.0/N),
			-0, 0, 0};*/
		float sz = 1-i/(double)N;
		vector<double> p = {
			-0.0000+sz*0.1*sin(M_PI*18*i/(double)N),
			-0.435+sz*0.1*cos(M_PI*18*i/(double)N),
			0.05+0.3*i/(double)N,
			0, 0, 0};
		tr.add(p);
	}
	
	tr.follow();


  rtde_control.servoStop();
}
