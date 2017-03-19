/**
 * Test for the KalmanFilter class with 1D projectile motion.
 *
 * @author: Hayk Martirosyan
 * @date: 2014.11.15
 */

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <iomanip>
#include "eigen-3.3/Eigen/Dense"
#include "kalman.hpp"

using namespace std;

struct Measurements {
	double radial;
	double theta;
};

void print_results(KalmanFilter kf)
{
	Eigen::VectorXd x = kf.state().transpose();
	cout << x(0) << " " << x(1) << " " << x(2) << " ";
	Eigen::MatrixXd cov = kf.covariance();
	cout << cov(0,0) << " " << cov(0,1)  << " " << cov(0,2) << " ";
	cout << cov(1,0) << " " << cov(1,1)  << " " << cov(1,2) << " ";
	cout << cov(2,0) << " " << cov(2,1)  << " " << cov(2,2) << endl;
}
int main(int argc, char* argv[]) {

  if(argc < 6)
	return 1;

  double r0 = atof(argv[1]);
  double theta0 = atof(argv[2]);
  double v0 = atof(argv[3]);
  double sigma0 = atof(argv[4]);
	
  // List of noisy position measurements (y)
  std::vector<Measurements> measurements;
  ifstream ifs(argv[5]);
  char buffer[256];
  string radial_str, theta_str;
  while(!ifs.eof() && ifs.getline(buffer, 256))
  {
	stringstream ss(buffer);

	getline(ss, radial_str, ',');
	double radial_measurement = atof(radial_str.c_str());
	getline(ss, theta_str, ',');
	double theta_measurement = atof(theta_str.c_str());
	
	Measurements m;
	m.radial = radial_measurement;
	m.theta = theta_measurement;
	measurements.push_back(m);
  }
  
  int n = 3; // Number of states
  int m = 2; // Measurements dimension

  double dt = 1.0; // Time step

  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  // Discrete projectile motion, measuring position and angle only
  A << 1, 0, 0, 0, 1, dt, 0, 0, 1;
  C << 1, 0, 0, 0, 1, 0;

  double radial_pertubations = pow(500,2);
  double angle_pertubations = pow(0.005,2);
  double vel_pertubations = pow(0.005,2);
  Q << radial_pertubations, 0, 0, 0, angle_pertubations, 0, 0, 0, vel_pertubations;
  
  // Reasonable covariance matrices
  double radial_measurement_noise = pow(2000,2);
  double theta_measurement_noise = pow(0.03,2);
  R << radial_measurement_noise, 0, 0, theta_measurement_noise;
  P << sigma0, 0, 0, 0, sigma0, 0, 0, 0, sigma0;

  //std::cout << "A: \n" << A << std::endl;
  //std::cout << "C: \n" << C << std::endl;
  //std::cout << "Q: \n" << Q << std::endl;
  //std::cout << "R: \n" << R << std::endl;
  //std::cout << "P: \n" << P << std::endl;

  // Construct the filter
  KalmanFilter kf(dt, A, C, Q, R, P);

  // Best guess of initial states
  Eigen::VectorXd x0(n);
  x0 << r0, theta0, v0;
  kf.init(0, x0);

  // Feed measurements into filter, output estimated states
  double t = 0;
  Eigen::VectorXd y(m);
  cout << fixed << setprecision(6) << left;
  cout << "Results: " << endl;
  print_results(kf);
  for(int i = 1; i < measurements.size(); i++) {
    t += dt;
    y << measurements[i].radial, measurements[i].theta;
    kf.update(y);
    cout << "Results: " << endl;
  	print_results(kf);
  }

  return 0;
}
