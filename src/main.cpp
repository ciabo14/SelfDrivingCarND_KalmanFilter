#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "ground_truth_package.h"
#include "measurement_package.h"
#include "system_configuration.h"

//#define DEBUG true

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

SystemConfiguration ParseCMDLine(int argc, char *argv[]) {

	SystemConfiguration conf;

	string usage_instructions = "Usage instructions: \n";
	usage_instructions += argv[0];
	usage_instructions += "\n";
	usage_instructions += "path/to/input.txt\n";
	usage_instructions += "path/to/output.txt\n";
	usage_instructions += "[r|l|b] for [radar only|laser only|both] \n";

	bool has_valid_args = false;

	// make sure the user has provided input and output files
	if (argc == 1) {
	cerr << usage_instructions << endl;
	} else if (argc == 2 || argc == 3) {
	cerr << "Please include an output file and the measurements you want to use.\n" << usage_instructions << endl;
	} else if (argc == 4) {
	has_valid_args = true;
	} else if (argc > 4) {
	cerr << "Too many arguments.\n" << usage_instructions << endl;
	}

	if (!has_valid_args) {
	exit(EXIT_FAILURE);
	}

	try{
		conf.inputFile= argv[1];
		conf.outputFile = argv[2];
		string sensorData = argv[3];

		if(sensorData.compare("r") == 0 || sensorData.compare("R") == 0 || sensorData.compare("radar") == 0){
			conf.radar = true;
			conf.laser = false;

		}
		else if(sensorData.compare("l") == 0 || sensorData.compare("L") == 0 || sensorData.compare("laser") == 0){
			conf.laser = true;
			conf.radar = false;
		}
		else if(sensorData.compare("b") == 0 || sensorData.compare("B") == 0 || sensorData.compare("both") == 0){
			conf.radar = true;
			conf.laser = true;
		}
		else
			throw "Not valid arg";
		cout << sensorData << conf.radar << conf.laser;
			
	}
	catch(exception &e)
	{
		cerr << "Please use a layout like the one described in the usage" << usage_instructions << endl;
		exit(EXIT_FAILURE);

	}

	return conf;
}


int main(int argc, char* argv[]) {

	SystemConfiguration conf;
	#ifndef DEBUG
	conf = ParseCMDLine(argc, argv);
	/*check_arguments(argc, argv);
	in_file_name_ = argv[1];
	out_file_name_ = argv[2];*/

	#endif

	#ifdef DEBUG
	
	conf.inputFile = "E:/Self Driving Car Nanodegree/Term 2/KFDevelopment/CarND-Extended-Kalman-Filter-Project-GITHUBDirectory/data/sample-laser-radar-measurement-data-2.txt";
	conf.outputFile = "E:/Self Driving Car Nanodegree/Term 2/KFDevelopment/CarND-Extended-Kalman-Filter-Project-GITHUBDirectory/data/sample-laser-radar-measurement-data-2-output.txt";
	conf.radar = true;
	conf.laser = true;
										
	#endif

	ifstream in_file_(conf.inputFile.c_str(), ifstream::in);

	ofstream out_file_(conf.outputFile.c_str(), ofstream::out);

	check_files(in_file_, conf.inputFile, out_file_, conf.outputFile);

	vector<MeasurementPackage> measurement_pack_list;
	vector<GroundTruthPackage> gt_pack_list;

	string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line)) {

	//cout << line << endl;
    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    istringstream iss(line);
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;

    if (sensor_type.compare("L") == 0) {
      // LASER MEASUREMENT

      // read measurements at this timestamp

      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float x;
      float y;
      iss >> x;
      iss >> y;
      meas_package.raw_measurements_ << x, y;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
	  if(conf.laser)
		measurement_pack_list.push_back(meas_package);
    } else if (sensor_type.compare("R") == 0) {
      // RADAR MEASUREMENT
      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float ro;
      float phi;
      float ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      meas_package.raw_measurements_ << ro, phi, ro_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
	  if(conf.radar)
		  measurement_pack_list.push_back(meas_package);
    }

    // read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    gt_package.gt_values_ = VectorXd(4);
    gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
	if((sensor_type.compare("R") && conf.radar) || (sensor_type.compare("L") && conf.laser))
		gt_pack_list.push_back(gt_package);
  }

  // Create a Fusion EKF instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  //Call the EKF-based fusion
  size_t N = measurement_pack_list.size();
  for (size_t k = 0; k < N; ++k) {
    // start filtering from the second frame (the speed is unknown in the first
    // frame)
    fusionEKF.ProcessMeasurement(measurement_pack_list[k]);

    // output the estimation
    out_file_ << fusionEKF.ekf_.x_(0) << "\t";
    out_file_ << fusionEKF.ekf_.x_(1) << "\t";
    out_file_ << fusionEKF.ekf_.x_(2) << "\t";
    out_file_ << fusionEKF.ekf_.x_(3) << "\t";

    // output the measurements
    if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
      // output the estimation
      out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
      out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
    } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
      // output the estimation in the cartesian coordinates
      float ro = measurement_pack_list[k].raw_measurements_(0);
      float phi = measurement_pack_list[k].raw_measurements_(1);
      out_file_ << ro * cos(phi) << "\t"; // p1_meas
      out_file_ << ro * sin(phi) << "\t"; // ps_meas
    }

    // output the ground truth packages
    out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
    out_file_ << gt_pack_list[k].gt_values_(3) << "\n";

    estimations.push_back(fusionEKF.ekf_.x_);
    ground_truth.push_back(gt_pack_list[k].gt_values_);
  }

  // compute the accuracy (RMSE)
  cout << "Accuracy - RMSE:" << endl << Tools::CalculateRMSE(estimations, ground_truth) << endl;

  // close files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_.is_open()) {
    in_file_.close();
  }
  
  return 0;
}