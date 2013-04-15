#include "MyTimer.h"

#include <iostream>
#include <fstream>
#include <cmath>

// Constructor
MyTimer::MyTimer():
	sum_ti(0),
	sum_ti2(0),
	N(0),
	is_started(false) {
	// Nothing else to do	
}


// Destructor
MyTimer::~MyTimer() {
	// Nothing to do
}


// Completely reset data
void MyTimer::reset() {
	sum_ti = 0.;
	sum_ti2 = 0.;
	N = 0;
	is_started = false;
}


// Start timer
void MyTimer::start() {
	is_started = true;
	start_time = high_resolution_clock::now();
}


// Stop timer and commit data point
void MyTimer::stop() {
	auto end = high_resolution_clock::now();
	if (is_started) {
		is_started = false;
		int64_t d = duration_cast<nanoseconds>(end-start_time).count();
		sum_ti += d;
		sum_ti2 += pow(d, 2);
		N += 1;
	}
}


// Stop timer discarding data point
void MyTimer::cancel() {
	is_started = false;
}


// Get statistics
void MyTimer::get_stats(double* tot_time, double* mean_time, double* std_dev,
		int* N) {
	double n = static_cast<double>(this->N);
	if (this->N) {
		*N = this->N;
		*tot_time = sum_ti*1.e-9;
		*mean_time = sum_ti*1.e-9/n;
		*std_dev = sqrt(max(0., sum_ti2/n - pow(sum_ti/n, 2)))*1.e-9;
	} else {
		*N = 0;
	}
}


// Print statistics
void MyTimer::print(const string& prefix, const string& filename) {
	double tot_time, mean_time, std_dev;
	int n;
	get_stats(&tot_time, &mean_time, &std_dev, &n);

	if (filename!="") {
		ofstream fout;
		fout.open(filename, ios::app);
		fout << prefix << "total: "<<tot_time<<" ("<<n<<"); mean: "<<mean_time<<
			" ("<<std_dev<<")"<<endl;
		fout.close();
		//FILE* out = fopen(filename.c_str(), "a");
		//fprintf(out, "Hey!");
		//fclose(out);
	} else {
		cout << prefix << "total: "<<tot_time<<" ("<<n<<"); mean: "<<mean_time<<
			" ("<<std_dev<<")"<<endl;
	}
}


