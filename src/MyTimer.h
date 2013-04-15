#ifndef MY_TIMER_H
#define MY_TIMER_H

#include <chrono>
#include <string>

using namespace std;
using namespace std::chrono;

//! Small timer class to get some statistics about several calls
class MyTimer {
public:
	//! Constructor
	MyTimer();

	//! Destructor
	virtual ~MyTimer();

	//! Completely reset data
	void reset();

	//! Start timer
	void start();

	//! Stop timer and commit data point
	void stop();

	//! Stop timer discarding data point
	void cancel();

	//! Get statistics (in seconds)
	void get_stats(double* tot_time, double* mean_time, double* std_dev,
			int* N);

	//! Print statistics
	void print(const string& prefix="", const string& filename="");

protected:
	//! Start point of current measurement
	high_resolution_clock::time_point start_time;

	//! Sum of each time measurements (in nanoseconds)
	int64_t sum_ti;

	//! Sum of square of each time measurements (for std_dev);
	int64_t sum_ti2;

	//! Number of measurements
	int N;

	//! Chrono started or not (to prevent stop without start)
	bool is_started;
};


#endif
