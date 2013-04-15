#ifndef MY_PRIORITY_QUEUE_H
#define MY_PRIORITY_QUEUE_h

#include <vector>
#include <algorithm>

using namespace std;

//! Priority queue including removal of random element
template<class T, class Less>
class MyPriorityQueue: public vector<T> {
public:
	//! Return and remove top element
	T pop() {
		T top(this->front());
		pop_heap(this->begin(), this->end(), Less());
		this->pop_back();
		return top;
	}
	//! Push element in heap
	void push(const T& e) {
		this->push_back(e);
		push_heap(this->begin(), this->end(), Less());
	}
	//! Remove random element
	void remove(const T& e) {
		auto it = find(this->begin(), this->end(), e);
		*it = this->back();
		this->pop_back();
		make_heap(this->begin(), this->end(), Less());
	}
};

#endif
