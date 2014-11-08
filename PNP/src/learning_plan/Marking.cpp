#include <pnp/learning_plan/Marking.h>

#include <algorithm>
#include <iostream>
#include <iterator>

using namespace std;

namespace learnpnp {

struct GetSecond {
	template <typename T1, typename T2>
	T2 operator() (std::pair<T1, T2> p) {
		return p.second;
	}
};

class MarkingNonZero {

public:
MarkingNonZero() {}

bool operator() (const std::pair<std::string, int> &pa) {
	return pa.second > 0;
}

};


Marking::Marking() : marking(), lastNonZero(findLastNonZero()) {}

Marking::Marking (const std::map<std::string, int>& markMap) : id("unknown") {
 
 	transform(markMap.begin(), markMap.end(), back_inserter(marking), GetSecond());
 	std::map<std::string, int>::const_iterator markIt = find_if(markMap.begin(), markMap.end(), MarkingNonZero());

	bool first = true;
	while( markIt != markMap.end()) {
		
		if(first) {
			id = (*markIt).first; //replace the string
			first = false;
		}
		else
			id+= string("-") + (*markIt).first; //sum up to the string

		markIt = find_if(++markIt, markMap.end(), MarkingNonZero());
	}

	lastNonZero = findLastNonZero();
}

Marking::Marking (const std::vector<int>& marking, std::string id ) : marking(marking), id(id) {
	lastNonZero = findLastNonZero();
}

std::string Marking::getId() const{
return id;
}

int Marking::findLastNonZero() {

	for(int i = marking.size()-1; i>= 0; --i) {
		if(marking[i] != 0)
			return i;
	}

	return -1;
}

//variables to measure the perfomance in operator<, remove them when not needed anymore
// int Marking::count1(0);
// int Marking::count2(0);

bool operator< (const Marking& a,const  Marking& b) {

	if(a.marking.size() != b.marking.size())
		return a.marking.size() < b.marking.size();

// 	for(int i = a.marking.size()-1; i>= 0; --i) {
//  		if(a.marking[i] != b.marking[i])
//  			return a.marking[i] < b.marking[i];
//  	}

	//one of the is the empty marking, the other is greater
	if(a.lastNonZero < 0) {
		return false;
	}
	else if (b.lastNonZero < 0) {
		return true;
	}

	//lastNonZeros are different, the higher is the greater
	if(a.lastNonZero != b.lastNonZero) {
// 		++Marking::count1;
// 		if(Marking::count1%10000 == 0)
// 			std::cout << "count1 " << Marking::count1 << std::endl;
		return a.lastNonZero < b.lastNonZero;
	}

	//they are the same, but their values might be different
	if(a.marking[a.lastNonZero] != b.marking[b.lastNonZero]) {
// 		++Marking::count1;
// 		if(Marking::count1%10000 == 0)
// 			std::cout << "count1 " << Marking::count1 << std::endl;
		
		return a.marking[a.lastNonZero] < b.marking[b.lastNonZero];
	}

	//we have to go for other non zero values :(
	for(int i = a.lastNonZero; i>= 0; --i) {
// 		++Marking::count2;
// 		if(Marking::count2%10000 == 0)
// 			std::cout << "count2 " << Marking::count2 << std::endl;
		
		if(a.marking[i] != b.marking[i])
			return a.marking[i] < b.marking[i];
	}
	
	return false;
}

bool operator ==(const Marking& a,const  Marking& b) {
	return a.id == b.id;
}

bool operator !=(const Marking& a,const  Marking& b) {
	return !(a == b);
}

std::istream &operator>>(std::istream& stream, std::pair<Marking, double>& markPair) {
	
	string id;
	stream >> id;

	stream >> markPair.second;
	
	vector<int> tempVector;
	copy(istream_iterator<int>(stream), istream_iterator<int>(), back_inserter(tempVector));
	
	markPair.first = Marking(tempVector, id);

	return stream;
}

std::ostream &operator<<(std::ostream& stream, const std::pair<Marking, double>& markPair) {
	
	stream << markPair.first.id << "\t\t";
	
	stream << markPair.second << "\t\t\t";
	
	copy(markPair.first.marking.begin(), markPair.first.marking.end(), ostream_iterator<int>(stream, "\t"));
	
	stream << std::endl;

	return stream;
}

}
