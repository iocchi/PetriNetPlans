/*
 * nodes.h
 * author: Vittorio Amos Ziparo
 */

#ifndef PNP_NODES_H
#define PNP_NODES_H

#include <set>
#include <map>

namespace PetriNetPlans {

static unsigned long _incrid = 0;

struct Node {
	Node() { id = _incrid++; }
	virtual ~Node() { }

	bool operator==(const Node& n) const	{ return n.id==id; }
	bool operator!=(const Node& n) const	{ return n.id!=id; }
	bool operator<(const Node& n) const		{ return n.id<id; }
	unsigned long int id;
};

struct Transition : public Node {
	Transition() : Node() { }
	virtual ~Transition() { }
};

struct Place : public Node {
	Place(unsigned int initialMarking=0) : Node(), initialMarking(initialMarking), currentMarking(0) { }
	virtual ~Place() { }

	unsigned int initialMarking;
	unsigned int currentMarking;
};

template <typename PlaceClass>
struct EdgePlace {
	bool operator==(const EdgePlace& e) { return this->p==e.p; }
	bool operator==(const PlaceClass* p) { return this->p==p; }

	PlaceClass* p;
	bool inhibitor;
	unsigned int weight;
};

} // namespace

#endif

