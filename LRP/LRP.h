#pragma once
#include <vector>
#include <fstream>
#include <gurobi_c++.h>

#define TMAX 3600

using namespace std;

struct coordinate {
	int x, y;
};

class LRP
{
private:
	string directory;
	string fileName;

	void readInstance(); // read prodhon instances
	void result(GRBModel &model);

	// parameters
	int numCustomers;
	int numDepots;
	vector<coordinate> dCoords; // depots coordinates
	vector<coordinate> cCoords; // customers coordinates
	int vehicleCap; // vehicle capacities
	vector<int> dCap; // depots capacities
	vector<int> cDemands; // customers demands
	vector<int> ocDepots; // opening costs for the depots
	int vCost; // opening cost of a route (cost of a vehicle)
	int type; // 0 or 1 (0 means that the costs are integer - 1 that costs are real)
	vector<vector<float>> distances;
	int maxVehicles; // maximum number of vehicles aveilable. since this is a decision variable, it will be considered maxVahicles = numCustomers, then this value will be minimized

	// decision variables
	vector<GRBVar> y; // yi = 1 iff depot i is opened
	vector<vector<vector<GRBVar>>> x; // xjlk = 1 iff edge j l is traversed from j to l in the route performed by vehicle k in K.
	vector<vector<GRBVar>> f; // fij = 1 iff customer j is assigned to depot i

	void varY(GRBModel &model);
	void varX(GRBModel &model);
	void varF(GRBModel &model);
	void fo(GRBModel &model);
	void c1(GRBModel &model); // guarantee that every customer belongs to one and only one route and that each customer has only one predecessor in the tour
	void c2(GRBModel &model); // Capacity constraints
	void c3(GRBModel &model); // ensure the continuity of each route and a return to the depot of origin
	void c4(GRBModel &model); // ensure the continuity of each route and a return to the depot of origin
	void c5(GRBModel &model); // subtour elimination constraints
	void c6(GRBModel &model); // specify that a customer can be assigned to a depot only if a route linking them is opened
	void c7(GRBModel &model); // Capacity constraints are satisfied

public:
	LRP(string directory, string fileName);
	LRP(string fileName);
	void model();
	void printData();
	~LRP();
};

