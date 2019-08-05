#pragma once
#include <vector>
#include <iomanip>
#include <fstream>
#include <gurobi_c++.h>

constexpr auto TMAX = 3600;

using namespace std;

struct coordinate {
	int x, y;
};

class LRP
{
private:
	string directory;
	string fileName;
	int wmodel; // indicates which model was built

	void readInstance(); // read prodhon instances
	void result(GRBModel &model);

	// prodhon's model

	// parameters
	int numCustomers, J, N;
	int numDepots, I, P;
	int V;
	vector<coordinate> dCoords; // depots coordinates
	vector<coordinate> cCoords; // customers coordinates
	vector<coordinate> coords;
	int vehicleCap; // vehicle capacities
	vector<int> dCap; // depots capacities
	vector<int> cDemands; // customers demands
	vector<int> ocDepots; // opening costs for the depots
	int vCost; // opening cost of a route (cost of a vehicle)
	int type; // 0 or 1 (0 means that the costs are integer - 1 that costs are real)
	vector<vector<float>> distances;
	int maxVehicles, K, R; // maximum number of vehicles aveilable. since this is a decision variable, it will be considered maxVahicles = numCustomers, then this value will be minimized

	// decision variables
	vector<GRBVar> y; // yi = 1 iff depot i is opened
	vector<vector<vector<GRBVar>>> x; // xjlk = 1 iff edge j l is traversed from j to l in the route performed by vehicle k in K.
	vector<vector<GRBVar>> f; // fij = 1 iff customer j is assigned to depot i
	vector<vector<GRBVar>> u;
	void varY(GRBModel &model);
	void varX(GRBModel &model);
	void varF(GRBModel &model);
	void varU(GRBModel &model);

	// fo and constraints
	void fo(GRBModel &model); // prodhon
	void c1(GRBModel &model); // guarantee that every customer belongs to one and only one route and that each customer has only one predecessor in the tour
	void c2(GRBModel &model); // Capacity constraints
	void c3(GRBModel &model); // ensure the continuity of each route and a return to the depot of origin
	void c4(GRBModel &model); // ensure the continuity of each route and a return to the depot of origin
	void c5(GRBModel &model); // subtour elimination constraints
	void c6(GRBModel &model); // specify that a customer can be assigned to a depot only if a route linking them is opened
	void c7(GRBModel &model); // Capacity constraints are satisfied
	void c8(GRBModel &model); // each facility must me designated to only one depot

	// barreto's model

	// parameters
	vector<vector<float>> c;
	vector<vector<float>> cij; // cost of travelling between clients
	vector<vector<float>> cik; // cost of travelling between clients and depots

	// variables
	vector<vector<vector<GRBVar>>> x_barreto; // 
	vector<vector<vector<GRBVar>>> xijl_barreto; //
	vector<vector<vector<GRBVar>>> xikl_barreto; //
	vector<vector<vector<GRBVar>>> xkil_barreto; //
	vector<GRBVar> y_barreto; //
	vector<vector<GRBVar>> u_barreto;

	void var_x_barreto(GRBModel &model); // 
	void var_xijl_barreto(GRBModel &model); // 
	void var_xikl_barreto(GRBModel &model); // 
	void var_xkil_barreto(GRBModel &model); // 
	void var_y_barreto(GRBModel &model); // 
	void var_u_barreto(GRBModel &model);

	void fo_barreto(GRBModel &model); // barreto's model objective function
	void c1_barreto(GRBModel &model);
	void c2_barreto(GRBModel &model);
	void c3_barreto(GRBModel &model);
	void c4_barreto(GRBModel &model);
	void c5_barreto(GRBModel &model);
	void c6_barreto(GRBModel &model);

	void prodhonModel(GRBModel & model);
	void barretoModel(GRBModel & model);

	float distance(coordinate a, coordinate b);

public:
	LRP(string directory, string fileName);
	LRP(string fileName);
	void model();
	void getData(GRBModel &model);
	void printData();
	~LRP();
};

