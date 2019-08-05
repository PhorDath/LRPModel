#pragma once

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>

using namespace std;

struct coordinate {
	int x, y;
};

class PI
{
private:
	string directory;
	string fileName;

	void readInstance(); // read prodhon instances

	// prodhon's model

	// parameters
	int numCustomers, J, N;
	int numDepots, I, P;
	int V;
	vector<coordinate> dCoords; // depots coordinates
	vector<coordinate> cCoords; // customers coordinates
	vector<coordinate> coords; // all coords in a vector
	int vehicleCap; // vehicle capacities
	vector<int> dCap; // depots capacities
	vector<int> cDemands; // customers demands
	vector<int> ocDepots; // opening costs for the depots
	int vCost; // opening cost of a route (cost of a vehicle)
	int type; // 0 or 1 (0 means that the costs are integer - 1 that costs are real)
	vector<vector<float>> distances;
	int maxVehicles, K, R; // maximum number of vehicles aveilable. since this is a decision variable, it will be considered maxVahicles = numCustomers, then this value will be minimized

	// extra functions
	float distance(coordinate a, coordinate b);

public:

	PI(string directory, string fileName);
	PI(string fileName);
	void printData();
	~PI();
};

