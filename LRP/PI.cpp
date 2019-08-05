#include "PI.h"

void PI::readInstance()
{
	fstream file;
	file.open(directory + fileName, ios::in);
	if (file.is_open() == false) {
		cout << "On directory " << directory << endl;
		cout << "Error opening file " << fileName << endl;
		exit(1);
	}
	coordinate aux;
	int aux_int;
	file >> numCustomers; // read number of customers
	file >> numDepots; // read number of depots
	for (int i = 0; i < numDepots; i++) {
		file >> aux.x >> aux.y; // read depot coordinate
		dCoords.push_back(aux);
	}
	for (int i = 0; i < numCustomers; i++) {
		file >> aux.x >> aux.y; // read customer coordinate
		cCoords.push_back(aux);
	}
	file >> vehicleCap; // read vehicle capacity
	for (int i = 0; i < numDepots; i++) {
		file >> aux_int; // read depot capcacity
		dCap.push_back(aux_int);
	}
	for (int i = 0; i < numCustomers; i++) {
		file >> aux_int; // read customer demand
		cDemands.push_back(aux_int);
	}
	for (int i = 0; i < numDepots; i++) {
		file >> aux_int; // read depot opening cost
		ocDepots.push_back(aux_int);
	}
	file >> vCost; // opening cost of a route (cost of a vehicle)
	file >> type; // 0 or 1 (0 means that the costs are integer - 1 that costs are real)
	// 
	J = numCustomers;
	N = J;
	I = numDepots;
	P = I;
	V = I + J;
	for (auto i : dCoords) {
		coords.push_back(i);
	}
	for (auto i : cCoords) {
		coords.push_back(i);
	}
	// distance matrix calculus
	coordinate a;
	coordinate b;
	distances.resize(V);
	for (int i = 0; i < V; i++) {
		distances.at(i).resize(V, 0);
	}
	for (int i = 0; i < V; i++) {
		for (int j = 0; j < V; j++) {
			a = coords.at(i);
			b = coords.at(j);
			distances.at(i).at(j) = distance(a, b);
		}
	}
	maxVehicles = numCustomers;
	K = maxVehicles;
	R = K;
	file.close();
}

float PI::distance(coordinate a, coordinate b)
{
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

PI::PI(string directory, string fileName)
{
	this->directory = directory;
	this->fileName = fileName;
	readInstance();
}

PI::PI(string fileName)
{
	this->directory = "";
	this->fileName = fileName;
	readInstance();
}

void PI::printData()
{
	cout << numCustomers << " " << numDepots << endl;
	cout << "Depots: \n";
	for (auto i : dCoords) {
		cout << i.x << " " << i.y << endl;
	}
	cout << "Customers: \n";
	for (auto i : cCoords) {
		cout << i.x << " " << i.y << endl;
	}
	cout << "Vehicle capacity: " << vehicleCap << endl;
	cout << "Depots capacity: ";
	for (auto i : dCap) {
		cout << i << " ";
	}
	cout << endl;
	cout << "Customers demand: ";
	for (auto i : cDemands) {
		cout << i << " ";
	}
	cout << endl;
	cout << "Deposit cost: ";
	for (auto i : ocDepots) {
		cout << i << " ";
	}
	cout << endl;
	cout << "Vehicle cost: " << vCost << endl;
	cout << "Type: " << type << endl;
	cout << "Distances: \n";
	for (auto i : distances) {
		for (auto j : i) {
			cout << fixed << setw(6) << setprecision(2) << j << " ";
		}
		cout << endl;
	}
}

PI::~PI()
{
}
