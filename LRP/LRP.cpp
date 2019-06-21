#include "LRP.h"

void LRP::readInstance()
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
	// distance matrix calculus
	distances.resize(numDepots);
	for (int i = 0; i < distances.size(); i++) {
		distances.at(i).resize(numCustomers, 0);
	}
	for (int i = 0; i < numDepots; i++) {
		for (int j = 0; j < numCustomers; j++) {
			coordinate a = cCoords.at(j);
			coordinate b = dCoords.at(i);
			distances.at(i).at(j) = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
		}
	}
	
	maxVehicles = numCustomers;
	file.close();
}

void LRP::result(GRBModel &model)
{
	int status = model.get(GRB_IntAttr_Status);
	if (status == GRB_UNBOUNDED) {
		cout << "O modelo nao pode ser resolvido porque e ilimitado." << endl;
	}
	if (status == GRB_OPTIMAL) {
		cout << "Solucao otima encontrada!" << endl;
		cout << "O valor da solucao otima eh: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
	}
	if (status == GRB_TIME_LIMIT) {
		cout << "Tempo limite!" << endl;
		cout << "O valor da melhot solucao ate o momento e: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
	}
	if (status == GRB_INFEASIBLE) {
		cout << "O modelo nao pode ser resolvido porque e inviavel." << endl;
	}
	else {
		cout << "Status: " << status << endl;
	}
	
}

void LRP::varY(GRBModel & model)
{
	y.resize(numDepots);
	for (int i = 0; i < y.size(); i++) {
		y.at(i) = model.addVar(0, 1, 1, GRB_BINARY, "y(" + to_string(i) + ")");
	}
	model.update();
}

void LRP::varX(GRBModel & model)
{
	x.resize(numDepots);
	for (int i = 0; i < x.size(); i++) {
		x.at(i).resize(numCustomers);
		for (int j = 0; j < x.at(i).size(); j++) {
			x.at(i).at(j).resize(maxVehicles);
		}
	}
	for (int i = 0; i < numDepots; i++) {
		for (int j = 0; j < numCustomers; j++) {
			for (int k = 0; k < maxVehicles; k++) {
				x.at(i).at(j).at(k) = model.addVar(0, 1, 1, GRB_BINARY, "x(" + to_string(i) + "," + to_string(j) + "," + to_string(k) + ")");
			}
		}
	}
	model.update();
}

void LRP::varF(GRBModel & model)
{
	f.resize(numDepots);
	for (int i = 0; i < f.size(); i++) {
		f.at(i).resize(numCustomers);
	}
	for (int i = 0; i < f.size(); i++) {
		for (int j = 0; j < f.at(i).size(); j++) {
			f.at(i).at(j) = model.addVar(0, 1, 1, GRB_BINARY, "f(" + to_string(i) + "," + to_string(j) + ")");
		}
	}
	model.update();
}

void LRP::fo(GRBModel & model)
{
	GRBLinExpr p1{ 0 }, p2{ 0 }, p3{ 0 };

	// first part
	// cost of opening the facilities
	for (int i = 0; i < numDepots; i++) {
		p1 += ocDepots.at(i) * y.at(i);
	}

	// second part
	// cost of travel
	for (int i = 0; i < numDepots; i++) {
		for (int j = 0; j < numCustomers; j++) {
			for (int k = 0; k < maxVehicles; k++) {
				p2 += distances.at(i).at(j) * x.at(i).at(j).at(k);
			}
		}
	}

	// third part
	// fixed cost of each vehicle used
	for (int k = 0; k < maxVehicles; k++) {
		for (int i = 0; i < numDepots; i++) {
			for (int j = 0; j < numCustomers; j++) {
					p3 += vCost * x.at(i).at(j).at(k);
			}
		}
	}

	GRBLinExpr fo{ 0 };
	fo = p1 + p2 + p3;
	model.setObjective(fo, GRB_MINIMIZE);
	model.update();
}

void LRP::c1(GRBModel & model)
{
	//cout << x.size() << " " << x.at(0).size() << " " << x.at(0).at(0).size() << endl;
	for (int j = 0; j < numCustomers; j++) {
		GRBLinExpr c1{ 0 };
		for (int k = 0; k < maxVehicles; k++) {
			for (int i = 0; i < numDepots; i++) { // i in V?
				c1 += x.at(i).at(j).at(k);
			}
		}
		model.addConstr(c1 == 1, "c1(" + to_string(j) + ")");
	}
	model.update();
}

void LRP::c2(GRBModel & model)
{
	for (int k = 0; k < maxVehicles; k++) {
		GRBLinExpr c2{ 0 };
		for (int j = 0; j < numCustomers; j++) {
			for (int i = 0; i < numDepots; i++) { // i in V?
				try {
					c2 += cDemands.at(j) * x.at(i).at(j).at(k);
				}
				catch (exception e) {
					cout << cDemands.size() << endl;
					cout << i << " " << j << " " << k << endl;
					cout << e.what() << endl;
					exit(1);
				}
			}
		}
		model.addConstr(c2 <= vehicleCap, "c2(" + to_string(k) + ")");
	}
	model.update();
}

void LRP::c3(GRBModel & model) // maybe wrong
{
	for (int k = 0; k < maxVehicles; k++) {
		for (int i = 0; i < numDepots; i++) {
			GRBLinExpr c31{ 0 };

			for (int j = 0; j < numCustomers; j++) {
				c31 += x.at(i).at(j).at(k);
			}
			
			GRBLinExpr c32{ 0 };
			for (int j = 0; j < numCustomers; j++) {
				c32 += x.at(i).at(j).at(k);
			}
			
			model.addConstr(c31 - c32 == 0, "c3(" + to_string(k) + ")");
		}		
	}
	model.update();
}

void LRP::c4(GRBModel & model)
{
	for (int k = 0; k < maxVehicles; k++) {
		GRBLinExpr c4{ 0 };
		for (int i = 0; i < numDepots; i++) {
			for (int j = 0; j < numCustomers; j++) {
				c4 += x.at(i).at(j).at(k);
			}
		}
		model.addConstr(c4 <= 1, "c4(" + to_string(k) + ")");
	}
	model.update();
}

void LRP::c5(GRBModel & model) // subrout elimination
{
	for (int k = 0; k < maxVehicles; k++) {
		GRBLinExpr c5{ 0 };
		for (int i = 0; i < numCustomers; i++) {
			for (int j = 0; j < numDepots; j++) {
				c5 += x.at(i).at(j).at(k);
			}
		}
		model.addConstr(c5 <= 1, "c5(" + to_string(k) + ")");
	}
	model.update();
}

void LRP::c6(GRBModel & model)
{
	for (int k = 0; k < maxVehicles; k++) {
		for (int i = 0; i < numDepots; i++) {
			for (int j = 0; j < numCustomers; j++) {
				GRBLinExpr c61{ 0 };
				for (int u = 0; u < numCustomers; u++) {
					c61 += x.at(i).at(u).at(k);

				}
				GRBLinExpr c62{ 0 };
				for (int u = 0; u < numDepots; u++) {
					c62 += x.at(u).at(j).at(k);
				}				
				model.addConstr(c61 + c62 <= 1 + f.at(i).at(j), "c6(" + to_string(k) + ")");
			}
		}		
	}
	model.update();
}

void LRP::c7(GRBModel & model)
{
	for (int i = 0; i < numDepots; i++) {
		GRBLinExpr c7{ 0 };
		for (int j = 0; j < numCustomers; j++) {
			c7 += cDemands.at(j) * f.at(i).at(j);
		}
		model.addConstr(c7 <= vehicleCap * y.at(i), "c7(" + to_string(i) + ")");
	}
	model.update();
}

LRP::LRP(string directory, string fileName)
{
	this->directory = directory;
	this->fileName = fileName;
	readInstance();
}

LRP::LRP(string fileName)
{
	this->directory = "";
	this->fileName = fileName;
	readInstance();
}

void LRP::model()
{
	GRBEnv env = GRBEnv(true);
	env.start();

	try {
		GRBModel model = GRBModel(env);
		model.set(GRB_StringAttr_ModelName, "LRP_" + fileName);

		varF(model);
		varX(model);
		varY(model);
		fo(model);
		c1(model);
		c2(model);
		c3(model);
		c4(model);
		//c5(model);
		c6(model);
		c7(model);

		model.write("teste.lp");
		model.getEnv().set(GRB_DoubleParam_TimeLimit, TMAX);
		model.optimize();

		result(model);
		model.write("teste.sol");
	}
	catch (GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}
	catch (exception e) {
		cout << e.what() << endl;
	}
}

void LRP::printData()
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
	cout << "Deposit cost: \n";
	for (auto i : ocDepots) {
		cout << i << endl;
	}
	cout << "Vehicle cost: " << vCost << endl;
	cout << "Type: " << type << endl;
	cout << "Distances: \n";
	for (auto i : distances) {
		for (auto j : i){
			cout << j << " ";
		}
		cout << endl;
	}
}

LRP::~LRP()
{
}

/*
void LRP::varY(GRBModel & model)
{
	y.resize(numDepots);
	for (int i = 0; i < y.size(); i++) {
		y.at(i) = model.addVar(0, 1, 1, GRB_BINARY, "y(" + to_string(i) + ")");
	}
	model.update();
}

void LRP::varX(GRBModel & model)
{
	x.resize(numCustomers);
	for (int i = 0; i < x.size(); i++) {
		x.at(i).resize(numDepots);
		for (int j = 0; j < x.at(i).size(); j++) {
			x.at(i).at(j).resize(maxVehicles);
		}
	}
	for (int i = 0; i < x.size(); i++) {
		for (int j = 0; j < x.at(i).size(); j++) {
			for (int k = 0; k < x.at(i).at(j).size(); k++) {
				x.at(i).at(j).at(j) = model.addVar(0, 1, 1, GRB_BINARY, "x(" + to_string(i) + "," + to_string(j) + "," + to_string(k) + ")");
			}
		}
	}
	model.update();
}

void LRP::varF(GRBModel & model)
{
	f.resize(numCustomers);
	for (int i = 0; i < f.size(); i++) {
		f.at(i).resize(numDepots);
	}
	for (int i = 0; i < f.size(); i++) {
		for (int j = 0; j < f.at(i).size(); j++) {
			f.at(i).at(j) = model.addVar(0, 1, 1, GRB_BINARY, "f(" + to_string(i) + "," + to_string(j) + ")");
		}
	}
	model.update();
}

void LRP::fo(GRBModel & model)
{
	GRBLinExpr p1{ 0 }, p2{ 0 }, p3{ 0 };

	// first part
	// cost of opening the facilities
	for (int i = 0; i < numDepots; i++) {
		p1 += ocDepots.at(i) * y.at(i);
	}

	// second part
	// cost of travel
	for (int i = 0; i < numCustomers; i++) {
		for (int j = 0; j < numDepots; j++) {
			for (int k = 0; k < maxVehicles; k++) {
				p2 += distances.at(i).at(j) * x.at(i).at(j).at(k);
			}
		}
	}

	// third part
	// fixed cost of each vehicle used
	for (int k = 0; k < maxVehicles; k++) {
		for (int i = 0; i < numCustomers; i++) {
			for (int j = 0; j < numDepots; j++) {
					p3 += vCost * x.at(i).at(j).at(k);
			}
		}
	}

	model.setObjective(p1 + p2 + p3, GRB_MINIMIZE);
	model.update();
}

void LRP::c1(GRBModel & model)
{
	for (int j = 0; j < numDepots; j++) {
		GRBLinExpr c1{ 0 };
		for (int k = 0; k < maxVehicles; k++) {
			for (int i = 0; i < numCustomers; i++) {
				c1 += x.at(i).at(j).at(k);
			}
		}
		model.addConstr(c1 == 1, "c1(" + to_string(j) + ")");
	}
	model.update();
}

void LRP::c2(GRBModel & model)
{
	for (int k = 0; k < maxVehicles; k++) {
		GRBLinExpr c2{ 0 };
		for (int j = 0; j < numDepots; j++) {
			for (int i = 0; i < numCustomers; i++) {
				c2 += cDemands.at(i) * x.at(i).at(j).at(k);
			}
		}
		model.addConstr(c2 <= vehicleCap, "c2(" + to_string(k) + ")");
	}
	model.update();
}

void LRP::c3(GRBModel & model)
{
	for (int k = 0; k < maxVehicles; k++) {
		GRBLinExpr c31{ 0 };
		GRBLinExpr c32{ 0 };
		// sum of edges leaving the node
		for (int i = 0; i < numCustomers; i++) {
			for (int j = 0; j < numDepots; j++) {
				c31 += x.at(i).at(j).at(k);
				c32 += x.at(j).at(i).at(k);
			}
		}
		model.addConstr(c31 - c32 == 0, "c3(" + to_string(k) + ")");
	}
	model.update();
}

void LRP::c4(GRBModel & model)
{
	for (int k = 0; k < maxVehicles; k++) {
		GRBLinExpr c4{ 0 };
		for (int i = 0; i < numCustomers; i++) {
			for (int j = 0; j < numDepots; j++) {
				c4 += x.at(i).at(j).at(k);
			}
		}
		model.addConstr(c4 <= 1, "c4(" + to_string(k) + ")");
	}
	model.update();
}

void LRP::c5(GRBModel & model) // subrout elimination
{
	for (int k = 0; k < maxVehicles; k++) {
		GRBLinExpr c5{ 0 };
		for (int i = 0; i < numCustomers; i++) {
			for (int j = 0; j < numDepots; j++) {
				c5 += x.at(i).at(j).at(k);
			}
		}
		model.addConstr(c5 <= 1, "c5(" + to_string(k) + ")");
	}
	model.update();
}

void LRP::c6(GRBModel & model)
{
	for (int k = 0; k < maxVehicles; k++) {
		for (int i = 0; i < numCustomers; i++) {
			for (int j = 0; j < numDepots; j++) {

				GRBLinExpr c61{ 0 };
				for (int u = 0; u < numCustomers; u++) {
					c61 += x.at(i).at(u).at(k);
				}
				GRBLinExpr c62{ 0 };
				for (int u = 0; u < numDepots; u++) {
					c61 += x.at(u).at(j).at(k);
				}
				model.addConstr(c61 + c62 <= 1, "c6(" + to_string(k) + ")");
			}
		}
	}
	model.update();
}
*/