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
	// calculate c, in barreto's model the costumers come before the depots, unlike prodhon's model
	coords.clear();
	for (auto i : cCoords) {
		coords.push_back(i);
	}
	for (auto i : dCoords) {
		coords.push_back(i);
	}
	// cij
	c.resize(V);
	for (int i = 0; i < V; i++) {
		c.at(i).resize(V, 0);
	}
	for (int i = 0; i < V; i++) {
		for (int j = 0; j < V; j++) {
			a = coords.at(i);
			b = coords.at(j);
			c.at(i).at(j) = distance(a, b);
		}
	}
	
	// calculate matrix cij and cik for barreto's model
	cij.resize(N);
	for (int i = 0; i < N; i++) {
		cij.at(i).resize(N, 0);
	}
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			a = cCoords.at(i);
			b = cCoords.at(j);
			// cout << "a: (" << a.x << "," << a.y << ")" << " - b: (" << b.x << "," << b.y << ")\n";
			// cout << "d: " << distance(a, b) << endl;
			cij.at(i).at(j) = distance(a, b);
		}
	}
	// cik
	cik.resize(N);
	for (int i = 0; i < N; i++) {
		cik.at(i).resize(P, 0);
	}
	for (int i = 0; i < N; i++) {
		for (int k = 0; k < P; k++) {
			a = cCoords.at(i);
			b = dCoords.at(k);
			cik.at(i).at(k) = distance(a, b);
		}
	}
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
	y.resize(I);
	for (int i = 0; i < I; i++) {
		y.at(i) = model.addVar(0, 1, 1, GRB_BINARY, "y(" + to_string(i) + ")");
	}
	model.update();
}

void LRP::varX(GRBModel & model)
{
	x.resize(V);
	for (int i = 0; i < V; i++) {
		x.at(i).resize(V);
		for (int j = 0; j < V; j++) {
			x.at(i).at(j).resize(K);
		}
	}
	for (int i = 0; i < V; i++) {
		for (int j = 0; j < V; j++) {
			for (int k = 0; k < K; k++) {
				//if(i != j)
				x.at(i).at(j).at(k) = model.addVar(0, 1, 0, GRB_BINARY, "x(" + to_string(i) + "," + to_string(j) + "," + to_string(k) + ")");
				//else
					//x.at(i).at(j).at(k) = model.addVar(0, 0, 0, GRB_CONTINUOUS, "x(" + to_string(i) + "," + to_string(j) + "," + to_string(k) + ")");
			}
		}
	}
	model.update();
}

void LRP::varF(GRBModel & model)
{
	f.resize(I);
	for (int i = 0; i < I; i++) {
		f.at(i).resize(J);
	}
	for (int i = 0; i < I; i++) {
		for (int j = 0; j < J; j++) {
			f.at(i).at(j) = model.addVar(0, 1, 1, GRB_BINARY, "f(" + to_string(i) + "," + to_string(j) + ")");
		}
	}
	model.update();
}

void LRP::varU(GRBModel & model)
{
	u.resize(V);
	for (int i = 0; i < V; i++) {
		u.at(i).resize(K);
	}
	for (int i = 0; i < V; i++) {
		for (int k = 0; k < K; k++) {
			u.at(i).at(k) = model.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "u(" + to_string(i) + "," + to_string(k) + ")");
		}
	}
	model.update();
}

void LRP::fo(GRBModel & model)
{
	GRBLinExpr p1{ 0 }, p2{ 0 }, p3{ 0 };

	// first part
	// cost of opening the facilities
	for (int i = 0; i < I; i++) {
		p1 += ocDepots.at(i) * y.at(i);
	}
	// second part
	// cost of travel
	for (int i = 0; i < V; i++) {
		for (int j = 0; j < V; j++) {
			for (int k = 0; k < K; k++) {
				if(i != j)
					p2 += distances.at(i).at(j) * x.at(i).at(j).at(k);
			}
		}
	}
	// third part
	// fixed cost of each vehicle used
	for (int k = 0; k < K; k++) { // k in K
		for (int i = 0; i < I; i++) { // i in I
			for (int j = I; j < V; j++) { // j in J
					p3 += vCost * x.at(i).at(j).at(k);
			}
		}
	}
	model.setObjective(p1 + p2 + p3, GRB_MINIMIZE);
	model.update();
}

// this set of constrins is the same from prodhon's (2) and barreto's model (6.37)
void LRP::c1(GRBModel & model)
{
	for (int j =  I; j < V; j++) { // for all j in J
		GRBLinExpr c1{ 0 };
		for (int k = 0; k < K; k++) { // for all k in K
			for (int i = 0; i < V; i++) { // for all i in V
				c1 += x.at(i).at(j).at(k);
			}
		}
		model.addConstr(c1 == 1, "c1(" + to_string(j-I) + ")");
	}
	model.update();
}

void LRP::c2(GRBModel & model)
{
	for (int k = 0; k < K; k++) { // for every vehicle
		GRBLinExpr c2{ 0 };
		for (int j = I; j < V; j++) { // for all j in J
			for (int i = 0; i < V; i++) { // for all i in V
				c2 += cDemands.at(j-I) * x.at(i).at(j).at(k);
			}
		}
		model.addConstr(c2 <= vehicleCap, "c2(" + to_string(k) + ")");
	}
	model.update();
}

// this set of constrains is the same from prodhon's (3) and berreto's (6.38) model
void LRP::c3(GRBModel & model) // maybe wrong
{
	for (int k = 0; k < K; k++) {
		for (int i = 0; i < V; i++) {
			GRBLinExpr c31{ 0 };
			GRBLinExpr c32{ 0 };
			for (int j = 0; j < V; j++) {
				c31 += x.at(i).at(j).at(k);
			}
			for (int j = 0; j < V; j++) {
				c32 += x.at(j).at(i).at(k);
			}
			model.addConstr(c31 - c32 == 0, "c3(" + to_string(k) + "," + to_string(i) + ")");
		}
	}
	model.update();
}

void LRP::c4(GRBModel & model)
{
	for (int k = 0; k < K; k++) {
		GRBLinExpr c4{ 0 };
		for (int i = 0; i < I; i++) {
			for (int j = I; j < V; j++) {
				c4 += x.at(i).at(j).at(k);
			}
		}
		model.addConstr(c4 <= 1, "c4(" + to_string(k) + ")");
	}
	model.update();
}

void LRP::c5(GRBModel & model) // subroute elimination
{
	for (int k = 0; k < K; k++) {
		for (int i = 0; i < V; i++) {
			for (int j = 0; j < V; j++) {
				if (i != j) {
					GRBLinExpr c5{ 0 };
					c5 = u.at(i).at(k) - u.at(j).at(k) + V * x.at(i).at(j).at(k);
					model.addConstr(c5 <= (V - 1), "c5(" + to_string(i) + "," + to_string(j) + "," + to_string(k) + ")");
				}				
			}
		}		
	}
	model.update();
}

void LRP::c6(GRBModel & model)
{	
	for (int i = 0; i < I; i++) { // for all i in I
		for (int j = I; j < V; j++) { // for all j in J
			for (int k = 0; k < K; k++) {
				GRBLinExpr c61{ 0 };
				for (int u = I; u < V; u++) { // u in J
					c61 += x.at(i).at(u).at(k);
				}
				GRBLinExpr c62{ 0 };
				for (int u = 0; u < V; u++) { // u in V\{j}
					if (u != j) {
						c62 += x.at(u).at(j).at(k);
					}						
				}
				model.addConstr(c61 + c62 <= 1 + f.at(i).at(j-I), "c6(" + to_string(i) + "," + to_string(j-I) + "," + to_string(k) + ")");
			}
		}		
	}
	model.update();
}

void LRP::c7(GRBModel & model)
{
	for (int i = 0; i < I; i++) {
		GRBLinExpr c7{ 0 };
		for (int j = I; j < V; j++) {
			c7 += cDemands.at(j-I) * f.at(i).at(j-I);
		}
		model.addConstr(c7 <= dCap.at(i) * y.at(i), "c7(" + to_string(i) + ")");
	}
	model.update();
}

void LRP::c8(GRBModel & model) // every client must be designated to only one depot
{
	for (int j = 0; j < J; j++) {
		GRBLinExpr c8{ 0 };
		for (int i = 0; i < I; i++) {			
			c8 += f.at(i).at(j);
		}
		model.addConstr(c8 == 1, "c8(" + to_string(j) + ")");
	}
	model.update();
}

void LRP::var_x_barreto(GRBModel & model)
{
	x_barreto.resize(V);
	for (int i = 0; i < V; i++) {
		x_barreto.at(i).resize(V);
		for (int j = 0; j < V; j++) {
			x_barreto.at(i).at(j).resize(R);
		}
	}
	for (int i = 0; i < V; i++) {
		for (int j = 0; j < V; j++) {
			for (int l = 0; l < R; l++) {
				x_barreto.at(i).at(j).at(l) = model.addVar(0, GRB_INFINITY, 0, GRB_INTEGER, "x(" + to_string(i) + "," + to_string(j) + "," + to_string(l) + ")");
			}
		}
	}
	model.update();
}

void LRP::var_xijl_barreto(GRBModel & model)
{
	xijl_barreto.resize(N);
	for (int i = 0; i < N; i++) {
		xijl_barreto.at(i).resize(N);
		for (int j = 0; j < N; j++) {
			xijl_barreto.at(i).at(j).resize(R);
		}
	}
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			for (int l = 0; l < R; l++) {
				xijl_barreto.at(i).at(j).at(l) = model.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "xijl(" + to_string(i) + "," + to_string(j) + "," + to_string(l) + ")");
			}
		}
	}
	model.update();
}

void LRP::var_xikl_barreto(GRBModel & model)
{
	xikl_barreto.resize(N);
	for (int i = 0; i < N; i++) {
		xikl_barreto.at(i).resize(P);
		for (int k = 0; k < P; k++) {
			xikl_barreto.at(i).at(k).resize(R);
		}
	}
	for (int i = 0; i < N; i++) {
		for (int k = 0; k < P; k++) {
			for (int l = 0; l < R; l++) {
				xikl_barreto.at(i).at(k).at(l) = model.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "xikl(" + to_string(i) + "," + to_string(k) + "," + to_string(l) + ")");
			}
		}
	}
	model.update();
}

void LRP::var_xkil_barreto(GRBModel & model)
{
	xkil_barreto.resize(P);
	for (int k = 0; k < P; k++) {
		xkil_barreto.at(k).resize(N);
		for (int i = 0; i < N; i++) {
			xkil_barreto.at(k).at(i).resize(R);
		}
	}
	for (int k = 0; k < P; k++) {
		for (int i = 0; i < N; i++) {
			for (int l = 0; l < R; l++) {
				xkil_barreto.at(k).at(i).at(l) = model.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "xkil(" + to_string(k) + "," + to_string(i) + "," + to_string(l) + ")");
			}
		}
	}
	model.update();
}

void LRP::var_y_barreto(GRBModel & model)
{
	y_barreto.resize(P);
	for (int k = 0; k < P; k++) {
		y_barreto.at(k) = model.addVar(0, 1, 0, GRB_BINARY, "y(" + to_string(k) + ")");
	}
	model.update();
}

void LRP::var_u_barreto(GRBModel & model)
{
	u_barreto.resize(V);
	for (int i = 0; i < V; i++) {
		u_barreto.at(i).resize(R);
	}
	for (int i = 0; i < V; i++) {
		for (int l = 0; l < R; l++) {
			u_barreto.at(i).at(l) = model.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "u(" + to_string(i) + "," + to_string(l) + ")");
		}
	}
	model.update();
}

void LRP::fo_barreto(GRBModel & model)
{
	// first portion
	GRBLinExpr p1{ 0 };
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			if (i != j) {
				for (int l = 0; l < R; l++) {
					p1 += c.at(i).at(j) * x_barreto.at(i).at(j).at(l);
				}
			}	
		}
	}
	// second portion
	GRBLinExpr p2{ 0 };
	GRBLinExpr p21{ 0 };
	GRBLinExpr p22{ 0 };
	for (int i = 0; i < N; i++) {
		for (int k = N; k < N + P; k++) {
			for (int l = 0; l < R; l++) {
				p2 += (c.at(i).at(k) * x_barreto.at(i).at(k).at(l)) + (c.at(k).at(i) * x_barreto.at(k).at(i).at(l));
				//p21 += cik.at(i).at(k) * x.at(i).at(k).at(l);
				//p22 += cik.at(k).at(i) * x.at(k).at(i).at(l);
			}
		}
	} 
	// third portion
	GRBLinExpr p3{ 0 };
	for (int k = N; k < N + P; k++) {
		p3 += ocDepots.at(k-N) * y_barreto.at(k-N);
	}
	model.setObjective(p1 + p2 + p3, GRB_MINIMIZE);
}

void LRP::c1_barreto(GRBModel & model)
{
	for (int i = 0; i < N; i++) { // 
		GRBLinExpr c1{ 0 };
		for (int j = 0; j < N + P; j++) { // 
			for (int l = 0; l < R; l++) { // 
				c1 += x_barreto.at(i).at(j).at(l);
			}
		}
		model.addConstr(c1 == 1, "c1(" + to_string(i) + ")");
	}
	model.update();
}

void LRP::c2_barreto(GRBModel & model)
{
	for (int g = 0; g < N + P; g++) {
		for (int l = 0; l < R; l++) {
			GRBLinExpr c21{ 0 };
			GRBLinExpr c22{ 0 };
			for (int i = 0; i < N + P; i++) {
				c21 += x_barreto.at(i).at(g).at(l);
			}
			for (int j = 0; j < N + P; j++) {
				c22 += x_barreto.at(g).at(j).at(l);
			}
			model.addConstr(c21 - c22 == 0, "c2(" + to_string(g) + "," + to_string(l) + ")");
		}
	}
	model.update();
}

void LRP::c3_barreto(GRBModel & model)
{
	for (int k = N; k < N + P; k++) {
		GRBLinExpr c3{ 0 };
		for (int i = 0; i < N; i++) {
			for (int l = 0; l < R; l++) {
				c3 += x_barreto.at(i).at(k).at(l);
			}
		}
		model.addConstr(c3 <= (float(dCap.at(k - N)) / float(vehicleCap)) * y_barreto.at(k - N), "c3(" + to_string(k - N) + ")");
	}
	model.update();
}

void LRP::c4_barreto(GRBModel & model)
{
	GRBLinExpr c41{ 0 };
	for (int i = 0; i < N; i++) {
		for (int k = N; k < P + N; k++) {
			for (int l = 0; l < R; l++) {
				c41 += x_barreto.at(i).at(k).at(l);
			}
		}
	}
	GRBLinExpr c42{ 0 };
	for (int i = 0; i < N; i++) {
		c42 += float(cDemands.at(i));
	}
	c42 /= float(vehicleCap);
	model.addConstr(c41 >= c42, "c4");
	model.update();
}

void LRP::c5_barreto(GRBModel & model)
{
	for (int l = 0; l < R; l++) {
		GRBLinExpr c51{ 0 };
		for (int i = 0; i < N; i++) {
			for (int j = 0; j < N; j++) {
				if (i != j)
					c51 += cDemands.at(i) * x_barreto.at(i).at(j).at(l);
			}
		}
		GRBLinExpr c52{ 0 };
		for (int i = 0; i < N; i++) {
			for (int k = 0; k < P; k++) {
				c52 += cDemands.at(i) * x_barreto.at(i).at(k).at(l);
			}
		}
		model.addConstr(c51 + c52 <= vehicleCap, "c5(" + to_string(l) + ")");
	}
	model.update();
}

void LRP::c6_barreto(GRBModel & model)
{
	for (int l = 0; l < R; l++) { // for each route
		for (int i = 0; i < V; i++) { // for each client
			for (int j = 0; j < V; j++) {
				if (i != j) {
					GRBLinExpr c6{ 0 };
					c6 += u_barreto.at(i).at(l) - u_barreto.at(j).at(l) + V * x_barreto.at(i).at(j).at(l);
					model.addConstr(c6 <= (V - 1), "c6(" + to_string(i) + "," + to_string(j) + "," + to_string(l) + ")");
				}
			}
		}
	}
	model.update();
}

void LRP::prodhonModel(GRBModel & model)
{
	wmodel = 1;
	varF(model);
	varX(model);
	varY(model);
	varU(model);
	fo(model);
	c1(model);
	c2(model);
	c3(model);
	c4(model);
	// c5(model);
	c6(model);
	c7(model);
	//c8(model);
}

void LRP::barretoModel(GRBModel & model)
{
	wmodel = 2;
	var_x_barreto(model);
	//var_xijl_barreto(model);
	//var_xikl_barreto(model);
	//var_xkil_barreto(model);
	var_y_barreto(model);
	var_u_barreto(model);
	fo_barreto(model);
	c1_barreto(model);
	c2_barreto(model);
	c3_barreto(model);
	//c4_barreto(model); // infisable
	c5_barreto(model);
	c6_barreto(model);
}

float LRP::distance(coordinate a, coordinate b)
{
	return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
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

		prodhonModel(model);
		//barretoModel(model);

		model.write(this->directory + this->fileName + ".lp");
		model.getEnv().set(GRB_DoubleParam_TimeLimit, TMAX);
		model.optimize();

		result(model);
		model.write("teste.sol");
		
		getData(model);
	}
	catch (GRBException e) {
		cout << "Error code = " << e.getErrorCode() << endl;
		cout << e.getMessage() << endl;
	}
	catch (exception e) {
		cout << e.what() << endl;
	}
}

void LRP::getData(GRBModel & model)
{
	int a = model.get(GRB_IntAttr_Status);
	if (a == GRB_OPTIMAL || a == GRB_TIME_LIMIT) {
		if (wmodel == 1) { // prodhon
			fstream op;
			op.open(directory + fileName + ".sol", ios::out);
			if (op.is_open() == false) {
				cout << "Error creating file " + fileName << endl;
				cout << "On directory " + directory << endl;
				exit(1);
			}
			op << "fo: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
			//op << numDepots << " " << numCustomers << " " << maxVehicles << endl;
			// var x
			for (int k = 0; k < K; k++) { // for each vehicle
				op << "Vehicle " << k << ": " << endl;
				for (int i = 0; i < V; i++) { // for each depot
					for (int j = 0; j < V; j++) {
						op << x.at(i).at(j).at(k).get(GRB_DoubleAttr_X) << " ";
					}
					op << endl;
				}
			}
			// var f
			op << "f: \n";
			for (int i = 0; i < I; i++) {
				for (int j = 0; j < J; j++) {
					op << f.at(i).at(j).get(GRB_DoubleAttr_X) << " ";
				}
				op << endl;
			}
			// var y
			op << "y: \n";
			for (int i = 0; i < I; i++) {
				op << y.at(i).get(GRB_DoubleAttr_X) << "\n";
			}
			op.close();
		}
		else if (wmodel == 2) { // barreto
			fstream op;
			op.open(directory + fileName + ".sol", ios::out);
			if (op.is_open() == false) {
				cout << "Error creating file " + fileName << endl;
				cout << "On directory " + directory << endl;
				exit(1);
			}
			op << "fo: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
			//op << numDepots << " " << numCustomers << " " << maxVehicles << endl;
			// var x
			for (int k = 0; k < K; k++) { // for each vehicle
				op << "Vehicle " << k << ": " << endl;
				for (int i = 0; i < V; i++) { // for each depot
					for (int j = 0; j < V; j++) {
						op << x_barreto.at(i).at(j).at(k).get(GRB_DoubleAttr_X) << " ";
					}
					op << endl;
				}
			}
			// var y
			op << "y: \n";
			for (int i = 0; i < P; i++) {
				op << y_barreto.at(i).get(GRB_DoubleAttr_X) << "\n";
			}
			op.close();
		}
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
	cout << "Deposit cost: ";
	for (auto i : ocDepots) {
		cout << i <<  " ";
	}
	cout << endl;
	cout << "Vehicle cost: " << vCost << endl;
	cout << "Type: " << type << endl;
	cout << "Distances: \n";
	for (auto i : distances) {
		for (auto j : i){
			cout << fixed << setw(6) << setprecision(2) << j << " ";
		}
		cout << endl;
	}
	cout << "C: \n";
	for (auto i : c) {
		for (auto j : i) {
			cout << fixed << setw(6) << setprecision(2) << j << " ";
		}
		cout << endl;
	}
	cout << "Cij: \n";
	for (auto i : cij) {
		for (auto j : i) {
			cout << fixed << setw(6) << setprecision(2) << j << " ";
		}
		cout << endl;
	}
	cout << "Cik: \n";
	for (auto i : cik) {
		for (auto j : i) {
			cout << fixed << setw(6) << setprecision(2) << j << " ";
		}
		cout << endl;
	}
	cout << endl;
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