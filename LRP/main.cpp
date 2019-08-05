#include <iostream>
#include "PI.h"

using namespace std;
const string dir = "D:/Victor/Pos-Graduacao/UFV/Project/Instances/Instances_Prodhon_LRP/";
const string a = "test04.txt";
const string b = "coord20-5-1.dat";

int main() {
	//LRP lrp(dir + a);
	//lrp.printData();
	//lrp.model();
	PI prodhon(dir + a);
	prodhon.printData();
	return 0;
}
