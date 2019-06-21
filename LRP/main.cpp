#include <iostream>
#include "LRP.h"

using namespace std;

const string dir = "D:/Victor/Pos-Graduacao/UFV/2 periodo/INF682/Trabalhos/02/Instances_Prodhon_LRP/";
const string inst = "coord20-5-1.dat";

int main() {
	LRP lrp(dir + "test01.txt");
	lrp.printData();
	lrp.model();
	return 0;
}
