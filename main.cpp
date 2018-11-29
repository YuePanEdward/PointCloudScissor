//By Pan Yue etal. @WHU
#include "utility.h"
#include "process.h"


using namespace std;
using namespace utility;


int main()
{

	cout << "!----------------------------------------------------------------------------!" << endl;
	cout << "!                             Point Cloud Scissor                            !" << endl;
	cout << "!                                 by Yue Pan                                 !" << endl;
	cout << "!----------------------------------------------------------------------------!" << endl;

	/*----------------------- 1. Data Input-------------------------*/

	string filename;
	double IoU;
	int axis;
	cout << "Input pcd Point Cloud file" << endl;
	cin >> filename;
	cout << "Input IoU" << endl;
	cin >> IoU;
	cout << "Cut along which axis? Choose from 1.X 2.Y 3.Z, please enter 1,2 or 3" << endl;
	cin >> axis;
	Process pro;
	pcXYZPtr pointCloud(new pcXYZ());
	pro.readPcdFileXYZ(filename, pointCloud);

	/*---------------- 2. Point Cloud Sorting----------------------*/
	pcXYZPtr pCsort(new pcXYZ());
	vector<double> cor;
	vector<int> corkey;
	pro.presort(pointCloud, axis, cor, corkey);
	pro.quicksort(cor, corkey, 0, pointCloud->size()-1);
	pro.saveindices(corkey, pointCloud, pCsort);
	
	/*--------------------- 3. Data Cutting------------------------*/
	pcXYZPtr pCSeg1(new pcXYZ());
	pcXYZPtr pCSeg2(new pcXYZ());
	pro.cut(pCsort, IoU, pCSeg1, pCSeg2);
	pro.writePcdFileXYZ("PCSeg1.pcd", pCSeg1);
	pro.writePcdFileXYZ("PCSeg2.pcd", pCSeg2);
	pro.display(pCSeg1, pCSeg2);
	cout << "Output Done. Have a nice day." << endl;
	
	int endok;
	cin >> endok;
	return 1;

}

	