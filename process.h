#ifndef PROCESS
#define PROCESS

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

#include <vector>
#include "utility.h"

using namespace utility;
using namespace std;

class Process
{
public:
	// pcd ÎÄ¼þ¶ÁÐ´;

	bool readPcdFileXYZ(const string &fileName, const pcXYZPtr &pointCloud);
	bool writePcdFileXYZ(const string &fileName, const pcXYZPtr &pointCloud);
	void display(const pcXYZPtr &cloudS, const pcXYZPtr &cloudT);
	void quicksort(vector<double> &data, vector<int> &datakey, int left, int right);
	void presort(const pcXYZPtr &pointCloud, int axis, vector<double> &cor, vector<int> &corkey);
	void saveindices(vector<int> &corkey, const pcXYZPtr &pCOriginal, const pcXYZPtr &pCSort);
	void cut(const pcXYZPtr &pointCloud, double IoU, const pcXYZPtr &pCSeg1, const pcXYZPtr &pCSeg2);

protected:

private:

};


#endif