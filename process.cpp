#include "process.h"
#include "utility.h"

#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <fstream>
#include <vector>
#include <pcl/io/pcd_io.h>


using namespace  std;
using namespace  utility;


bool Process::readPcdFileXYZ(const std::string &fileName, const pcXYZPtr &pointCloud)
{
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *pointCloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file\n");
		return false;
	}
	cout << "Data loaded" << endl;
	cout << "Raw point number:" << pointCloud->size() << endl;
	return true;
}


bool Process::writePcdFileXYZ(const string &fileName, const pcXYZPtr &pointCloud)
{
	if (pcl::io::savePCDFileBinary(fileName, *pointCloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't write file\n");
		return false;
	}
	return true;
}


void Process::display(const pcXYZPtr &cloudS, const pcXYZPtr &cloudT)
{

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(255, 255, 255);
	char t[256];
	string s;
	int n = 0;

	pcXYZRGBPtr pointcloudS(new pcXYZRGB());
	pcXYZRGBPtr pointcloudT(new pcXYZRGB());

	for (size_t i = 0; i < cloudT->points.size(); ++i)
	{
		pcl::PointXYZRGB pt;
		pt.x = cloudT->points[i].x;
		pt.y = cloudT->points[i].y;
		pt.z = cloudT->points[i].z;
		pt.r = 0;
		pt.g = 0;
		pt.b = 255;
		pointcloudT->points.push_back(pt);
	}

	viewer->addPointCloud(pointcloudT, "pointcloudT");

	for (size_t i = 0; i < cloudS->points.size(); ++i)
	{
		pcl::PointXYZRGB pt;
		pt.x = cloudS->points[i].x;
		pt.y = cloudS->points[i].y;
		pt.z = cloudS->points[i].z;
		pt.r = 255;
		pt.g = 0;
		pt.b = 0;
		pointcloudS->points.push_back(pt);
	}
	viewer->addPointCloud(pointcloudS, "pointcloudS");

	cout << "Click X(close) to continue..." << endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void Process::presort(const pcXYZPtr &pointCloud, int axis, vector<double> &cor, vector<int> &corkey)
{
	for (size_t i = 0; i < pointCloud->size(); i++)
	{
		switch(axis){
		case 1:{cor.push_back(pointCloud->points[i].x); break; }
		case 2:{cor.push_back(pointCloud->points[i].y); break; }
		case 3:{cor.push_back(pointCloud->points[i].z); break; }
		default: cout << "A problem encountered." << endl;
		}
		corkey.push_back(i);
	}
}

void Process::quicksort(vector<double> &data, vector<int> &datakey, int left, int right)
{
	if (left >= right)
		return;
	
	int pivot = (left + right) / 2;
	
	vector<double> less;
	vector<double> greater;
	vector<int> lesskey;
	vector<int> greaterkey;

	double tmp;
	int tmpkey;

	for (int i = left; i < right; i++)
	{
		if (i == pivot)
		{
			tmp = data[pivot];
			tmpkey = datakey[pivot];
		}

		else if (data[i] < data[pivot])
		{
			less.push_back(data[i]);
			lesskey.push_back(datakey[i]);
		}

		else
		{
			greater.push_back(data[i]);
			greaterkey.push_back(datakey[i]);
		}
	
	}

	for (int i = left; i < right; i++)
	{
		if (i < left + less.size())
		{
			data[i] = less[less.size() - i - 1 + left];
			datakey[i] = lesskey[less.size() - i - 1 + left];
		}
			
		else if (i == left + less.size())
		{
			data[i] = tmp;
			datakey[i] = tmpkey;
			pivot = i;
		}
		else
		{
			data[i] = greater[greater.size() - i + less.size() + left];
			datakey[i] = greaterkey[greater.size() - i + less.size() + left];
		}
	}
	quicksort(data, datakey, left, pivot);
	quicksort(data, datakey, pivot + 1, right);

}
void Process::saveindices(vector<int> &corkey, const pcXYZPtr &pCOriginal, const pcXYZPtr &pCSort)
{
	pCSort->width = corkey.size();
	pCSort->height = 1;
	pCSort->resize(pCSort->width*pCSort->height);
	for (size_t i = 0; i < corkey.size(); i++)
	{
		pCSort->points[i] = pCOriginal->points[corkey[i]];
	}
}

void Process::cut(const pcXYZPtr &pointCloud, double IoU, const pcXYZPtr &pCSeg1, const pcXYZPtr &pCSeg2)
{
	int indice_down, indice_up;
	indice_down = (0.5 - IoU / 2)*pointCloud->size();
	indice_up = (0.5 + IoU / 2)*pointCloud->size();
	
	
	pCSeg1->width = indice_up;
	pCSeg2->width = pointCloud->size()-indice_down;

	pCSeg1->height = 1;
	pCSeg2->height = 1;

	pCSeg1->resize(pCSeg1->width*pCSeg1->height);
	pCSeg2->resize(pCSeg2->width*pCSeg2->height);
	
	for (size_t i = 0; i < indice_up; i++)
	{
		pCSeg1->points[i] = pointCloud->points[i];
	}

	for (size_t i = indice_down ; i < pointCloud->size(); i++)
	{
		pCSeg2->points[i-indice_down] = pointCloud->points[i];
	}
	cout << "Data Cut Done" << endl;
	cout << "Seg1 point number:" << pCSeg1->size() << endl;
	cout << "Seg2 point number:" << pCSeg2->size() << endl;
}