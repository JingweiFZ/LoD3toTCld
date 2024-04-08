// LoD3toPtCld.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#define _CRT_SECURE_NO_WARNINGS
//#include "stdafx.h"

#include <iostream>
#include "globle_para.hpp"

#include "utils.hpp"
#include "LoD3label_utils.hpp"
#include <chrono>
//#include "util.h"

using namespace std;
using namespace pcl;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZRGBL PointL;
typedef pcl::PointCloud<pcl::PointXYZRGBL> PCXYZRGBL;
typedef pcl::PointCloud<pcl::PointXYZRGBL>::Ptr PCXYZRGBLPtr;
typedef pcl::PointCloud<pcl::PointXYZ> PCXYZ;

//***** main folder: D:\vs_code\ThermalAnalysis
//extern string data_folder = "D:/data_folder/MLS2016/";
//extern string save_folder = "D:/Voxelization/output/";
string filepara = "label_T_parameters.txt";

int main()
{
	std::printf("/****************start the labeling of thermal point clouds***************/n");
	auto start = std::chrono::high_resolution_clock::now();

    std::vector<string> parameters;
    
    loadParameters(filepara, parameters);//load parameters from file

    string save_dir = parameters[9];
    string temp_dir = parameters[12];

    //read point clouds from LoD3 model
	std::printf("/*** load LoD3 model point clouds from file: %s \n", parameters[14].c_str());
    PCXYZRGBLPtr lod3(new PCXYZRGBL);

    //read las file point clouds
	string outpcdfilename = save_dir+"lod3.pcd";
    lasTxt2pcd(parameters[14], outpcdfilename);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBL>(outpcdfilename, *lod3) == -1) {
        std::cerr << "Couldn't read file input_file.las." << std::endl;
        return -1;
    }
    else {
		std::printf("\t finish reading LoD3 model point clouds from file: %s \n", parameters[14].c_str());
    }

    //read point clouds from thermal point clouds file
	std::printf("/*** load thermal point clouds from file: %s \n", parameters[16].c_str());
    PCXYZRGBLPtr Tcloud(new PCXYZRGBL);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBL>(parameters[16].c_str(), *Tcloud) == -1) {
        std::cerr << "Couldn't read file input_file.las." << std::endl;
        return -1;
    }
    else {
		std::printf("\t finish reading Thermal point clouds from file: %s \n", parameters[14].c_str());
    }

    //co-registration of thermal point clouds and LoD3 model point clouds
	std::printf("/*** co-registration of thermal point clouds and LoD3 model point clouds \n");

    //transformation and rendering the thermal point clouds
	std::printf("/*** transformation and rendering the thermal point clouds \n");

	Eigen::Matrix4d Mtx;
	readTMatrix(parameters[55], Mtx);

	PCXYZRGBLPtr Tlod3(new PCXYZRGBL);
	string transformedtxt = save_dir + "transformed_pcd.txt";
	transformTxt(parameters[14],Mtx,transformedtxt);
	//pcl::transformPointCloud(*lod3, *Tlod3, Mtx);
	txt2pcd(transformedtxt, Tlod3);
	if (Tlod3->points.size() > 0) {
		pcl::io::savePCDFileBinary(save_dir + "Tlod3.pcd", *Tlod3);
		printf("\t save transformed point clouds to %s\n", save_dir + "Tlod3.pcd");
	}
	else {
		printf("\t Error: cannot transform the point clouds!");
		system("pause");
	}
	
    //label the thermal point clouds
	/*******************************************************/
	/****************label thermal point clouds *****************/
	/*******************************************************/
	cv::Mat lut(1, 256, CV_8UC3);
	// 创建 LUT，根据像素值设置颜色
	for (int i = 0; i < 256; ++i) {
		if (i >= 1 && i <= 5) {
			if (i == 1) {
				lut.at<cv::Vec3b>(0, i) = cv::Vec3b(17, 89, 198); // Blue
			}
			else if (i == 2) {
				lut.at<cv::Vec3b>(0, i) = cv::Vec3b(219, 169, 142); // Green
			}
			else if (i == 3) {
				lut.at<cv::Vec3b>(0, i) = cv::Vec3b(204, 242, 255); // Red
			}
			else if (i == 4) {
				lut.at<cv::Vec3b>(0, i) = cv::Vec3b(0, 0, 168); // Cyan
			}
			else if (i == 5) {
				lut.at<cv::Vec3b>(0, i) = cv::Vec3b(255, 0, 255); // Magenta
			}
		}
		else {
			lut.at<cv::Vec3b>(0, i) = cv::Vec3b(0, 0, 0); // 其他像素值设为黑色
		}
	}
	/*******look up table*************************/
	std::printf("/********* label the thermal point clouds *********\n");
	pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cld(new pcl::PointCloud<pcl::PointXYZRGBL>);
	pcl::KdTreeFLANN<pcl::PointXYZRGBL> kdtree;

	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;

	lod3->clear();
	/*printf("load reference file...");
	string loadref = parameters[58];
	if (pcl::io::loadPCDFile(loadref, *Tlod3) != -1) {
		printf("load reference point cloud %s\n", loadref.c_str());
		printf("\t point cloud resolution: %s\n", parameters[50].c_str());
		printf("\t set threshold: %s\n", parameters[52].c_str());
	}
	else {
		printf("Error: cannot load reference point cloud %s\n", parameters[14].c_str());
		system("pause");
		exit(0);
	}*/
	float threshold = atof(parameters[52].c_str());
	std::printf("\t set threshold: %f\n", threshold);

	kdtree.setInputCloud(Tlod3);
	pcl::PointXYZRGBL pt;
	int count = 0;
	for(int i=0;i<Tcloud->points.size();i++){
		pt.x = Tcloud->points[i].x;
		pt.y = Tcloud->points[i].y;
		pt.z = Tcloud->points[i].z;
		pt.r = Tcloud->points[i].r;
		pt.g = Tcloud->points[i].g;
		pt.b = Tcloud->points[i].b;

		if (kdtree.nearestKSearch(pt, 1, pointIdxNKNSearch, pointNKNSquaredDistance) <= 0 ||
			pointNKNSquaredDistance[0] > threshold)
		{
			Tcloud->points[i].label = 0;
			//cout << pointNKNSquaredDistance[0]<<"- ";
		}
		else {
			Tcloud->points[i].label = Tlod3->points[pointIdxNKNSearch[0]].label;
			pt.a = Tlod3->points[pointIdxNKNSearch[0]].label;
			pt.rgb = Tlod3->points[pointIdxNKNSearch[0]].rgb;
			cld->push_back(pt);
			count++;
		}
		cld->push_back(pt);
	}
	cld->height = 1;
	cld->width = cld->points.size();
	cld->is_dense = true;
	pcl::io::savePCDFileASCII(save_dir + "pure_" + parameters[18].c_str(), *cld);
	//pcl::copyPointCloud(*cld, *Tcloud);
	
	cld->clear();
	std::printf("\t label the thermal point clouds: %d  out of %d\n", count,Tcloud->points.size());
	pcl::io::savePCDFileASCII(save_dir+ parameters[18].c_str(), *Tcloud);
	cout<<"\t saved to: "<<save_dir+ parameters[18].c_str()<<endl;
	
	if (Tcloud->points.size() == 0) {
		std::printf("/t No matched point clouds!");
		system("pause");
	}
	std::printf("save the labeled thermal point clouds to file: %s\n", save_dir + parameters[18].c_str());

	// 获取结束时间点
	auto stop = std::chrono::high_resolution_clock::now();

	// 计算持续时间
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

	// 输出运行时间
	std::cout << "Time taken by function: " << duration.count() << " milliseconds" << std::endl;

    //label the images
	/*******************************************************/
	/****************label images *****************/
	/*******************************************************/
	std::printf("/*** label the images \n");

	string pos_file=parameters[32].c_str();
	string img_folder = parameters[21].c_str();
	int min_img= atoi(parameters[27].c_str());
	int max_img= atoi(parameters[29].c_str());

	// read pos information
	vector<Pos> vPosset_;
	readPos(pos_file,min_img, max_img, vPosset_);
	int xx=atoi(parameters[46].c_str());
	int yy=atoi(parameters[47].c_str());

	//iterative all the images
	char buffer[100];
	for (int i = min_img; i < max_img; i++) {
		string img_name = parameters[21];
		string name = parameters[23];
		std::sprintf(buffer, "%05d", i);
		name.append(buffer);
		string save_img_name = save_dir;
		save_img_name.append("images\/");
		save_img_name.append(name);
		save_img_name.append("_label.png");
		img_name.append(name);
		img_name.append(parameters[25]);

		cv::Mat img = cv::imread(img_name);
		if (img.empty()) {
			std::printf("\t Error: cannot load image %s\n", img_name.c_str());
			continue;
		}
		else {
			std::printf("\t load image %s\n", img_name.c_str());
		}

		cv::Mat save_img(img.rows,img.cols, CV_8UC3, cv::Scalar(0, 0, 0));

		//cv::cvtColor(img, save_img, cv::COLOR_GRAY2BGR);
		img.copyTo(save_img);
		if (img.empty()) {
			std::printf("Error: cannot load image %s\n", img_name.c_str());
			system("pause");
			exit(0);
		}
		else {
			std::printf("\t load image %s\n", img_name.c_str());
		}

		//get the initial parameters of the camera
		float dx, dy, dz;
		float rx, ry, rz;
		float cx, cy;
		float f;


		dx = atof(parameters[34].c_str()); dy = atof(parameters[35].c_str()); dz = atof(parameters[36].c_str());
		rx = atof(parameters[38].c_str()); ry = atof(parameters[39].c_str()); rz = atof(parameters[40].c_str());
		cx = atof(parameters[42].c_str()); cy = atof(parameters[43].c_str());
		f = atof(parameters[44].c_str());

		//parameters for the calculation
		Eigen::Matrix4d R_az;
		Eigen::Matrix4d R_ele;
		Eigen::Matrix4d R_rot;
		Eigen::Matrix4d R_int;
		Eigen::Matrix4d Rx;
		Eigen::Matrix4d Ry;
		Eigen::Matrix4d Rz;
		Eigen::Matrix4d R;
		Eigen::Vector4d pos;
		Eigen::Vector4d pos2;
		Eigen::Vector4d p;
		Eigen::Vector4d tt;
		int ind1;
		int ind2;
		int xi, yi;
		int pointsize = 1;
		int avggray ;
		int avggraycount;

		Dx(rx * (M_PI / 180.0) + M_PI, R_rot);						// boresight angles of IR camera
		Dy(ry * (M_PI / 180.0), R_ele);
		Dz(rz * (M_PI / 180.0), R_az);
		R_int = R_rot * R_ele * R_az;


		Pos Ps = vPosset_[i - min_img];
		Dx(Ps.R.x * M_PI / 180.0, Rx);
		Dy(-Ps.R.y * M_PI / 180.0, Ry);
		Dz(Ps.R.z* M_PI / 180.0, Rz);
		R = Rx * Ry * Rz;

		pos(0) = Ps.T.x; pos(1) = Ps.T.y; pos(2) = Ps.T.z; pos(3) = 1.0;
		pos2 -= pos;
		if (pos2.norm() < 0.02) ind2 = 0;
		else ind2 = 1;

		//construct the matrix to store the range of the points
		std::vector<std::vector<float>> myrange(img.rows, std::vector<float>(img.cols, -1.0f));
		//label the image
		for (int j = 0; j < Tcloud->points.size(); j++) {
				
			ind1 = 1;
			if (!ind2) ind1 = 0;
			p(0) = Tcloud->points[j].x - pos(0);
			p(1) = Tcloud->points[j].y - pos(1);
			p(2) = Tcloud->points[j].z - pos(2);
			p(3) = 1.0;

			p(3) = p(0);									// conversion ENU to NED
			p(0) = p(1);
			p(2) = -p(2);
			p(1) = p(3);
			p(3) = 1.0;
			tt = R * p;
			tt(0) -= dx;
			tt(1) -= dy;
			tt(2) -= dz;
			tt = R_int * tt;
			float x = 0.0;
			float y = 0.0;
			if (tt(0) > 1.0) {								// projection of 3D points to focal plane
				if (tt(0) > 80.0) {
					ind1 = 0;
				}
				x = cx - f * tt(2) / tt(0);
				y = cy - f * tt(1) / tt(0);
				xi = (int)x;
				yi = (int)y;
				if (ind1) {
					if ((xi > pointsize) && (xi < (xx - 1 - pointsize)) && (yi > pointsize) && (yi < (yy - 1 - pointsize))) {
						//label_img.at<uchar>(xi, yi) = Tcloud->points[j].label;
						//calculate the distance
						if(myrange[xi][yi]<0){myrange[xi][yi]=tt(0);}
						else if(myrange[xi][yi]>tt(0) ) {//&& Tcloud->points[j].label>0
							myrange[xi][yi]=tt(0); save_img.at<cv::Vec3b>(xi, yi) = lut.at<cv::Vec3b>(0, Tcloud->points[j].label);} 
					}
				}
			}
			
			

			/*PointT pt = Tcloud->points[j];
			int x = (int)pt.x;
			int y = (int)pt.y;
			int z = (int)pt.z;
			int label = (int)pt.label;
			if (label == 0) {
				continue;
			}
			else {
				save_img.at<Vec3b>(y, x)[0] = 0;
				save_img.at<Vec3b>(y, x)[1] = 0;
				save_img.at<Vec3b>(y, x)[2] = 255;
			}*/
		}

		cv::imwrite(save_img_name, save_img);
		std::printf("\t save image %s\n", save_img_name.c_str());
	}
	// 获取结束时间点
	stop = std::chrono::high_resolution_clock::now();

	// 计算持续时间
	duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

	// 输出运行时间
	std::cout << "Time taken by function: " << duration.count() << " milliseconds" << std::endl;

	std::printf("/*** save the labeled images \n\n");
	std::printf(" /****************end the labeling of thermal point clouds******\n");


}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
