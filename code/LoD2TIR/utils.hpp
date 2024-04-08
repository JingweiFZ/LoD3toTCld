
#include "globle_para.hpp"

using namespace std;
using namespace pcl;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZRGBL PointL;

//for CGAl
// Type declarations.

bool isFileExtension(const std::string& fileName, const std::string& fileExtension) {
	// 查找文件名中最后一个点的位置
	size_t dotPosition = fileName.find_last_of('.');
	if (dotPosition == std::string::npos) {
		// 如果找不到点，则文件名没有后缀
		return false;
	}

	// 提取文件后缀并转换为小写
	std::string extractedExtension = fileName.substr(dotPosition + 1);
	std::transform(extractedExtension.begin(), extractedExtension.end(), extractedExtension.begin(), ::tolower);

	// 比较文件后缀与指定后缀
	return extractedExtension == fileExtension;
}

int imgcalibration( vector<string> parameters) {
	/// <summary>
	/// Geometric calibration of the images
	/// </summary>
	/// <param name="img"></param>
	/// <param name="out"></param>
	/// <param name="parameters"></param>
	/// <returns></returns>
	printf("image calibrtaion...\t");
	cv::Mat cameraMatrix_;
	cv::Mat newcameraMatrix_;
	cv::Mat distCoeffs_;
	cv::Size imageSize_;
	int height_, width_;
	
	cv::Mat map1;
	cv::Mat map2;
	string imname;
	string temp_name;
	cv::Mat image, image0, image1;
	float fx_, fy_, fz_, cx_, cy_;
	float q1_, q2_, k1_, k2_, k3_;
	fx_ = atof(parameters[36].c_str());
	fy_ = atof(parameters[38].c_str());
	cx_ = atof(parameters[40].c_str());
	cy_ = atof(parameters[42].c_str());
	q1_ = atof(parameters[52].c_str());
	q2_ = atof(parameters[53].c_str());
	k1_ = atof(parameters[48].c_str());
	k2_ = atof(parameters[49].c_str());
	k3_ = atof(parameters[50].c_str());
	width_ = atoi(parameters[46].c_str());
	height_ = atoi(parameters[44].c_str());

	cameraMatrix_ = (cv::Mat1d(3, 3) << fx_, 0.0, cy_, 0.0, fy_, cx_, 0.0, 0.0, 1.0);	// optical axis (estimated) in image center 
	distCoeffs_ = (cv::Mat1d(1, 5) << k1_, k2_, q1_, q2_, -k3_);												// distortion of IR camera (estimated)
	imageSize_.width = width_;//yy
	imageSize_.height = height_;//xx
	newcameraMatrix_ = getOptimalNewCameraMatrix(cameraMatrix_, distCoeffs_, imageSize_, 0, imageSize_, 0, true);
	cv::initUndistortRectifyMap(cameraMatrix_, distCoeffs_, cv::Mat(), newcameraMatrix_, imageSize_, CV_32FC1, map1, map2);

	string img_folder = parameters[24].c_str();
	string save_folder = parameters[2].c_str();
	int startIm = atoi(parameters[27].c_str());
	int endIm = atoi(parameters[28].c_str());
	char buffer[100];
	int inv = atoi(parameters[11].c_str());
	for (int im_f = startIm; im_f < endIm; im_f=im_f+inv) {
		string imname = img_folder;
		imname.append("img_");
		sprintf_s(buffer, "%05d", im_f);
		imname.append(buffer);
		imname.append(".tif");
		image0 = cv::imread(imname, CV_LOAD_IMAGE_UNCHANGED);

		if (!image0.data) {
			cout << "\t cannot read file: " << imname << endl;
			continue;
		}

		float valu;
		image1 = cv::Mat::zeros(height_, width_, CV_8UC1);
		for (int i = 0; i < height_; i++) {
			for (int j = 0; j < width_; j++) {
				valu = 80 + ((image0.at<uint16_t>(i, j) - 15100) / 3);
				if (valu < 0) valu = 0;
				if (valu > 255) valu = 255;
				image1.at<uchar>(i, j) = (uchar)valu;
			}
		}
		cv::remap(image1, image, map1, map2, cv::INTER_CUBIC);
		cv::imwrite(save_folder + buffer + ".jpg", image);
	}
	
	
	//set distortion rectification parameters																					// focal length of IR camera (estimated)
	

	printf("finish!\n");

	return 1;
}

int corres2016pcd(int imgframe) {
	return (int)((float)imgframe * 0.4165 - 1967.3);
}

int
loadpcdfromfile(std::string filename, PCXYZRGBAPtr& cloud) {
	printf("Load point clouds from file: %s", filename.c_str());
	PCXYZRGBAPtr cld(new PCXYZRGBA);
	if (pcl::io::loadPCDFile(filename, *cld) == -1) {
		PCL_ERROR("\r error load pcd file!");
	}
	else {
		printf("\t load point clouds from : %s", filename.c_str());
	}
	cloud = cld;
	return 1;
}

void loadParameters(string filename, vector<string>& parameter) {
	cout << "\n read " << filename << endl;
	ifstream in_file(filename, ios::in);

	if (in_file.is_open()) {
		string line;
		parameter.clear();
		while (!in_file.eof()) {
			getline(in_file, line);
			parameter.emplace_back(line);
			//cout << temp << endl;
		}
		in_file.close();
		cout << "\t finish reading file..." << endl;
	}
	else {
		cout << "\t Error: cannot read file: " << filename << endl;
		system("pause"); exit(0);
	}

}

int
txt2pcd(string txtfile, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
	printf("\t load point cloud from txt...\n");


	ifstream file(txtfile);
	pcl::PointXYZRGB pt;
	if (!file.is_open()) {
		printf("\t Error: cannot open the file!\n");
		return -1;
	}
	else {
		double x, y, z;
		int r, g, b;
		int label;
		while (!file.eof()) {
			file >> x >> y >> z >> r >> g >> b >> label;
			pt.x = x; pt.y = y; pt.z = z;
			pt.r = uchar(r); pt.g = uchar(g); pt.b = uchar(b);
			//pt.label = label;
			cloud->points.push_back(pt);
		}file.close();
		cloud->width = cloud->points.size();
		cloud->height = 1;

	}


	printf("\t finish loading point cloud!\n");
	return 1;
}

int lasTxt2pcd(string inputfile, string outputfile) {
	//convert las txt 2 pcd
	printf("\t las txt 2 pcd\n");
	printf("\t read the file from %s\n", inputfile.c_str());

	pcl::PointCloud<PointXYZRGBL>::Ptr cloud(new pcl::PointCloud<PointXYZRGBL>);

	double x, y, z;
	int r, g, b, l;
	PointXYZRGBL pt;

	ifstream in_file(inputfile, ios::in);
	if (!in_file.is_open()) {
		cout << "\t Error: cannot read file: " << inputfile << endl;
		system("pause"); exit(0);
	} 
	else {
		while (!in_file.eof()) {
			//in_file>>pt.x >> pt.y >> pt.z >> pt.r >> pt.g >> pt.b >> pt.label;
			in_file>>x >> y >> z >> r >> g >> b >> l;
			pt.x = x; pt.y = y; pt.z = z;
			pt.r = r; pt.g = g; pt.b = b;
			pt.label = l;
			//pt.x=round(pt.x*1000) / 1000.0 - 691067.0; pt.y=round(pt.y*1000) / 1000.0 - 5336096.0; pt.z=round(pt.z*1000) / 1000.0 - 526.0;
			//cout << pt << endl;

			cloud->push_back(pt);
		}
		in_file.close();

		cout << "\t finish reading file..." << endl;
		cloud->width = 1; cloud->height = cloud->points.size();
		pcl::io::savePCDFileASCII(outputfile, *cloud);
		cloud->clear();
		printf("\t save the file to %s\n", outputfile.c_str());
	}

	printf("\tfinish!\n");
}

int loadpcdsequence(PCXYZRGBAPtr& output_cloud, string data_folder, int pcd_nr1, int pcd_nr2, int off=1) {
	///load the pcd sequences point clouds
	printf("Load point clouds from file: %s, from %d to %d.\n", data_folder.c_str(), pcd_nr1, pcd_nr2);
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);

	const string filelas1 = data_folder + "laserscanner1/";
	const string filelas2 = data_folder + "laserscanner2/";
	/*int pcd_nr1 = (int)(startIm * 0.4165 - 1967.3);
	int pcd_nr2 = (int)(endIm * 0.4165 - 1967.3);*/
	char buffer[100];
	int kk, anz_z, anz_p, i, j, count;
	string filelasf;
	count = 0;
	std::cout << "read point clouds...\n";
	for (int ie = pcd_nr1; ie <= pcd_nr2; ie=ie+off) {
		for (kk = 1; kk <= 2; kk++) {

			sprintf_s(buffer, "%05d", ie);
			if (kk == 1) filelasf = filelas1;
			else filelasf = filelas2;
			filelasf.append(buffer);
			filelasf.append(".pcd");
			if (pcl::io::loadPCDFile(filelasf, *cloud) == -1) {
				PCL_ERROR("cannot read pcdfile");
				std::cout << "\t filelas\n";
				continue;
			}

			anz_z = cloud->width;
			anz_p = cloud->height;

			for (i = 0; i < anz_z; i++) {
				for (j = 0; j < anz_p; j++) {

					if (((cloud->at(i, j).x) != (cloud->at(i, j).x)) || ((cloud->at(i, j).y) != (cloud->at(i, j).y)) || ((cloud->at(i, j).z) != (cloud->at(i, j).z))) {
						cloud->at(i, j).x = NAN;
						cloud->at(i, j).y = NAN;
						cloud->at(i, j).z = NAN;
						cloud->at(i, j).b = (uint8_t)(0);
						cloud->at(i, j).g = (uint8_t)(0);
						cloud->at(i, j).r = (uint8_t)(0);
					}
					else {
						output->push_back(cloud->at(i, j));
						count++;
					}
				}
			}
			//pcl::copyPointCloud(cloudl, *cloudm);
			cloud->clear();
		}
	}
	output->height = output->points.size(); output->width = 1;

	cout << "\t in total: " << output->points.size() << " points\n";
	std::cout << "\t finish reading...\n";
	output_cloud = output;
	return 1;
}

int loadpcd(PCXYZRGBAPtr& input_cloud, vector<string> parameters) {
	/// <summary>
	/// general function to load the point clouds
	/// </summary>
	/// <param name="intput_cloud"></param>
	/// <param name="parameters"></param>
	/// <returns></returns>

	int label_pcl = atoi(parameters[7].c_str());
	string dir_pcl = parameters[10].c_str();

	int label_img = atoi(parameters[21].c_str());
	string dir_img = parameters[24].c_str();
	string file_pos = parameters[31].c_str();
	int startIm, endIm, off;
	off=atoi(parameters[18].c_str());
	

	if (label_pcl == 0) {//read single point clouds
		loadpcdfromfile( parameters[13].c_str(), input_cloud);
	}
	else if (label_pcl == 1) {//read opint clouds from sequences direct number
		PCXYZRGBAPtr cloud;
		startIm = atoi(parameters[16].c_str());
		endIm = atoi(parameters[17].c_str());

		loadpcdsequence(input_cloud, dir_pcl, startIm, endIm,off);
	}
	else if (label_pcl == 2) {//read corresponding point clouds as image sequences
		PCXYZRGBAPtr cloud;
		startIm = atoi(parameters[27].c_str());
		endIm = atoi(parameters[28].c_str());
		startIm = corres2016pcd(startIm);
		endIm = corres2016pcd(endIm);

		loadpcdsequence(input_cloud, dir_pcl, startIm, endIm,off);
	}
	
	
	return 0;
}

int loadpcd(PCXYZRGBAPtr& input_cloud, vector<string> parameters, int startpcd, int endpcd) {
	/// <summary>
	/// general function to load the point clouds
	/// </summary>
	/// <param name="intput_cloud"></param>
	/// <param name="parameters"></param>
	/// <returns></returns>

	int label_pcl = atoi(parameters[7].c_str());
	string dir_pcl = parameters[10].c_str();

	int label_img = atoi(parameters[21].c_str());
	string dir_img = parameters[24].c_str();
	string file_pos = parameters[31].c_str();
	int startIm, endIm, off;
	off = atoi(parameters[18].c_str());


	PCXYZRGBAPtr cloud;

	loadpcdsequence(input_cloud, dir_pcl, startpcd, endpcd, off);



	return 0;
}

void
WashingPC(PCXYZRGBAPtr cld_, PCXYZRGBAPtr& output_cloud, float resolution_, float threshold) {//delete duplicate points

	printf("Wash point cloud and filter redundent points.\n");
	int change = cld_->points.size();
	int ori_ = change;

	PCXYZRGBAPtr new_cloud;// (new PCXYZRGBA);
	PointXYZRGBA pt;

	while (change) {
		new_cloud.reset(new PCXYZRGBA);
		octree::OctreePointCloudSearch<PointXYZRGBA> octree(resolution_);
		octree.setInputCloud(cld_);
		octree.addPointsFromInputCloud();
		std::vector<int> pointIdxNKNSearch;
		std::vector<float> pointNKNSquaredDistance;
		std::vector<int> idx;
		int total = cld_->points.size();
		//int * p = new int[total] {-1};
		std::vector<int> p(total, -1);

		for (int i = 0; i < total; i++) {
			pt = cld_->points.at(i);
			if (p[i] == -1)p[i] = 1;
			else if (p[i] == 0)continue;

			if (octree.nearestKSearch(pt, 2, pointIdxNKNSearch, pointNKNSquaredDistance)&& p[i]==-1) {
				if (pointNKNSquaredDistance[0] < threshold && p[pointIdxNKNSearch[0]] == -1) {
					p[pointIdxNKNSearch[0]] = 0;
				}
				if (pointNKNSquaredDistance[1] < threshold && p[pointIdxNKNSearch[1]] == -1) {
					p[pointIdxNKNSearch[1]] = 0;
				}
			}
		}

		int count = 0;
		for (int i = 0; i < total; i++) {
			if (p[i] == 1) { new_cloud->points.push_back(cld_->points.at(i)); count++; }
		}
		new_cloud->height = count; new_cloud->width = 1;
		
		change = total - new_cloud->points.size();
		printf("\t chang points: %d (%d -> %d)\t", change, total, (int)new_cloud->points.size());
		//cout << "Washing data: change " << change << endl;
		/*cld_.reset();
		cld_ = new_cloud;*/
		cld_.swap(new_cloud);
	}
	output_cloud = new_cloud;

	//pcl::copyPointCloud(*new_cloud, *this->points_cloud_);

	pcl::console::print_info("Wash point cloud: Point reduction from %i to %i\n", ori_, cld_->points.size());
}

void FacadeValidate(vector<vector<cv::Point>>& pointSet, cv::Mat img, int thread) {
	//validate the points by size
	vector<cv::Point> pts;
	vector<int> a(pointSet.size(), 0);
	vector<vector<cv::Point>> pointTemp; pointTemp = pointSet;

	cv::Mat im;
	int max_x, max_y, min_x, min_y;
	for (int i = 0; i < pointSet.size(); i++) {

		im = cv::Mat::zeros(img.size(), CV_8UC1);
		pts = pointSet[i];
		max_x = pts[0].x; max_y = pts[0].y;
		min_x = max_x; min_y = max_y;
		for (int n = 0; n < pts.size(); n++) {
			im.at<uchar>(pts[n]) = 255;
			max_x = max(max_x, pts[n].x); max_x = min(min_x, pts[n].x);
			max_y = max(max_y, pts[n].y); max_y = min(min_y, pts[n].y);
		}

		if (max_x - min_x > thread || max_y - min_y > thread) { a[i] = 1; }
	}

	pointSet.clear();
	for (int i = 0; i < a.size(); i++) {
		if (a[i] > 0) pointSet.emplace_back(pointTemp[i]);
	}
}

void
LoadPointclouds(string fileroot, int start_frame, int end_frame, pcl::PointCloud<PointT>::Ptr outputcloud) {
	//merge the point clouds by 
	//fileroot
	//int start
	//int end
	printf("load point clouds and merge from frame %i to %i\t",  start_frame, end_frame);
	int ie,kk, anz_z, anz_p,i,j;
	char buffer[100];
	string filelas1 = fileroot; filelas1.append("laserscanner1/");
	string filelas2 = fileroot; filelas2.append( "laserscanner2/");
	string filelasf;
	double pcd_nrf;

	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr texcloud(new pcl::PointCloud<PointT>);
	PointT texpoint;
	int count = 0;

	for (ie = start_frame; ie < end_frame; ie++) {

		for (kk = 1; kk <= 2; kk++) {

			sprintf_s(buffer, "%05d", ie);
			if (kk == 1) filelasf = filelas1;
			else filelasf = filelas2;
			filelasf.append(buffer);
			filelasf.append(".pcd");
			pcl::io::loadPCDFile(filelasf, *cloud);
			anz_z = cloud->width;
			anz_p = cloud->height;

			for (i = 0; i < anz_z; i++) {
				for (j = 0; j < anz_p; j++) {

					if (((cloud->at(i, j).x) != (cloud->at(i, j).x)) || ((cloud->at(i, j).y) != (cloud->at(i, j).y)) || ((cloud->at(i, j).z) != (cloud->at(i, j).z))) {
						cloud->at(i, j).x = NAN;
						cloud->at(i, j).y = NAN;
						cloud->at(i, j).z = NAN;
						cloud->at(i, j).b = (uint8_t)(0);
						cloud->at(i, j).g = (uint8_t)(0);
						cloud->at(i, j).r = (uint8_t)(0);
					}
					else {
						texpoint = cloud->at(i, j);
						texpoint.r =  cloud->at(i, j).r;
						texpoint.g = cloud->at(i, j).g;
						texpoint.b = cloud->at(i, j).b;
						//texpoint.label = 0;
						texcloud->push_back(texpoint);
						count++;
					}
				}
			}
		}
	}
	std::cout << count << " points" << endl;
	texcloud->width = 1; texcloud->height = count;

	/*outpcdname = fold_out;
	outpcdname.append("merged");
	outpcdname.append(".pcd");
	pcl::io::savePCDFileBinary(outpcdname, *texcloud);*/
	pcl::copyPointCloud(*texcloud, *outputcloud);
}

void
renderIntensity(pcl::PointCloud<PointT>::Ptr inputcloud, pcl::PointCloud<PointT>::Ptr outputcloud) {
	//render the point clouds
	printf("render the point clouds with intensity\n");
	pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);

	PointT pt;
	octree::OctreePointCloudSearch<PointXYZRGBA> octree(0.05);
	octree.setInputCloud(inputcloud);
	octree.addPointsFromInputCloud();
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	std::vector<int> idx;
	int total = outputcloud->points.size();
	//int * p = new int[total] {-1};
	int count=0;
	for (int i = 0; i < total; i++) {
		pt = outputcloud->points.at(i);


		if (octree.nearestKSearch(pt, 1, pointIdxNKNSearch, pointNKNSquaredDistance)) {
			output->points.push_back(inputcloud->points.at(pointIdxNKNSearch[0]));
			count++;
		}
	}

	output->height = count; output->width = 1;

	pcl::copyPointCloud(*output, *outputcloud);
}

cv::Mat clustering(cv::Mat img_ori, std::vector<std::vector<cv::Point>>& pointSet, int num) {
	cv::Mat img;
	//cv::cvtColor(img_ori, img, cv::COLOR_RGB2GRAY);
	img_ori.convertTo(img, CV_8UC1);
	std::vector<cv::Point> output_points;
	pointSet.clear();
	for (int j = 0; j < img.rows; j++) {
		for (int i = 0; i < img.cols; i++)
		{
			if ((int)img.at<uchar>(j, i) > 0)
			{
				//cv::circle(cv::Mat(input_img), cv::Point(i, j), 1, cv::Scalar(255,0,0), 1);
				//drawCross(vis, cv::Point(i, j), cv::Scalar(0, 0, 255), 6, 1);
				output_points.push_back(cv::Point(i, j));
			}
		}
	}

	//cv::imshow("Labels", img); cv::waitKey(0);
	std::vector<int> label;
	int th_distance = 2;
	int th2 = th_distance * th_distance;
	int n_labels = cv::partition(output_points, label, [th2](const cv::Point& lhs, const cv::Point& rhs) {
		return ((lhs.x - rhs.x) * (lhs.x - rhs.x) + (lhs.y - rhs.y) * (lhs.y - rhs.y)) < th2;
		});
	std::vector<std::vector<cv::Point>> contours(n_labels);//contours saves the label results

	//find the local maximum
	for (int i = 0; i < output_points.size(); ++i)
	{
		contours[label[i]].emplace_back(output_points[i]);
	}

	// Build a vector of random color, one for each class (label)
	std::vector<cv::Vec3b> colors;
	for (int i = 0; i < n_labels; ++i)
	{
		colors.push_back(cv::Vec3b(rand() & 255, rand() & 255, rand() & 255));
	}

	cv::Mat vis;
	vis = img_ori.clone();
	cv::cvtColor(vis, vis, cv::COLOR_GRAY2RGB);
	cv::Mat3b lbl(img.rows, img.cols, cv::Vec3b(0, 0, 0));
	for (int i = 0; i < output_points.size(); ++i)
	{
		lbl(output_points[i]) = colors[label[i]];
		//drawCross(lbl, output_points[i], colors[label[i]], 3, 1);
	}

	imshow("Labels", lbl);cv::waitKey(0); cv::destroyAllWindows();
	//cv::imwrite("../partition.png", lbl);
	std::cout << "\t clustering; the minimum cluser has " << num << " points,\t";

	std::vector<cv::Point> cluster; output_points.clear();
	std::cout << "in total: " << n_labels << "clusters.\n";
	int max_x, max_y, min_x, min_y;
	cv::Mat fac = cv::Mat::zeros(img.size(), CV_8UC1);

	FacadeValidate(pointSet, img, num);

	for (int i = 0; i < n_labels; i++) {
		cluster.clear();
		copy(contours[i].begin(), contours[i].end(), back_inserter(cluster));
		
		float max = 0;
		cv::Point pt;

		try {
			if (cluster.size() >= num) {// large cluster for keypoints detection
				pointSet.emplace_back(cluster);
				std::vector<cv::Point>::iterator it = cluster.begin();
				pt.x = (*it).x; pt.y = (*it).y; max = img.at<float>((*it).y, (*it).x);
				while (it != cluster.end()) {
					fac.at<uchar>(*it) = 255;
					it++;
				}
				output_points.emplace_back(pt);
				max = 0;
			}
		}
		catch (std::exception& e) {
			std::cout << e.what() << std::endl;
			std::system("pause");
		}
	}
	return fac;
}

float lineDist(cv::Vec4f l){
	float dx, dy;
	dx = l[0] - l[2]; dy = l[1] - l[3];
	return sqrt(dx * dx + dy * dy);
}

cv::Mat FacadeValidateByLine(vector<vector<cv::Point>>& pointSet, 
	cv::Mat img, int thread) {
	//validate the points by size
	vector<cv::Point> pts;
	vector<int> a(pointSet.size(), 0);
	vector<vector<cv::Point>> pointTemp; pointTemp = pointSet;

	//line detection
	cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector();
	vector<cv::Vec4f> lines_std;
	
	cv::Mat im; 
	int max_x, max_y, min_x, min_y;
	for (int i = 0; i < pointSet.size(); i++) {

		im = cv::Mat::zeros(img.size(), CV_8UC1);
		pts = pointSet[i];
		max_x = pts[0].x; max_y = pts[0].y;
		min_x = max_x; min_y = max_y;
		for (int n = 0; n < pts.size(); n++) {
			im.at<uchar>(pts[n]) = 255;
		}
		lsd->detect(im, lines_std);

		for (int n = 0; n < lines_std.size(); n++) {
			if (lineDist(lines_std[n]) >= thread) { a[i] = 1; continue; }
		}
	}

	pointSet.clear(); im = cv::Mat::zeros(img.size(), CV_8UC1);
	for (int i = 0; i < a.size(); i++) {
		if (a[i] > 0) {
			pointSet.emplace_back(pointTemp[i]);
			for (int m = 0; m < pointTemp[i].size(); m++) {
				im.at<uchar>(pointTemp[i][m]) = 255;
			}
		}
	}

	
	return im;
}

bool BetweenPoints(cv::Point P, cv::Point A, cv::Point B) {
	//wehter point P is within pointA and pointB?
	//A1*B2=A2*B1
	if (fabs((P.x - A.x) * (B.y - A.y) - (P.y - A.y) * (B.x - A.x)) <= 1e-2)return TRUE;
	else return FALSE;
}

float getDist_P2L(cv::Point pointP, cv::Vec4f line)
{
	//求直线方程
	cv::Point pointA, pointB, crossPt;
	pointA = cv::Point((int)line[0], (int)line[1]); pointB = cv::Point((int)line[2], (int)line[3]);
	if (pointA.x == pointB.x) { crossPt.x = pointA.x; crossPt.y = pointP.y; 
		if (BetweenPoints(crossPt, pointA, pointB)) {return fabs((float)(crossPt.x-pointP.x));}
		else {
			return min(sqrt(pow(pointA.x - pointP.x, 2) + pow(pointA.y - pointP.y, 2)),
				sqrt(pow(pointB.x - pointP.x, 2) + pow(pointB.y - pointP.y, 2)));
		}
	}//horizontal line
	else if(pointA.y==pointB.y){ crossPt.y = pointA.y; crossPt.x = pointP.x; 
		if (BetweenPoints(crossPt, pointA, pointB)) { return fabs((float)(crossPt.y - pointP.y)); }
		else {
			return min(sqrt(pow(pointA.x - pointP.x, 2) + pow(pointA.y - pointP.y, 2)),
				sqrt(pow(pointB.x - pointP.x, 2) + pow(pointB.y - pointP.y, 2)));
		}
	}//vertical line
	else{
		int A = 0, B = 0, C = 0,k;
		B = pointA.y - pointB.y;
		A = pointA.x - pointB.x;
		k = B / A;
		C = pointA.y - k * pointA.x;
		
		crossPt.x = (A * pointA.x + B * pointA.y - B * C) / (A + k * B);
		crossPt.y = (k * crossPt.x + C);
		//代入点到直线距离公式
		if (BetweenPoints(crossPt, pointA, pointB)) { return sqrt(pow(crossPt.x - pointP.x, 2) + pow(crossPt.y - pointP.y, 2)); }
		else {
			return min(sqrt(pow(pointA.x - pointP.x, 2) + pow(pointA.y - pointP.y, 2)),
				sqrt(pow(pointB.x - pointP.x, 2) + pow(pointB.y - pointP.y, 2)));
		}

	}
	//int A = 0, B = 0, C = 0;
	//A = pointA.y - pointB.y;
	//B = pointB.x - pointA.x;
	//C = pointA.x * pointB.y - pointA.y * pointB.x;
	////代入点到直线距离公式
	//float distance = 0;
	//distance = ((float)abs(A * pointP.x + B * pointP.y + C)) / ((float)sqrtf(A * A + B * B));
	//return distance;
}

float getDist_L2L(cv::Vec4f l1, cv::Vec4f l2) {
	cv::Point pointA, pointB, pointC, pointD;
	pointA = cv::Point((int)l1[0], (int)l1[1]); pointB = cv::Point((int)l1[2], (int)l1[3]);
	pointC = cv::Point((int)l2[0], (int)l2[1]); pointD = cv::Point((int)l2[2], (int)l2[3]);
	float dist;
	dist = min(getDist_P2L(pointA, l2), getDist_P2L(pointB, l2));
	dist = min(dist, getDist_P2L(pointC, l1)); dist = min(dist, getDist_P2L(pointD, l1));

	return dist;
}

vector<int> mergeCloseLines(vector<cv::Vec4f>& inputlines, vector<cv::Vec4f>& outputlines, cv::Mat img, float distT, float angT) {
	int ss = inputlines.size();
	vector<int> label(ss, 0); //cout << ss << " lines" << endl;
	int cluster = 0; double ang1, ang2;
	float dist;

	for (int i = 0; i < ss; i++) {
		ang1 = atan2(inputlines[i][3] - inputlines[i][1], inputlines[i][2] - inputlines[i][0]);
		if (label[i] == 0) { cluster++; label[i] = cluster; }
		for (int j = i+1; j < ss; j++) {
			ang2 = atan2(inputlines[j][3] - inputlines[j][1], inputlines[j][2] - inputlines[j][0]);
			dist = getDist_L2L(inputlines[i], inputlines[j]);

			if (fabs(ang1 - ang2) <= angT || fabs(fabs(ang1 - ang2) - M_PI) <= angT) {
				if (dist <= distT) {
					if (label[j] == 0) { label[j] = label[i]; }
					else {
						int l = label[j]; label[j] = label[i];
						for (int m = 0; m < ss; m++) { if (label[m] == l)label[m] = label[i]; }//change all the neighbor to label[i]
					}
				}
			}
		}
	}
	set<int> idx(label.begin(), label.end());
	cv::Mat p=cv::Mat::zeros(img.size(),CV_8UC1);
	set<int>::iterator it = idx.begin();
	vector<cv::Point> pts; cv::Vec4f line; outputlines.clear();
	cv::Point2f minPt, maxPt;
	for (it; it!=idx.end(); it++) {
		p = p * 0;
		minPt.x = inputlines[0][0]; minPt.y = inputlines[0][1];
		maxPt.x = inputlines[0][0]; maxPt.y = inputlines[0][1];
		for (int j = 0; j < label.size(); j++) {
			if (label[j] == *it) { cv::line(p, cv::Point(inputlines[j][0], inputlines[j][1]), 
				cv::Point(inputlines[j][2], inputlines[j][3]), 255, 1); 
			minPt.x = min(minPt.x, inputlines[j][0]); minPt.y = min(minPt.y, inputlines[j][1]);
			maxPt.x = max(maxPt.x, inputlines[j][2]); maxPt.y = max(maxPt.y, inputlines[j][3]);
			}
		}
		pts.clear();
		for (int j = 0; j < p.rows; j++) {for (int i = 0; i < p.cols; i++){
				if (p.at<uchar>(j, i) > 0){
					pts.emplace_back(cv::Point(i, j));}
			}
		}

		cv::fitLine(pts, line, cv::DIST_L2,0,1e-2,1e-2);//result for fit line is different
		cv::Point point0;
		point0.x = line[2];
		point0.y = line[3];

		if(line[0]==0){//vertical lines
			outputlines.emplace_back(cv::Vec4f(minPt.x, minPt.y, minPt.x, maxPt.y));
		}
		else {
			double k = line[1] / line[0];

			//计算直线的端点(y = k(x - x0) + y0)
			cv::Point point1, point2;
			point1.x = minPt.x;
			point1.y = k * (point1.x - point0.x) + point0.y;
			point2.x = maxPt.x;
			point2.y = k * (point2.x - point0.x) + point0.y;

			outputlines.emplace_back(cv::Vec4f((float)point1.x, (float)point1.y, (float)point2.x, (float)point2.y));
		}
		
	}

	return label;
}

cv::Point2f getCrossPoint(cv::Vec4f LineA, cv::Vec4f LineB)
{
	cv::Point2f crossPoint;

	double ka, kb;
	ka = atan2(double(LineA[3] - LineA[1]), double(LineA[2] - LineA[0]));
	kb = atan2(double(LineB[3] - LineB[1]), double(LineB[2] - LineB[0]));
	if (fabs(ka - kb) <= 2.0 * M_PI / 180.0 ||fabs( fabs(ka - kb)-M_PI) <= 2.0 * M_PI / 180.0) crossPoint= cv::Point2f(-1, -1);
	else if (LineA[0] == LineA[2] && LineB[0] != LineB[2]) {
		kb = tan(kb);
		crossPoint.x = LineA[0];
		crossPoint.y=kb*crossPoint.x+LineB[3]-kb*LineB[2];
	}
	else if (LineB[0] == LineB[2] && LineA[0] != LineA[2]) {
		ka = tan(ka);
		crossPoint.x = LineB[0];
		crossPoint.y = ka * crossPoint.x + LineA[3] - ka * LineA[2];
	}
	else {
		ka = tan(ka); kb = tan(kb);
		int B1 = LineA[1] - ka * LineA[0]; int B2 = LineB[1] - kb * LineB[0];
		crossPoint.x = (B2 - B1) / (ka - kb);
		crossPoint.y = B1 - ka * crossPoint.x;
	}
	//ka = (double)(LineA[3] - LineA[1]) / (double)(LineA[2] - LineA[0]); //slope of line A
	//kb = (double)(LineB[3] - LineB[1]) / (double)(LineB[2] - LineB[0]); //slope of line B

	/*
	crossPoint.x = (ka * LineA[0] - LineA[1] - kb * LineB[0] + LineB[1]) / (ka - kbfp);
	crossPoint.y = (ka * kb * (LineA[0] - LineB[0]) + ka * LineB[1] - kb * LineA[1]) / (ka - kb);*/
	return crossPoint;
}

cv::Mat
FacadeCompliment(vector<cv::Vec4f> &inputlines, vector<cv::Vec4f> &outputlines,cv::Mat img){
	//impliment the lines to the only whole segments
	//1. merge close lines
	cv::Vec4f l;
	for (int i = 0; i < inputlines.size(); i++)
	{
		l = inputlines[i][0] < inputlines[i][2] ? inputlines[i] : cv::Vec4f(inputlines[i][2], inputlines[i][3], inputlines[i][0], inputlines[i][1]);
	}
	vector<int> labels; vector<cv::Vec4f> reduced_lines;
	labels = mergeCloseLines(inputlines, reduced_lines, img, 6, 4.0/180.0*M_PI);
	printf("\t Reduce lines from %i to %i.", inputlines.size(), reduced_lines.size());

	cv::Mat im = img.clone()*0;
	cv::cvtColor(im, im, cv::COLOR_GRAY2RGB);
	for (int i = 0; i < reduced_lines.size(); i++) {
		cv::line(im, cv::Point(reduced_lines[i][0], reduced_lines[i][1]), cv::Point(reduced_lines[i][2], reduced_lines[i][3]),
			cv::Scalar(255, 0, 0), 2);
	}imshow("reduced lines", im); cv::waitKey(0); cv::destroyAllWindows();

	//2.find closest lines to each other
	int ss = reduced_lines.size();
	vector<int> pair_pt_f(reduced_lines.size(),0); vector<int> pair_pt_e(reduced_lines.size(), 0);
	vector<pair<float, float>>line_dist(reduced_lines.size());
	int idxf, idxe; float disf, dise;
	cv::Point pt1, pt2;cv::Point2f ptF, ptE;
	float d1, d2;
	for (int i = 0; i < reduced_lines.size(); i++) {
		pt1 = cv::Point((int)reduced_lines[i][0], (int)reduced_lines[i][1]); pt2 = cv::Point((int)reduced_lines[i][2], (int)reduced_lines[i][3]);
		disf = img.cols; dise = img.cols; idxf = 0; idxe = 0;
		for (int j = 0; j < reduced_lines.size(); j++) {
			if (i != j) {
				d1 = getDist_P2L(pt1, reduced_lines[j]); d2 = getDist_P2L(pt2, reduced_lines[j]);
				if (disf > d1) { disf = d1; idxf = j; }
				if (dise > d1) { dise = d1; idxe = j; }
			}
		}
		pair_pt_f.emplace_back(idxf); pair_pt_e.emplace_back(idxe); line_dist.emplace_back(make_pair(disf, dise));
		//3. calculate the cross points and connect
		ptF = getCrossPoint(reduced_lines[i], reduced_lines[idxf]); ptE = getCrossPoint(reduced_lines[i], reduced_lines[idxe]);
		reduced_lines[i] = cv::Vec4f(ptF.x, ptF.y, ptE.x, ptE.y);
	}
	
	vector<int> endpt; 
	for (int i = 0; i < pair_pt_f.size(); i++) {//find the star and end lines which has identical start and end
		if (pair_pt_f[i] == pair_pt_e[i]) { endpt.emplace_back(i); }
	}
	int label,count,shift; vector<int> sequence; vector<int> occupy(reduced_lines.size(), 1);
	count = 0; shift = 0;
	if (endpt.size() <= 2) {//only two endpoints, resonable
		label = endpt[shift]; occupy[label] = 0; sequence.emplace_back(label); shift++;
		//vector<int>::iterator itf = find(pair_pt_f.begin(), pair_pt_e.end(), label);
		vector<int>::iterator ite = find(pair_pt_e.begin(), pair_pt_e.end(), label);
		while (ite != pair_pt_e.end() && count<reduced_lines.size()) {
			label = *ite; sequence.emplace_back(label);
			if (occupy[label] > 0) {
				occupy[label] = 0;
				vector<int>::iterator ite = find(pair_pt_e.begin(), pair_pt_e.end(), label);
			}//can be sequence
			else { label = endpt[shift++]; occupy[label] = 0; sequence.emplace_back(label);
			vector<int>::iterator ite = find(pair_pt_e.begin(), pair_pt_e.end(), label);}
			count++;
		}
	}

	//4. find the endpoints of the segments and extend to the boundary
	ptF = cv::Point2f(reduced_lines[sequence[0]][0], reduced_lines[sequence[0]][1]);
	ptE = cv::Point2f(reduced_lines[sequence[reduced_lines.size()]][2], reduced_lines[sequence[reduced_lines.size()]][3]);

	if (ptF.x > 0 && ptF.x < img.cols && ptF.y>0 && ptF.y < img.rows) {//not connect the boundary, need to extend
		ptF = getCrossPoint(reduced_lines[sequence[0]], cv::Vec4f(0, 1, 0, 2));
		if(ptF.y>img.rows){ ptF = getCrossPoint(reduced_lines[sequence[0]], cv::Vec4f(1, img.rows, 1, img.rows)); }
		reduced_lines[sequence[0]] = cv::Vec4f(ptF.x, ptF.y, reduced_lines[sequence[0]][2], reduced_lines[sequence[0]][3]);
	}
	if (ptE.x > 0 && ptE.x < img.cols && ptE.y>0 && ptE.y < img.rows) {//not connect the boundary, need to extend
		ptE = getCrossPoint(reduced_lines[sequence[ss]], cv::Vec4f(img.cols, 1, img.cols, 2));
		if (ptE.y <0) { ptE = getCrossPoint(reduced_lines[sequence[ss]], cv::Vec4f(img.cols, 0, img.cols, 0)); }
		reduced_lines[sequence[ss]] = cv::Vec4f(ptE.x, ptE.y, reduced_lines[sequence[ss]][2], reduced_lines[sequence[ss]][3]);
	}

	outputlines.clear();
	//5. finally get the whole lines and points
	im = cv::Mat::zeros(img.size(), CV_8UC1);
	for (int i = 0; i < ss; i++) {
		cv::line(im, cv::Point((int)reduced_lines[i][0], (int)reduced_lines[i][1]),
			cv::Point((int)reduced_lines[i][2], (int)reduced_lines[i][3]), 255, 1);
		outputlines.emplace_back(reduced_lines[sequence[i]]);
	}

	return im;
}

void FacadeSeperation(cv::Mat img, pcl::PointCloud<PointT>::Ptr pointclouds, pcl::PointCloud<PointT>::Ptr outputclouds,
	float pix = 0.2f) {
	//seperate the inner and outer point sets
	pcl::PointCloud<PointT>::Ptr cloudIn(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloudOut(new pcl::PointCloud<PointT>);

	//float pix=0.2;
	PointT min_pt, max_pt;
	pcl::getMinMax3D(*pointclouds, min_pt, max_pt);
	float offx, offy, offz;
	offx = (float)min_pt.x;
	offy = (float)min_pt.y;
	offz = (float)min_pt.z;

	//cv::threshold(img, img, 1, 255,cv::THRESH_BINARY);
	int countIn, countOut;
	int x, y;
	countIn = 0; countOut = 0;
	for (int i = 0; i < pointclouds->points.size(); i++) {
		x = (int)((float)(pointclouds->points.at(i).x - offx) / pix);
		y = (int)((float)(pointclouds->points.at(i).y - offy) / pix);
		if (img.at<uchar>(x, y) == 1) { countIn++; cloudIn->points.emplace_back(pointclouds->points.at(i)); }
		else if (img.at<uchar>(x, y) == 2) { countOut++; cloudOut->points.emplace_back(pointclouds->points.at(i)); }
	}
	if (countIn > countOut) *outputclouds = *cloudIn;
	else { *outputclouds = *cloudOut; }
}

void GroundRemovelByHeight(pcl::PointCloud<PointT>::Ptr p_cloud, pcl::PointCloud<PointT>::Ptr output_cloud, float h) {
	//remove ground by height
	printf("remove the ground by height %f\n", h);
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());

	PointT min_p, max_p;
	pcl::getMinMax3D(*p_cloud, min_p, max_p);
	float base = (float)min_p.z;
	int s = p_cloud->points.size();
	for (int i = 0; i < s; i++) {
		if (fabs((float)p_cloud->points.at(i).z - base) >= h) {
			cloud->points.push_back(p_cloud->points.at(i));
		}
	}
	cloud->width = cloud->points.size();
	cloud->height = 1;
	cloud->is_dense = true;
	*output_cloud = *cloud;
}

void
FacadesExtractionByRoute(pcl::PointCloud<PointT>::Ptr inputcloud, pcl::PointCloud<PointT>::Ptr inputRoute,
	pcl::PointCloud<PointT>::Ptr outputcloud, pcl::PointCloud<PointT>::Ptr outputfacades, float pix = 0.2f) {
	//extract the point clouds with given route information
	//*************1. project the orginal point clouds and ge the density map
	printf("Outdoor scene extraction...\n");
	PointT min_pt, max_pt; int i, j;
	pcl::getMinMax3D(*inputcloud, min_pt, max_pt);
	PointT min_ptr, max_ptr;
	pcl::getMinMax3D(*inputRoute, min_ptr, max_ptr);
	min_pt.x = min(min_pt.x, min_ptr.x); min_pt.y = min(min_pt.y, min_ptr.y); min_pt.z = min(min_pt.z, min_ptr.z);
	max_pt.x = max(max_pt.x, max_ptr.x); max_pt.y = max(max_pt.y, max_ptr.y); max_pt.z = max(max_pt.z, max_ptr.z);
	pcl::PointCloud<PointT>::Ptr ori_cloud(new pcl::PointCloud<PointT>());
	pcl::copyPointCloud(*inputcloud, *ori_cloud);
	GroundRemovelByHeight(inputcloud, inputcloud, 5);

	//cout << min_pt << max_pt << endl;
	float off_x, off_y, off_z;
	int width, height;
	off_x = min_pt.x; off_y = min_pt.y; off_z = min_pt.z;
	width = (max_pt.x - off_x) / pix + 1;
	height = (max_pt.y - off_y) / pix + 1;
	cv::Mat proj = cv::Mat::zeros(cv::Size(height, width), CV_64FC1);
	int s = inputcloud->points.size();
	PointT pt;
	double top = 0, down;
	cv::Mat heightMap = cv::Mat::zeros(proj.size(), CV_64FC1);
	for (i = 0; i < s; i++) {
		pt = inputcloud->points.at(i);
		proj.at<double>((int)((pt.x - off_x) / pix), (int)((pt.y - off_y) / pix)) += 1;
		heightMap.at<double>((int)((pt.x - off_x) / pix), (int)((pt.y - off_y) / pix)) =
			max(heightMap.at<double>((int)((pt.x - off_x) / pix), (int)((pt.y - off_y) / pix)), (((double)pt.z - (double)off_z) / (double)pix));
		top = max(top, (((double)pt.z - (double)off_z) / (double)pix));
	}
	proj.convertTo(proj, CV_8UC1);
	cv::imshow("proj", proj); //cv::waitKey();
	//Process point clouds height map
	cv::Mat heighThread;
	cv::Point maxLoc, minLoc;

	float thread = 12.0 / pix;
	cv::threshold(proj, heighThread, thread, 255, cv::THRESH_BINARY);//binary image with enough height
	cv::imshow("Nadirview", 255-heighThread); //cv::waitKey();

	vector<vector<cv::Point>> points; cv::Mat facade;
	facade = clustering(heighThread, points, 200);
	facade = FacadeValidateByLine(points, facade, 10.0/pix);
	cv::threshold(facade, facade, 80, 255, cv::THRESH_BINARY);

	//facades extraction
	pcl::PointCloud<PointT>::Ptr Facades(new pcl::PointCloud<PointT>());

	//cv::threshold(img, img, 1, 255,cv::THRESH_BINARY);
	int fad;
	int x, y;
	fad = 0;
	for (i = 0; i < ori_cloud->points.size(); i++) {
		x = (int)((float)(ori_cloud->points.at(i).x - off_x) / pix);
		y = (int)((float)(ori_cloud->points.at(i).y - off_y) / pix);
		if (facade.at<uchar>(x, y) > 0) { fad++; Facades->points.emplace_back(ori_cloud->points.at(i)); }
	}
	Facades->height = fad; Facades->width = 1;
	*outputfacades = *Facades;

	cv::Mat dst = 255 - facade;
	cv::imshow("Facade", dst); //cv::waitKey();

	//line detection
	//cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector();
	vector<cv::Vec4f> lines_std;
	//lsd->detect(dst, lines_std);lsd->drawSegments(image, lines);

	// Show found lines
	cv::Mat image = facade.clone() * 0;

	double rho = 1.0;
	double theta = 1.0 / 180.0 * CV_PI;
	double threshold = 13.0;
	double minLineLength = 12.0;
	double maxLineGap = 7.0;

	//Hough line transformation
	cv::HoughLinesP(facade, lines_std, rho, theta, threshold, minLineLength, maxLineGap);

	//draw the lines
	cv::Mat LineImg;
	LineImg = facade.clone();
	cv::Vec4i LineStand;
	cv::cvtColor(LineImg, LineImg, cv::COLOR_GRAY2RGB);
	vector<cv::Vec4f> lines;
	for (i = 0; i < lines_std.size(); i++) {
		if (lineDist(lines_std[i]) > 20.0 / (double)pix)lines.emplace_back(lines_std[i]);
	}

	LineImg = LineImg * 0;
	for (i = 0; i < lines.size(); i++) {
		LineStand = lines_std[i];
		cv::line(LineImg, cv::Point(LineStand[0], LineStand[1]), cv::Point(LineStand[2], LineStand[3]), cv::Scalar(0, 0, 255), 2);

	}
	cv::imshow("Hough lines", LineImg); cv::waitKey(0); cv::destroyAllWindows();
	//cv::imwrite("../lineinter.png", LineImg);

	cv::Mat posIm;
	cv::cvtColor(LineImg, posIm, cv::COLOR_RGB2GRAY);
	vector<cv::Point> pos;
	for (i = 0; i < inputRoute->points.size(); i++) {
		pos.emplace_back(cv::Point((int)((inputRoute->points.at(i).y - off_y) / pix),(int)((inputRoute->points.at(i).x - off_x) / pix)));
	}
	cv::threshold(posIm, posIm, 1, 255, cv::THRESH_BINARY);
	dst = posIm.clone()*0;
	for (i = 0; i < pos.size() - 1; i++) {
		cv::line(dst, pos[i], pos[i + 1], 255, 1);
	}pos.clear(); cv::Vec4f line; //imshow("pos", dst); cv::waitKey();
	for (i = 0; i < dst.rows; i++) {
		for (j = 0; j < dst.cols; j++){
			if ((int)dst.at<uchar>(i, j) > 0){pos.push_back(cv::Point(i, j));}
		}
	}cv::fitLine(pos, line, cv::DIST_L2, 0, 1e-2, 1e-2);
	float dx, dy ; dx = -line[1] / sqrt(pow(line[0], 2) + pow(line[1], 2)); dy = line[0] / sqrt(pow(line[0], 2) + pow(line[1], 2));

	cout << "\t image size:" << posIm.rows << " , " << posIm.cols << "\n";
	cv::threshold(dst, dst, 1, 255, cv::THRESH_BINARY); //imshow("posIm", posIm); cv::waitKey();
	cv::Point2f p; cv::Point2f pk;
	for (i = 0; i < pos.size(); i++) {//from pos location to the boudary and find the closest facade as interested areas
		p.x = (float)pos[i].x; p.y = (float)pos[i].y;
		p.x = (p.x + dx); p.y = (p.y + dy); 
		while (posIm.at<uchar>((int)p.x, (int)p.y) <= 200 && (int)p.x<posIm.rows && p.y<posIm.cols && p.x* p.y>=0) {
			posIm.at<uchar>((int)p.x, (int)p.y) = 155;
			p.x = (p.x + dx); p.y = (p.y + dy);
		}
		p.x = pos[i].x; p.y = pos[i].y;
		p.x = (p.x - dx); p.y = (p.y - dy);
		while ((int)p.x < posIm.rows && p.y < posIm.cols && p.x * p.y >= 0 && posIm.at<uchar>((int)p.x, (int)p.y) <= 200 ) {
			//cout << "\t(" << p.x << "," << p.y << ")\t";
			posIm.at<uchar>((int)p.x, (int)p.y) = 155;
			p.x = (p.x - dx); p.y = (p.y - dy);
		}
	}
	posIm = posIm + dst; 

	cv::medianBlur(posIm, posIm, 3); imshow("marker", posIm); cv::waitKey(); cv::destroyAllWindows();

	pcl::PointCloud<PointT>::Ptr cloudOut(new pcl::PointCloud<PointT>());

	//cv::threshold(img, img, 1, 255,cv::THRESH_BINARY);
	int countIn, countOut;
	//int x, y;
	countIn = 0; countOut = 0;
	for (i = 0; i < ori_cloud->points.size(); i++) {
		x = (int)((float)(ori_cloud->points.at(i).x - off_x) / pix);
		y = (int)((float)(ori_cloud->points.at(i).y - off_y) / pix);
		if (posIm.at<uchar>(x, y) >0 )  { countOut++; cloudOut->points.emplace_back(ori_cloud->points.at(i)); }
	}
	cloudOut->height = countOut; cloudOut->width = 1;

	*outputcloud = *cloudOut;

	cout << "\t finish facades extraction\n";
}

void EnhancePcd(pcl::PointCloud<PointT>::Ptr pointclouds, pcl::PointCloud<PointT>::Ptr output,
	float times=1, float offset=0) {
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	int total = pointclouds->points.size();
	pcl::copyPointCloud(*pointclouds, *output);
	int avg;

	for (int i = 0; i < total; i++) {
		avg = output->points.at(i).r;
		avg = avg * times + offset > 255 ? 255 : avg * times + offset;
		output->points.at(i).r = avg;
		output->points.at(i).g = avg;
		output->points.at(i).b = avg;
	}

}

void FascadesExtraction(pcl::PointCloud<PointT>::Ptr pointclouds, pcl::PointCloud<PointT>::Ptr outputclouds, float pix=0.2f) {
	//extract the point clouds
	//float pix = 0.2f;

	//******1. get the boundary of the point clouds
	PointT min_pt,max_pt;
	pcl::getMinMax3D(*pointclouds, min_pt, max_pt);
	

	////******2. set the pix and projection to the point clouds

	//// PCA：计算主方向
	//Eigen::Vector4f centroid;							// 质心
	//pcl::compute3DCentroid(*pointclouds, centroid);	// 齐次坐标，（c0,c1,c2,1）

	//Eigen::Matrix3f covariance;
	//computeCovarianceMatrixNormalized(*pointclouds, centroid, covariance);		// 计算归一化协方差矩阵

	//// 计算主方向：特征向量和特征值
	//Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	//Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
	////Eigen::Vector3f eigen_values = eigen_solver.eigenvalues();
	//eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));	// 校正主方向间垂直（特征向量方向： (e0, e1, e0 × e1) --- note: e0 × e1 = +/- e2）

	//// 转到参考坐标系，将点云主方向与参考坐标系的坐标轴进行对齐
	//Eigen::Matrix4f transformation(Eigen::Matrix4f::Identity());
	//transformation.block<3, 3>(0, 0) = eigen_vectors.transpose();										// R^(-1) = R^T
	//transformation.block<3, 1>(0, 3) = -1.f * (transformation.block<3, 3>(0, 0) * centroid.head<3>());	// t^(-1) = -R^T * t

	//pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);	// 变换后的点云
	//pcl::transformPointCloud(*pointclouds, *transformed_cloud, transformation);

	////PointT min_pt, max_pt;						// 沿参考坐标系坐标轴的边界值
	//pcl::getMinMax3D(*transformed_cloud, min_pt, max_pt);
	//const Eigen::Vector3f mean_diag = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());	// 形心

	//// 参考坐标系到主方向坐标系的变换关系
	//const Eigen::Quaternionf qfinal(eigen_vectors);
	//const Eigen::Vector3f tfinal = eigen_vectors * mean_diag + centroid.head<3>();

	//pcl::copyPointCloud(*transformed_cloud, *pointclouds);

	cout << "\t min and max boundary: " << min_pt << max_pt << endl;
	float off_x, off_y, off_z;
	int width, height;
	off_x = min_pt.x; off_y = min_pt.y; off_z = min_pt.z;
	width = (max_pt.x -off_x) / pix + 1;
	height = (max_pt.y - off_y) / pix + 1;
	cv::Mat proj = cv::Mat::zeros(cv::Size( height, width), CV_64FC1);
	int s = pointclouds->points.size();
	PointT pt;
	double top=0, down;
	cv::Mat heightMap = cv::Mat::zeros(proj.size(), CV_64FC1);
	for (int i = 0; i < s; i++) {
		pt = pointclouds->points.at(i);
		proj.at<double>((int)((pt.x - off_x) / pix), (int)((pt.y - off_y) / pix)) += 1;
		heightMap.at<double>((int)((pt.x - off_x) / pix), (int)((pt.y - off_y) / pix)) = 
			max(heightMap.at<double>((int)((pt.x - off_x) / pix), (int)((pt.y - off_y) / pix)),(((double)pt.z-(double)off_z)/ (double)pix));
		top = max(top, (((double)pt.z - (double)off_z) / (double)pix));
	}
	proj.convertTo(proj, CV_8UC1);
	cv::imshow("proj", proj); //cv::waitKey();
	//Process point clouds height map
	cv::Mat heighThread;
	cv::Point maxLoc, minLoc;
	//cv::minMaxLoc(proj, &down, &top, &minLoc, &maxLoc);
	//normalize(heightMap, heighThread, 0, 1, cv::NORM_MINMAX);
	float thread = 10.0 / pix;
	cv::threshold(proj, heighThread,thread, 255, cv::THRESH_BINARY);//binary image with enough height
	cv::imshow("heightMap", heighThread); //cv::waitKey();

	vector<vector<cv::Point>> points; cv::Mat facade;
	facade=clustering(heighThread, points, 200);
	facade=FacadeValidateByLine(points, facade, 50);
	cv::threshold(facade, facade, 80, 255,cv::THRESH_BINARY);
	
	/*for (int m = 0; m < facade.rows; m++) {
		facade.at<uchar>(m, 0) = 255;
		facade.at<uchar>(m, facade.cols-1) = 255;
	}
	for (int m = 0; m < facade.cols; m++) {
		facade.at<uchar>( 0,m) = 255;
		facade.at<uchar>(facade.rows-1 ,m) = 255;
	}*/
	cv::Mat dst = 255 - facade;
	//cv::distanceTransform(facade, dst, cv::DIST_L1, 3);
	cv::imshow("facade", dst); //cv::waitKey();
	
	//line detection
	//cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector();
	vector<cv::Vec4f> lines_std;
	//lsd->detect(dst, lines_std);lsd->drawSegments(image, lines);

	// Show found lines
	cv::Mat image = facade.clone()*0;

	double rho = 1.0;
	double theta = 1.0/180.0* CV_PI;
	double threshold = 13.0;
	double minLineLength = 12.0;
	double maxLineGap = 7.0;

	//Hough line transformation
	cv::HoughLinesP(facade, lines_std, rho,  theta, threshold, minLineLength, maxLineGap);

	//draw the lines
	cv::Mat LineImg;
	LineImg = facade.clone();
	cv::Vec4i LineStand;
	cv::cvtColor(LineImg, LineImg, cv::COLOR_GRAY2RGB);
	vector<cv::Vec4f> lines;
	for (int i = 0; i < lines_std.size(); i++) {
		if (lineDist(lines_std[i]) > 5.0 / (double)pix)lines.emplace_back(lines_std[i]);
	}
	
	for (int i = 0; i < lines.size(); i++) {
		LineStand = lines_std[i];
		cv::line(LineImg, cv::Point(LineStand[0], LineStand[1]), cv::Point(LineStand[2], LineStand[3]), cv::Scalar(0, 0, 255), 1);

	}
	cv::imshow("Hough lines", LineImg);cv::waitKey(0); cv::destroyAllWindows();
	//cv::imwrite("../lineinter.png", LineImg);

	cv::Mat Marker = FacadeCompliment(lines_std, lines, facade);
	cv::imshow("marker", Marker); cv::waitKey(); cv::destroyAllWindows();
	int radius; radius = 0.5/pix;//set radius for the buffer
	vector<cv::Point> pts;
	for (int j = 0; j < Marker.rows; j++) {//select the target points in the marker image
		for (int i = 0; i < Marker.cols; i++){
			if ((int)Marker.at<uchar>(j, i) > 0){
				pts.emplace_back(cv::Point(i, j));
			}
		}
	}
	cv::Mat buffer = cv::Mat::zeros(Marker.size(), CV_8UC1);
	for (int i = 0; i < pts.size(); i++) {
		cv::circle(buffer, pts[i], radius, 255, 1);
	}
	buffer=clustering(buffer, points, 1);
	cv::Mat buffer2 = 255 - buffer;//buffer2 inverse the non facade areas
	buffer2 = clustering(buffer2, points, 1);
	Marker = buffer + buffer2 + 3;

	FacadeSeperation(Marker,pointclouds, outputclouds, pix);
	
	//cv::minMaxLoc(proj, &down, &top, &minLoc,&maxLoc);
	cv::Mat output;
	proj.convertTo(proj, CV_8UC1);
	cv::Canny(proj, output, 120, 220);

	//vector<vector<cv::Point>> contours;
	//findContours(proj, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	//// perform watershed，分水岭变换
	//cv::Mat markers;

	//// generate random color，每个卡片赋予随机颜色
	//vector<cv::Vec3b> colors;
	//for (size_t i = 0; i < contours.size(); i++) {
	//	int r = cv::theRNG().uniform(0, 255);
	//	int g = cv::theRNG().uniform(0, 255);
	//	int b = cv::theRNG().uniform(0, 255);
	//	colors.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
	//}

	//// fill with color and display final result
	//vector<cv::Point> p;
	//cv::Mat dst = cv::Mat::zeros(proj.size(), CV_8UC3);
	//for (int i = 0; i < contours.size(); i++) {
	//	p = contours[i];
	//	for (int j = 0; j < p.size(); j++) {
	//		dst.at<cv::Vec3b>(p[j].x, p[j].y) = colors[i];
	//	}
	//}

	//cv::imshow("Final Result", dst);

	//cv::waitKey(0);

	//region glowing to detect the projected facades
	cv::Mat src = cv::Mat::zeros(proj.size(), CV_8UC3);
	vector<cv::Mat> channels2;
	channels2.push_back(proj.clone());
	channels2.push_back(proj.clone());
	channels2.push_back(proj.clone());
	cv::merge(channels2, src);

	
	cv::namedWindow("Projection");
	cv::imshow( "Projection", src);

	cv::Mat binaryImg;
	cv::threshold(proj.clone(), binaryImg, 40, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	//cv::imshow("binary image", binaryImg); 

	cv::Mat distImg;
	distanceTransform(binaryImg, distImg, cv::DIST_L1, 3, 5);//距离变换
	normalize(distImg, distImg, 0, 1, cv::NORM_MINMAX);
	cv::imshow("distance result", distImg);

	// binary again，再次二值化
	cv::threshold(distImg, distImg, .4, 1, cv::THRESH_BINARY);
	cv::Mat k1 = cv::Mat::ones(1, 1, CV_8UC1);
	cv::Mat element5(5, 5, CV_8U, cv::Scalar(1));
	//cv::morphologyEx(distImg, distImg, cv::MORPH_CLOSE, element5, cv::Point(-1, -1), 1);
	//cv::erode(distImg, distImg, k1, cv::Point(-1, -1));//二值腐蚀
	//imshow("distance binary image", distImg); 

	// markers，标记
	cv::Mat dist_8u;
	distImg.convertTo(dist_8u, CV_8U);
	vector<vector<cv::Point>> contours;
	cv::findContours(binaryImg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	// generate random color，每个卡片赋予随机颜色
	vector<cv::Vec3b> colors;
	for (size_t i = 0; i < contours.size(); i++) {
		int r = cv::theRNG().uniform(0, 255);
		int g = cv::theRNG().uniform(0, 255);
		int b = cv::theRNG().uniform(0, 255);
		colors.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
	}

	// create makers
	cv::Mat markers = cv::Mat::zeros(dist_8u.size(), CV_8UC3);
	int idx, si; si = contours[0].size(); idx = 0;
	for (size_t i = 0; i < contours.size(); i++) {
		drawContours(markers, contours, i, colors[i],-1);
		if (contours[i].size() > si) { si = contours[i].size(); idx = i; }
	}
	printf("In total %i contours, and the largest is No. %i", contours.size(), idx);
	//cv::circle(markers, cv::Point(5, 5), 3, cv::Scalar(255, 255, 255), -1);

	cv::Mat facades=cv::Mat::zeros(src.size(),CV_8UC1);
	cv::drawContours(facades, contours, idx, 255, -1);
	
	cv::imshow("Final Result", facades);

	cv::waitKey(0); cv::destroyAllWindows();

	//******3. define the target area
	

	//******4.process and finalize the result
}



void RegionGrow(pcl::PointCloud<PointT>::Ptr cloud,
	pcl::PointCloud<PointT>::Ptr output_planes) {
	//segmentation region growing
	pcl::PointCloud<PointT>::Ptr cloud_xyz(new pcl::PointCloud<PointT>);
	pcl::copyPointCloud(*cloud, *cloud_xyz);
	vector<string> filelist;

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	//pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> >(new pcl::search::KdTree<PointT>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud_xyz);
	normal_estimator.setKSearch(30);
	//normal_estimator.setRadiusSearch(0.3);
	normal_estimator.compute(*normals);


	pcl::RegionGrowing<PointT, pcl::Normal> reg;
	reg.setMinClusterSize(1000);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(cloud_xyz);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	//reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	//reg.setCurvatureThreshold(1.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;

	pcl::PCDWriter writer;
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it) {
		pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(cloud_xyz->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_(rg)" << j << ".pcd";
		std::cout << "Name of the file:" << ss.str() << endl;
		filelist.push_back(ss.str());
		writer.write<PointT>(ss.str(), *cloud_cluster, false); //*
		j++;
	}

	int counter = 0;
	std::cout << std::endl;

	/*pcl::PointCloud <PointT>::Ptr colored_cloud = reg.getColoredCloud();
	pcl::visualization::CloudViewer viewer("Cluster viewer");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped())
	{
	}*/

	
}


bool NotGroundPlane(pcl::PointCloud<PointT>::Ptr cloud, double thread) {
	//calculate the avergar normal vector
	int s = cloud->points.size();
	double x, y, z;
	x = 0; y = 0; z = 0;

	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	ne.setInputCloud(cloud);
	ne.setRadiusSearch(0.1);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	ne.compute(*cloud_normals);
	double num = 0;

	for (int ix = 0; ix < s; ix++)
	{
		if (isnan(cloud_normals->points[ix].normal_x) ||
			isnan(cloud_normals->points[ix].normal_y) ||
			isnan(cloud_normals->points[ix].normal_z))
		{
			continue;
		}
		else {
			num++;
			x += (double)cloud_normals->points[ix].normal_x;
			y += (double)cloud_normals->points[ix].normal_y;
			z += (double)cloud_normals->points[ix].normal_z;
		}
	}
	x = x / num; y = y / num; z = z / num;
	double l = sqrt(x * x + y * y + z * z);
	x = x / l; y = y / l; z = z / l;
	//printf("normal vector: %f, %f, %f", x, y, z);

	if (abs(z) <= thread) { return 1; }
	else { return 0; }
}


void GroundRemovel(pcl::PointCloud<PointT>::Ptr p_cloud,pcl::PointCloud<PointT>::Ptr output_cloud) {
	//remove grounds bu plane detection
	//Euclidean segmentation
	pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::copyPointCloud(*p_cloud, *cloud);
	std::cout << "Ransac planar segmentation" << std::endl;

	std::vector<string> filelist;

	//std::vector<std::string>* parameter = new std::vector<std::string>;
	//readParametersfromtxt(parafile, parameter);

	int iterativeNum = 1000, min_point = 3000;
	float SegThred = 0.6, ratio = 0.2;

	//create the segmentation object for the planar model and set all the parameters
	Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);
	pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(iterativeNum);
	seg.setDistanceThreshold(SegThred);
	seg.setAxis(axis);
	seg.setEpsAngle(3.0f * (M_PI / 180.0f));

	Eigen::Vector4f centroid;			

	PointT min_pt, max_pt;

	int i = 0, nr_points = (int)cloud->points.size();
	int j = 0;
	int min_Size = 1000, max_Size = 0;
	output_cloud->clear();
	filelist.clear();
	while (cloud->points.size() > ratio * nr_points) {
		//segment the larget planar componen from the remaining cloud
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0) {
			std::cout << "Could not estimate a planar model for the given dataset." << endl;
			break;
		}

		//Extract the planar inliers form the input cloud
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);

		//Get the point associated witht the planar surface
		extract.filter(*cloud_plane);
		//std::cout << "Point cloud representing the planar component:" << cloud_plane->points.size() << endl;

		if ((*cloud_plane).size() > min_point) {
			//pcl::getMinMax3D(*cloud_plane, min_pt, max_pt);
			std::stringstream ss;
			ss << "cloud_cluster(RanPlanar)_" << j << ".pcd";
			std::cout << "Name of the file:" << ss.str() << endl;
			//filelist.push_back(ss.str());
			writer.write<PointT>(ss.str(), *cloud_plane, false); //*
			j++;
			if ((*cloud_plane).size() < min_Size) min_Size = (*cloud_plane).size();
			if ((*cloud_plane).size() > max_Size) max_Size = (*cloud_plane).size();

			*output_cloud += *cloud_plane;

			//Remove the planar inliers, extract the rest
			extract.setNegative(true);
			extract.filter(*cloud_f);
			*cloud = *cloud_f;
		}
		
	}
	
	std::cout << "In total " << filelist.size() << " point segmentations" << std::endl;
	std::cout << "The max cloud size is " << max_Size << " points. And the min cloud size is " << min_Size << " points." << std::endl;
	//cloud->clear();
	cloud_plane->clear();
	//return 0;
}


int savePcdasTxt(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, string filename) {
	////save the point cloud as txt file
	char delimiter = '.'; // 特殊字符
	std::string result;

	// 查找特殊字符的位置
	std::size_t pos = filename.find(delimiter);
	if (pos != std::string::npos) {
		// 截取特殊字符之前的部分
		result = filename.substr(0, pos);
		filename = result;
	}
	else {
		std::cout << "Delimiter not found." << std::endl;
	}

	filename.append(".txt");
	FILE* file = fopen(filename.c_str(), "w");

	if (file != nullptr) {
		for (const pcl::PointXYZRGBA& point : cloud->points) {
			fprintf(file, "%f %f %f %u %u %u\n", point.x, point.y, point.z, point.r, point.g, point.b);
		}
	}
	else {
		PCL_ERROR("Failed to open the output TXT file.\n");
		return -1;
	}

	return 1;
}

int RanPlanCluSeg(pcl::PointCloud<PointT>::Ptr p_cloud,
	pcl::PointCloud<PointT>::Ptr output_cloud) {
	//Euclidean segmentation
	pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::copyPointCloud(*p_cloud, *cloud);
	std::cout << "Ransac planar segmentation" << std::endl;

	std::vector<string> filelist;

	//std::vector<std::string>* parameter = new std::vector<std::string>;
	//readParametersfromtxt(parafile, parameter);

	int iterativeNum = 100, min_point = 3000;
	float SegThred = 0.05, ratio = 0.2;
	//if (parameter->size() >= 4) {
	//	iterativeNum = atoi((*parameter)[0].c_str());
	//	SegThred = atof((*parameter)[1].c_str());
	//	ratio = atof((*parameter)[2].c_str());
	//	min_point = atoi((*parameter)[3].c_str());
	//}

	//create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(iterativeNum);
	seg.setDistanceThreshold(SegThred);

	Eigen::Vector4f centroid;							// 质心
	PointT min_pt, max_pt;
	pcl::getMinMax3D(*p_cloud, min_pt, max_pt);
	//pcl::compute3DCentroid(*p_cloud, centroid);
	double Base = (double)min_pt.z;

	int i = 0, nr_points = (int)cloud->points.size();
	int j = 0;
	int min_Size = 1000, max_Size = 0;
	output_cloud->clear();
	filelist.clear();
	while (cloud->points.size() > ratio * nr_points) {
		//segment the larget planar componen from the remaining cloud
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0) {
			std::cout << "Could not estimate a planar model for the given dataset." << endl;
			break;
		}

		//Extract the planar inliers form the input cloud
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);

		//Get the point associated witht the planar surface
		extract.filter(*cloud_plane);
		//std::cout << "Point cloud representing the planar component:" << cloud_plane->points.size() << endl;

		if ((*cloud_plane).size() > min_point) {
			pcl::getMinMax3D(*cloud_plane, min_pt, max_pt);
			if (NotGroundPlane(cloud_plane, 0.1) ) {
				std::stringstream ss;
				ss << "cloud_cluster(RanPlanar)_" << j << ".pcd";
				std::cout << "Name of the file:" << ss.str() << endl;
				//filelist.push_back(ss.str());
				writer.write<PointT>(ss.str(), *cloud_plane, false); //*
				j++;
				if ((*cloud_plane).size() < min_Size) min_Size = (*cloud_plane).size();
				if ((*cloud_plane).size() > max_Size) max_Size = (*cloud_plane).size();

				*output_cloud += *cloud_plane;
			}
			

		}
		//Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*cloud = *cloud_f;

		
	}

	std::cout << "In total " << filelist.size() << " point segmentations" << std::endl;
	std::cout << "The max cloud size is " << max_Size << " points. And the min cloud size is " << min_Size << " points." << std::endl;
	cloud->clear();
	cloud_plane->clear();
	return 0;
}

void Planeseg_regiongrow(pcl::PointCloud<PointT>::Ptr cloud,
	pcl::PointCloud<PointT>::Ptr output_planes) {
	//segment the result for
	//convert pointclous to CGAl type
	output_planes->clear();
	Pointn convertpt; Point_set point_set;
	Pwn_vector points;
	for (pcl::PointCloud<PointT>::iterator it1 = cloud->begin(); it1 != cloud->end(); ++it1) {
		point_set.insert(Pointn(it1->x, it1->y, it1->z));
	}
	point_set.add_normal_map();
	for (int i = 0; i < static_cast<int>(point_set.size()); i++) {
		Point_with_normal pt;
		pt.first = point_set.point(i);
		pt.second = point_set.normal(i);
		points.emplace_back(pt);
	}

	//shape detection
	Region_growing shape_detection;// Instantiates shape detection engine.
	shape_detection.set_input(points);// Provides the input data.
	shape_detection.template add_shape_factory<Plane>();// Registers planar shapes via template method.
	shape_detection.detect();// Detects registered shapes with default parameters.
	std::cout << shape_detection.shapes().end() - shape_detection.shapes().begin() << " shapes detected." << std::endl;

	//convert the point to point cloud
	pcl::PointCloud<PointT>::Ptr Ccloud(new pcl::PointCloud<PointT>);
	(*Ccloud).width = (*cloud).width; (*Ccloud).height = (*cloud).height; (*Ccloud).is_dense = false;
	(*Ccloud).points.resize((*Ccloud).height * (*Ccloud).width);

	int count = 0;
	//std::vector<PCXYZPtr> output_planes;

	Region_growing::Shape_range shapes = shape_detection.shapes();
	Region_growing::Shape_range::iterator its = shapes.begin();

	cv::RNG rng(12345);

	while (its != shapes.end()) {
		pcl::PointCloud<PointT>::Ptr t_c(new pcl::PointCloud<PointT>);
		(*t_c).width = (*cloud).width; (*t_c).height = (*cloud).height; (*t_c).is_dense = false;

		pcl::PointXYZ pt;
		if (Plane* plane = dynamic_cast<Plane*>(its->get())) {
			std::cout << "Plane_" << count++ << std::endl;
			cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			const std::vector<size_t> indices = its->get()->indices_of_assigned_points();
			std::vector<size_t>::const_iterator iti = indices.begin();
			int i = 0;
			try {
				(*t_c).points.resize(indices.end() - indices.begin());
				while (iti != indices.end()) {
					// Retrieves point
					Point_with_normal pw = *(points.begin() + (*iti));
					kernel::Point_3 p = pw.first;
					//std::cout << "POINT[" << p.x() << "," << p.y() << "," << p.z() << "]" << std::endl;
					t_c->points[i].x = p.x(); t_c->points[i].y = p.y(); t_c->points[i].z = p.z();
					t_c->points[i].r = color[1]; t_c->points[i].g = color[2]; t_c->points[i].b = color[3];
					// Proceeds with next point.
					iti++; i++;
				}
			}
			catch (std::exception& e) {
				std::cout << e.what() << std::endl;
			}

		}
		its++;
		*Ccloud += *t_c;
		//PCXYZPtr t_cloud(new PCXYZ);
		if ((*t_c).points.size() > 5000) {
			pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>());
			pcl::copyPointCloud(*t_c, *temp_cloud);
			*output_planes+=(*temp_cloud);
		}
		t_c->clear(); //temp_cloud->clear();
	}

}

cv::Mat drawHist(int* hist, int bins = 255) {
	//plot the histgram
	cv::Mat histImage = cv::Mat::zeros(540, 1020, CV_8UC1);
	//const int bins = 255;

	int maxValue = 0;
	for (int i = 0; i <= 255; i++) {
		maxValue = std::max(maxValue, hist[i]);
	}

	cv::Point2i maxLoc;
	//cv::minMaxLoc(hist, 0, &maxValue, 0, &maxLoc);
	int scale = 4;
	int histHeight = 540;

	for (int i = 0; i < bins; i++)
	{
		float binValue = (hist[i]);
		int height = cvRound(binValue * histHeight / maxValue);
		cv::rectangle(histImage, cv::Point(i * scale, histHeight),
			cv::Point((i + 1) * scale - 1, histHeight - height), cv::Scalar(255), -1);

	}
	return histImage;
}

void Dx(double a, Eigen::Matrix4d& D) {									// rotation x-axis
	D.setIdentity();
	D(0, 0) = 1;	D(0, 1) = 0;	D(0, 2) = 0;
	D(1, 0) = 0;	D(1, 1) = cos(a);	D(1, 2) = sin(a);
	D(2, 0) = 0;	D(2, 1) = -sin(a);	D(2, 2) = cos(a);
}

void Dy(double a, Eigen::Matrix4d& D) {									// rotation y-axis
	D.setIdentity();
	D(0, 0) = cos(a);	D(0, 1) = 0;	D(0, 2) = sin(a);
	D(1, 0) = 0;	D(1, 1) = 1;	D(1, 2) = 0;
	D(2, 0) = -sin(a);	D(2, 1) = 0;	D(2, 2) = cos(a);
}

void Dz(double a, Eigen::Matrix4d& D) {									// rotation z-axis
	D.setIdentity();
	D(0, 0) = cos(a);	D(0, 1) = sin(a);	D(0, 2) = 0;
	D(1, 0) = -sin(a);	D(1, 1) = cos(a);	D(1, 2) = 0;
	D(2, 0) = 0;	D(2, 1) = 0;	D(2, 2) = 1;
}

void
renderThermal(pcl::PointCloud<PointT>::Ptr inputcloud,string fileroot, int startIm,int endIm,
	string posInfo,
	pcl::PointCloud<PointT>::Ptr outputcloud) {
	printf("render the point clouds with thermal images %i to %i\n",startIm,endIm);
	
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
	cv::Mat image0;
	cv::Mat image1;
	cv::Mat image2;
	cv::Mat map1;
	cv::Mat map2;
	cv::Mat cameraMatrix;
	cv::Mat newCameraMatrix;
	cv::Mat distCoeffs;
	cv::Size imageSize;
	char buffer[100];
	string imname;
	string filelasf;
	string outpcdname;
	ifstream fidnav;
	ifstream fidsyncpcd;
	double pcd_nrf;
	double x;
	double y;
	double focal_length;
	double d_int_x;
	double d_int_y;
	double d_int_z;
	double n_roll = 0.0;
	double n_pitch = 0.0;
	double n_yaw = 0.0;
	int64 mf;
	int ie;
	int i;
	int j;
	int ii;
	int ind1;
	int ind2;
	int jj;
	int kk;
	int xi;
	int yi;
	int anz_z;
	int anz_p;
	int pcd_nr;
	int n_frame;
	int valu;
	int avggray;
	int avggraycount;
	int waspcd = 0;
	int xx = 480;
	int yy = 640;

	PointT texpoint;
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	cv::Mat img_dist;
	cv::Mat img_idx;

	Dx(-0.5 * (M_PI / 180.0) + M_PI, R_rot);						// boresight angles of IR camera
	Dy(-10.6 * (M_PI / 180.0), R_ele);
	Dz(-119.9 * (M_PI / 180.0), R_az);
	R_int = R_rot * R_ele * R_az;
	d_int_x = -0.92;												// lever arm of IR camera
	d_int_y = -0.50;
	d_int_z = -0.36;

	mf = 15311;
	focal_length = 500.0;																					// focal length of IR camera (estimated)
	cameraMatrix = (cv::Mat1d(3, 3) << 500.0, 0.0, 320, 0.0, 500.0, 240, 0.0, 0.0, 1.0);	// optical axis (estimated) in image center 
	distCoeffs = (cv::Mat1d(1, 5) << -0.23, 0.0, 0.000, 0.000, -0.0);												// distortion of IR camera (estimated)
	imageSize.width = yy;
	imageSize.height = xx;
	newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0, imageSize, 0, true);
	initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), newCameraMatrix, imageSize, CV_32FC1, map1, map2);

	pos(3) = 0.0;
	pos2(0) = 0.0;
	pos2(1) = 0.0;
	pos2(2) = 0.0;
	pos2(3) = 0.0;
	fidnav.open(posInfo);
	//fidsyncpcd.open(pos.c_str());
	int count = 0;
	int total = inputcloud->points.size();
	int* pr = new int[total] {0};

	for (ie = startIm; ie <= endIm; ie++) {		// process sequence of IR images

		fidnav >> n_frame >> pos(0) >> pos(1) >> pos(2) >> n_roll >> n_pitch >> n_yaw;
		while (n_frame < ie)fidnav >> n_frame >> pos(0) >> pos(1) >> pos(2) >> n_roll >> n_pitch >> n_yaw;
		pos2 -= pos;
		if (pos2.norm() < 0.02) ind2 = 0;
		else ind2 = 1;
		//if (ie == min_frame) ind2 = 0;
		Dx(n_roll * M_PI / 180.0, Rx);
		Dy(-n_pitch * M_PI / 180.0, Ry);
		Dz(n_yaw * M_PI / 180.0, Rz);
		R = Rx * Ry * Rz;

		imname = fileroot;
		imname.append("img_");
		sprintf_s(buffer, "%05d", n_frame);
		imname.append(buffer);
		imname.append(".tif");
		image0 = cv::imread(imname,-1);
		image1 = cv::Mat::zeros(xx, yy, CV_8UC1);

		for (i = 0; i < xx; i++) {
			for (j = 0; j < yy; j++) {
				valu = 80 + ((image0.at<uint16_t>(i, j) - mf) / 3);
				if (valu < 0) valu = 0;
				if (valu > 255) valu = 255;
				image1.at<uchar>(i, j) = (uchar)valu;
			}
		}

		cv::remap(image1, image2, map1, map2, cv::INTER_CUBIC);			// undistort IR image

		/*texcloud->width=count--;
		texcloud->height=1;*/

		
		std::cout << n_frame << "\t";
		img_dist = cv::Mat::zeros(xx, yy, CV_32FC1);
		img_idx = cv::Mat::ones(xx, yy, CV_32FC1) * -1;
		
		for (j = 0; j < total; j++) {

			//if (texcloud->at(j).label > 0)continue;

			ind1 = 1;
			if (!ind2) ind1 = 0;
			//std::cout << i;
			p(0) = inputcloud->at(j).x - pos(0);
			p(1) = inputcloud->at(j).y - pos(1);
			p(2) = inputcloud->at(j).z - pos(2);
			p(3) = 1.0;
			//std::cout << j<<"\t";

			p(3) = p(0);									// conversion ENU to NED
			p(0) = p(1);
			p(2) = -p(2);
			p(1) = p(3);
			p(3) = 1.0;
			tt = R * p;
			tt(0) -= d_int_x;
			tt(1) -= d_int_y;
			tt(2) -= d_int_z;
			tt = R_int * tt;
			x = 0.0;
			y = 0.0;
			if (tt(0) > 1.0) {								// projection of 3D points to focal plane
				/*if (tt(0) > 80.0) {
					ind1 = 0;
				}*/
				ind1 = 1;
				x = newCameraMatrix.at<double>(1, 2) - newCameraMatrix.at<double>(1, 1) * tt(2) / tt(0);
				y = newCameraMatrix.at<double>(0, 2) - newCameraMatrix.at<double>(0, 0) * tt(1) / tt(0);
				xi = (int)x;
				yi = (int)y;
				if (ind1) {
					if ((xi > 0) && (xi < (xx - 1)) && (yi > 0) && (yi < (yy - 1))) {
						avggray = (int)(image2.at<uchar>(xi, yi)) + 1;
						inputcloud->at(j).r = avggray;// *1.6 > 255 ? 255 : avggray * 1.6;// (uint8_t)avggray * 2 > 255 ? 255 : (uint8_t)avggray;
						inputcloud->at(j).g = inputcloud->at(j).r;
						inputcloud->at(j).b = inputcloud->at(j).r;
						pr[j] = 1;
					}
				}
			}

		}

	}

	cloud->clear(); int N = 0;
	pcl::PointCloud<PointT>::Ptr cld(new pcl::PointCloud<PointT>);
	for (j = 0; j < total; j++) {

		if (pr[j] > 0) {
			cloud->push_back(inputcloud->points.at(j));
			N++;
		}
	}
	cloud->height = 1; cloud->width = N;
	cout << "\t pointNum: " << N << endl;

	fidnav.close();
	fidnav.clear();
	std::cout << endl;
	pcl::copyPointCloud(*cloud, *outputcloud);
}

void getFiles(string fid, vector<string>& files) {
	//read all the files in the folder fid
	//fid: folder id
	//files: the file names in the folder
	files.clear();
	for (const auto& entry : fs::directory_iterator(fid)) {
		files.push_back(entry.path().filename().string());
	}
		

}

void getSpecificFiles(string dire, string suffix, vector<string>& files) {
	//read all the files in the folder fid
	//fid: folder id
	//files: the file names in the folder
	files.clear();
	for (const auto& entry : fs::directory_iterator(dire)) {
		if (entry.path().extension().string() == suffix) {
			std::string filename = entry.path().filename().string();
			std::string extension = entry.path().extension().string();

			// 检查文件名是否包含特定名称并且文件格式为 '.pcd'
			if (filename.find("sub_cloud_xx") != std::string::npos && extension == ".pcd") {
				files.push_back(filename); // 将符合条件的文件名保存到 vector 中
			}

			//files.push_back(entry.path().filename().string());  
		}
	}
	printf("/t intotal %i files\n", files.size());
}

double computeVectorAngle(const octomap::point3d& vector1, const octomap::point3d& vector2)
{
	double dotProduct = vector1.x() * vector2.x() +
		vector1.y() * vector2.y() +
		vector1.z() * vector2.z();
	double magnitudesProduct = std::sqrt(vector1.x()*vector1.x()+ vector1.y() * vector1.y() + vector1.z() * vector1.z() ) * 
		std::sqrt(vector2.x() * vector2.x() + vector2.y() * vector2.y() + vector2.z() * vector2.z());
	assert(magnitudesProduct != 0.0);

 	if (magnitudesProduct == 0.0)
	{
		return 0.0; // Handle division by zero
	}

	double cosineAngle = dotProduct / magnitudesProduct;
	double angleRadians = std::acos(cosineAngle);

	// Limit the angle to be between 0 and 90 degrees (0 and π/2 radians)
	if (angleRadians > M_PI / 2.0)
	{
		angleRadians = M_PI - angleRadians;
	}

	return angleRadians/M_PI*180.0;
}