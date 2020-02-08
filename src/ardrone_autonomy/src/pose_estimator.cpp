#include<math.h>
#include<iostream>
#include"Eigen/Dense"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

cv::RotatedRect codeBoundBox;

double calc_a0(double phi_1, double phi_2, double p1, double p2, double d12, double b) {
	return -2 * phi_1 * phi_2 * p1 * pow(p2, 2) * d12 * b
		+ pow(phi_2, 2) * pow(p2, 2) * pow(d12, 2) + 2 * pow(p1, 3) * d12
		- pow(p1, 2) * pow(d12, 2) + pow(phi_2, 2) * pow(p1, 2) * pow(p2, 2)
		- pow(p1, 4) - 2 * pow(phi_2, 2) * p1 * pow(p2, 2) * d12
		+ pow(phi_1, 2) * pow(p1, 2) * pow(p2, 2) + pow(phi_2, 2) * pow(p2, 2) * pow(d12, 2) * pow(b, 2);
}


double calc_a1(double phi_1, double phi_2, double p1, double p2, double d12, double b) {
	return 2 * pow(p1, 2) * p2 * d12 * b + 2 * phi_1 * phi_2 * pow(p2, 3) * d12
		- 2 * pow(phi_2, 2) * pow(p2, 3) * d12 * b - 2 * p1 * p2 * pow(d12, 2) * b;
}


double calc_a2(double phi_1, double phi_2, double p1, double p2, double d12, double b) {
	return -pow(phi_2, 2) * pow(p1, 2) * pow(p2, 2) - pow(phi_2, 2) * pow(p2, 2) * pow(d12, 2) * pow(b, 2)
		- pow(phi_2, 2) * pow(p2, 2) * pow(d12, 2) + pow(phi_2, 2) * pow(p2, 4) + pow(phi_1, 2) * pow(p2, 4)
		+ 2 * p1 * pow(p2, 2) * d12 + 2 * phi_1 * phi_2 * p1 * pow(p2, 2) * d12 * b
		- pow(phi_1, 2) * pow(p1, 2) * pow(p2, 2) + 2 * pow(phi_2, 2) * p1 * pow(p2, 2) * d12
		- pow(p2, 2) * pow(d12, 2) * pow(b, 2) - 2 * pow(p1, 2) * pow(p2, 2);
}


double calc_a3(double phi_1, double phi_2, double p1, double p2, double d12, double b) {
	return 2 * pow(p2, 3) * d12 * b + 2 * pow(phi_2, 2) * pow(p2, 3) * d12 * b
		- 2 * phi_1 * phi_2 * pow(p2, 3) * d12;
}


double calc_a4(double phi_1, double phi_2, double p1, double p2, double d12, double b) {
	return - pow(phi_2, 2) * pow(p2, 4) - pow(phi_1, 2) * pow(p2, 4) - pow(p2, 4);
}

double calc_distance(Eigen::Vector3d p1, Eigen::Vector3d p2) {
	return sqrt(pow(p2(0) - p1(0), 2) + pow(p2(1) - p1(1), 2) + pow(p2(2) - p1(2), 2));
}

double calc_b(Eigen::Vector3d f1, Eigen::Vector3d f2) {
	double dot = f1.dot(f2);
	double b = sqrt((1 / (1 - pow(dot, 2)) ) - 1);
	if (dot < 0)
		return -1 * b;
	else return b;
}

Eigen::Vector4d calc_roots(double a, double b, double c, double d, double e) {
	double p = (8 * a * c - 3 * pow(b, 2)) / (8 * pow(a, 2));
	double q = (pow(b, 3) - 4 * a * b * c + 8 * pow(a, 2) * d) / (8 * pow(a, 3));

	double delta_0 = pow(c, 2) - 3 * b * d + 12 * a * e;
	double delta_1 = 2 * pow(c, 3) - 9 * b * c * d + 27 * pow(b, 2) * e + 27 * a * pow(d, 2) + 72 * a * c * e;

	double Q = cbrt(delta_1 + sqrt(pow(delta_1, 2) - 4 * pow(delta_0, 3)) / 2);
	double S = .5 * sqrt(-2 / 3 * p + 1 / (3 * a) * (Q + delta_0 / Q));

	Eigen::Vector4d roots;
	roots(0) = -b / (4 * a) - S + .5 * sqrt(-4 * pow(S, 2) - 2 * p + q / S);
	roots(1) = -b / (4 * a) - S - .5 * sqrt(-4 * pow(S, 2) - 2 * p + q / S);
	roots(2) = -b / (4 * a) + S + .5 * sqrt(-4 * pow(S, 2) - 2 * p - q / S);
	roots(3) = -b / (4 * a) + S - .5 * sqrt(-4 * pow(S, 2) - 2 * p - q / S);

	return roots; // special cases ?
}

Eigen::Vector3d calc_C_eta(double alpha, double theta, double d12, double b) {

	double cos_alpha = alpha / (sqrt(pow(alpha, 2) + 1));
	double sin_alpha = 1 / sqrt(1 + pow(alpha, 2));          // +/-
	double sin_theta = sqrt(1 - pow(theta, 2));				// +/-

	Eigen::Vector3d C_eta;
	C_eta(0) = d12 * cos_alpha * (sin_alpha * b + cos_alpha);
	C_eta(1) = d12 * sin_alpha * theta * (sin_alpha * b + cos_alpha);
	C_eta(2) = d12 * sin_alpha * sin_theta * (sin_alpha * b + cos_alpha);

	return C_eta;
}

Eigen::Matrix3d calc_Q(double alpha, double theta) {

	double cos_alpha = alpha / (sqrt(pow(alpha, 2) + 1));
	double sin_alpha = 1 / sqrt(1 + pow(alpha, 2));          // +/-
	double sin_theta = sqrt(1 - pow(theta, 2));				// +/-

	Eigen::Matrix3d Q;
	
	Q(0, 0) = -cos_alpha;	Q(0, 1) = -sin_alpha * theta;	Q(0, 2) = -sin_alpha * sin_theta;
	Q(1, 0) = sin_alpha;	Q(1, 1) = -cos_alpha * theta;	Q(1, 2) = -cos_alpha * sin_theta;
	Q(2, 0) = 0;			Q(2, 1) = -sin_theta;			Q(2, 2) = theta;

	return Q;
}

void perspective_3_point() {

	Eigen::Vector3d P1(4, 0, .5);
	Eigen::Vector3d P2(4, 0, 0);
	Eigen::Vector3d P3(4, -.5, 0);

	double focalLength = 50;
	double px = 0;
	double py = 0;
	double skew = 0;

	Eigen::Matrix3d K;
	K << focalLength,	skew,				px,
		 0,				focalLength,		py,
		 0,				0,					1;

	// place holder, extract from image
	Eigen::Vector3d q1(0, 50, 1);
	Eigen::Vector3d q2(0, 0, 1);
	Eigen::Vector3d q3(50, 0, 1);

	Eigen::Vector3d f1 = K.inverse() * q1;
	Eigen::Vector3d f2 = K.inverse() * q2;
	Eigen::Vector3d f3 = K.inverse() * q3;

	f1 /= f1.norm(); 
	f2 /= f2.norm();
	f3 /= f3.norm();

	Eigen::Vector3d tx = f1;
	Eigen::Vector3d tz = f1.cross(f2) / (f1.cross(f2)).norm();
	Eigen::Vector3d ty = tz.cross(tx);

	Eigen::Matrix3d T;
	T << tx, ty, tz;
	T.transposeInPlace();

	Eigen::Vector3d nx = P2 - P1 / (P2- P1).norm();
	Eigen::Vector3d nz = nx.cross(P3 - P1) / (nx.cross(P3- P1)).norm();
	Eigen::Vector3d ny = nz.cross(nx);

	Eigen::Matrix3d N;
	N << nx, ny, nz;
	N.transposeInPlace();

	Eigen::Vector3d P3_eta = N * (P3 - P1);

	double p1 = P3_eta(0);
	double p2 = P3_eta(1);

	double d12 = calc_distance(P1, P2);
	double b = calc_b(f1, f2);

	Eigen::Vector3d f3_tau = T * f3;

	double phi_1 = f3_tau(0) / f3_tau(2);
	double phi_2 = f3_tau(1) / f3_tau(2);

	double a0 = calc_a0(phi_1, phi_2, p1, p2, d12, b);
	double a1 = calc_a1(phi_1, phi_2, p1, p2, d12, b);
	double a2 = calc_a2(phi_1, phi_2, p1, p2, d12, b);
	double a3 = calc_a3(phi_1, phi_2, p1, p2, d12, b);
	double a4 = calc_a4(phi_1, phi_2, p1, p2, d12, b);

	Eigen::Vector4d roots = calc_roots(a0, a1, a2, a3, a4); // right order?

	Eigen::Vector4d cot_alpha;
	for (int i = 0; i < 4; i++) {
		cot_alpha(i) = (phi_1 / phi_2 * p1 + roots(i) * p2 - d12 * b) / (phi_1 / phi_2 * roots(i) * p2 - p1 + d12);
	}

	Eigen::MatrixXd C_etas(3, 4);
	for (int i = 0; i < 4; i++) {
		C_etas.col(i) = calc_C_eta(cot_alpha(i), roots(i), d12, b);
	}

	Eigen::Matrix3d Q [4];
	for (int i = 0; i < 4; i++) {
		Q[i] = calc_Q(cot_alpha(i), roots(i));
	}

	Eigen::MatrixXd C(3, 4);
	Eigen::Matrix3d R [4];
	for (int i = 0; i < 4; i++) {
		C.col(i) = P1 + N.transpose() * C_etas.col(i);
		R[i] = N.transpose() * Q[i].transpose() * T;
	}

	std::cout << C << std::endl;
}

cv::RotatedRect findMarker(cv::Mat image){
    cv::Mat gray;
    cv::cvtColor(image, gray, CV_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);
    cv::Mat edges;
    cv::Canny(gray, edges, 35, 125);

    cv::imshow("edges", edges);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(edges, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

    int maxArea = cv::contourArea(contours[0]);
    int index = 0;

    for(int i = 1; i < contours.size(); i++){
        if(cv::contourArea(contours[i]) > maxArea){
            maxArea = cv::contourArea(contours[i]);
            index = i;
        }
    }

    return cv::minAreaRect(contours[index]);
  }

void image_callback(const sensor_msgs::ImageConstPtr& msg){

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  codeBoundBox = findMarker(cv_ptr->image);

  cv::rectangle(cv_ptr->image, codeBoundBox.boundingRect(), cv::Scalar( 0, 255, 0 ), 2, 1 );

  cv::imshow("box", cv_ptr->image);
  cv::waitKey(3);
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "pose_estimator");

  ros::NodeHandle node;
  image_transport::ImageTransport it(node);

  image_transport::Subscriber imageSub = it.subscribe("/ardrone/front/image_raw", 1, image_callback);

  perspective_3_point();

  std::cout << "Done" << std::endl;

  ros::spin();

  return 0;
}


