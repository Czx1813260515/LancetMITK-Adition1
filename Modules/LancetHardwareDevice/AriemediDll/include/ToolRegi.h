#ifndef TOOLREGI_HPP
#define TOOLREGI_HPP

#ifdef ARMDCOMBINEDAPI_EXPORTS
#define ARMDCOMBINED_API __declspec(dllexport)
#else
#define ARMDCOMBINED_API __declspec(dllimport)
#endif

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>

namespace TransformationStatus
{
	enum values
	{
		Enabled = 0x0000,
		PartiallyOutOfVolume = 0x0001,
		OutOfVolume = 0x0002,
		PartiallyMatched = 0x0003,
		TooFewMarkers = 0x0004,
		BadTransformFit = 0x0005,
		// = 0x0006,
		// = 0x0007,
		// = 0x0008,
		ToolMissing = 0x0009,
	};

	//converts the error code to std::string representation
	ARMDCOMBINED_API std::string toString(uint16_t errorCode);
}

struct ToolPlaneMarker
{
	std::vector<std::vector<double>> regiSet;
	std::vector<std::vector<double>> regiDifSet;
};

struct MarkerPosition
{
	double P[4];//marker position (x,y,z,1)
	int leftExposure = 0;//marker exposure
	int rightExposure = 0;//marker exposure
	int type = 0;//0(unknown), 1(passive), 2(active)
};

struct Transformation
{
	uint16_t status = TransformationStatus::ToolMissing;//transformation status
	double matrix[4][4];//transformation matrix [R|t]
	double invmatrix[4][4];//inverse transformation matrix [R|t]
	double qw, qx, qy, qz;//transformation quaternion
	double tx, ty, tz;//tip position
	double error;//matching error
};

struct ToolTrackingData
{
	std::string name = "";//tool name
	Transformation transform;//transform information
	int matchStatus = 0;//matching status
	int matchedPlane = 0;//matched plane index
	int matchedMarkersNum = 0;//matched markers number on the plane
	std::vector<MarkerPosition> markers;//positions of all markers on the matched plane
	std::string timespec = "";//acquisition time
};

class ToolRegi
{
public:
	ToolRegi(int version);
	~ToolRegi();

	//input the tool to be tracked
	void SetInputData(std::vector<std::vector<MarkerPosition>> source, std::vector<MarkerPosition> target, std::vector<ToolPlaneMarker> planeMarker, double tip[3], int minMatchingNum, double maxFRE);
	
	//match markers to tool
	bool Register();
	
	//get tool tracking data
	void GetTrackingData(ToolTrackingData& data);

	//inverse tool tracking data
	static bool InverMatrix(double Matrix[4][4], double invMatrix[4][4]);

protected:
	int deviceVersion = 0;

	ToolTrackingData trackingData;
	
	std::vector<std::vector<MarkerPosition>> toolMarker;
	std::vector<MarkerPosition> targetMarker;
	double tipPoint[3];
	double maxFRE = 1;
	int minRegNum = 3;

	std::vector<ToolPlaneMarker> toolPlaneMarker;
	std::vector<int> order;

	//matching function
	std::vector<MarkerPosition> arrangeTri(std::vector<MarkerPosition> src, bool record, std::vector<double> set);
	bool triMatch(std::vector<MarkerPosition> a, std::vector<MarkerPosition> b);
	void pointRegister(std::vector<MarkerPosition> source, std::vector<MarkerPosition> target, double Matrix[4][4]);
	int jacobiN(double** a, int n, double* w, double** v);
	void perpendiculars(const double v1[3], double v2[3], double v3[3], double theta);
	static double eucDist(double* a, double* b);
	MarkerPosition matrixPlusPoint(double Matrix[4][4], MarkerPosition point);

	//post-processing of matching result 
	void mat2Quat(Transformation& tran);
	std::vector<MarkerPosition> reOrderMarkers(std::vector<MarkerPosition>, std::vector<double> set, std::vector<double> difset, int planeIndex);

	//transformation status detection function
	float getCosine(std::vector<float> Vector1, std::vector<float> Vector2);
	std::vector<float> getVector(MarkerPosition p1, MarkerPosition p2);
	float getDis(MarkerPosition p1, MarkerPosition p2);
	std::vector<float> use3PointsComputeNormalVector(MarkerPosition p1, MarkerPosition p2, MarkerPosition p3);
	MarkerPosition getProjectionPt(MarkerPosition p, MarkerPosition p1, MarkerPosition p2, MarkerPosition p3);
	int judgePtInOut(MarkerPosition p, std::vector<MarkerPosition> Pts);
	int judgePtInOut_SE(std::vector<MarkerPosition> Pts);
	int judgePtInOut_MAX(std::vector<MarkerPosition> Pts);
	std::vector<MarkerPosition> loadVertexSE();
	std::vector<MarkerPosition> loadVertexMAX();
};
#endif