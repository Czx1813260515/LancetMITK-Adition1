#ifndef ARMDCOMBINEDAPI_H
#define ARMDCOMBINEDAPI_H

#ifdef ARMDCOMBINEDAPI_EXPORTS
#define ARMDCOMBINED_API __declspec(dllexport)
#else
#define ARMDCOMBINED_API __declspec(dllimport)
#endif

#include "Tracking.h"
#include "ToolRegi.h"

#include <map>
#include <chrono>
#include <iomanip>
#include <cctype>
#include <io.h>

namespace DeviceType
{
	enum value
	{
		None = 0x0000,
		SE = 0x0001,
		LITE = 0x0002,
		MAX = 0x0003,
		MAXV = 0x0004,
		PRO = 0x0005,
		TD = 0x0006,
	};
}

namespace GenerationStatus
{
	enum value
	{
		OK = 0x0000,
		MinMatchedNumLess = 0x0001,
		MinMatchedNumMore = 0x0002,
		PlaneNumLess = 0x0003,
		PlaneNumMore = 0x0004,
		PlaneNumNotMatching = 0x0005,
		MarkerNumLess = 0x0006,
		MarkerNumMore = 0x0007,
		MarkerNumLimitExceed = 0x0008,
		DistanceTooSmall = 0x0009,
		NameError = 0x000A,
		PathNotExist = 0x000B,
		TypeError = 0x000C,
	};

	//return the std::string representation of the AROM generation status
	ARMDCOMBINED_API std::string toString(uint16_t generationCode);
}

namespace TransmissionStatus
{
	enum value
	{
		NoneData = 0x0000,
		TrackingData = 0x0001,
		ImagingData = 0x0002,
		AllData = 0x0003,
		Suspend = 0x0004,
		ReconstructionReady = 0x0005,
	};
}

namespace TransmissionType
{
	enum value
	{
		None = 0x0000,
		Passive = 0x0001,
		Active = 0x0002,
		Mixed = 0x0003,
		Alternate = 0x0004,
	};
}

namespace ConnectionStatus
{
	enum value
	{
		Connected = 0x0000,
		DisConnected = 0x0001,
		DisConnectedFailed = 0x0002,
		PortConflicts = 0x0003,
		ResloveError = 0x0004,
		InitialUDPError = 0x0005,
		Interruption = 0x0006,
		InvalidAddr = 0x0007,
		DeviceInexist = 0x0008,
		UnknownError = 0x0009,
	};

	//return the std::string representation of the connection status
	ARMDCOMBINED_API std::string toString(uint16_t conditionCode);
}

namespace DeviceAlert
{
	enum value
	{
		Normal = 0x0000,
		Vibration = 0x0001,
		TempTooHigh = 0x0002,
		TempTooLow = 0x0003,
	};

	//return the std::string representation of the device alert
	ARMDCOMBINED_API std::string toString(uint16_t alertCode);
}

namespace PivotAlert
{
	enum value
	{
		Normal = 0x0000,
		AbnormalData = 0x0001,
		InadequateData = 0x0002,
		ComputationError = 0x0003,
	};

	//return the std::string representation of the calibration alert
	ARMDCOMBINED_API std::string toString(uint16_t alertCode);
}

struct ToolCalibrationData
{
	std::string name = "";//tool name
	bool type = false;//passive(false) or active(true) tool
	int minNumMarker = 3;//minmum number of markers to be matched
	int planeNum = 1;//plane number
	std::vector<std::vector<MarkerPosition>> markers;//maker position per plane
	double pin[3] = {0.0, 0.0, 0.0};//tip position
	double calbError = -1;//calibration error
	double maxFRE = 1;//maximum matching FRE
	double maxAngle = 60;//maximum angle for matching
};

class ARMDCOMBINED_API ARMDCombinedAPI
{
public:
	ARMDCombinedAPI();
	virtual ~ARMDCombinedAPI();

	/**
	* @brief: connect RT device
	* @param1: hostnameor/IP address of the device
	* @param2: auto or fixed exposure
	* @param3: show or hide the UDP error
	* @return: connection result
	*/
	int connect(std::string hostname, bool autoExposure = true, bool errorPrint = false);

	/**
	* @brief: disconnect RT device
	*/
	void disconnect();

	/**
	* @brief: get local and device IP addresses
	* @return: local and device IP addresses
	*/
	std::vector<std::string> getConnectionIPs();

	/**
	* @brief: print API and firmware version
	*/
	void printAPIandFirmwareVersion();

	/**
	* @brief: set tracking data transmission type
	* @param: transmission type: 1-only passive markers(default), 2-only active markers, 3-mixed transamission, 4-alternate transmission
	*/
	void setTrackingDataTransmissionType(int type);

	/**
	* @brief: get device type
	* @return: device type index
	*/
	uint16_t getDeviceType();

	/**
	* @brief: load ARMD passive tool definition
	* @param1: path of files
	* @param2: way of load ("cover"/"add"), "cover" is default
	*/
	void loadPassiveToolAROM(std::string path, std::string parm = "cover");

	/**
	* @brief: load ARMD passive tool definition
	* @param1: vector of files
	* @param2: way of load ("cover"/"add"), "cover" is default
	*/
	void loadPassiveToolAROM(std::vector<std::string> paths, std::string parm = "cover");

	/**
	* @brief: load ARMD wireless tool definition
	* @param1: path of files
	* @param2: way of load ("cover"/"add"), "cover" is default
	*/
	void loadActiveWirelessToolAROM(std::string path, std::string parm = "cover");

	/**
	* @brief: load ARMD wireless tool definition
	* @param1: vector of files
	* @param2: way of load ("cover"/"add"), "cover" is default
	*/
	void loadActiveWirelessToolAROM(std::vector<std::string> paths, std::string parm = "cover");

	/**
	* @brief: load ARMD active tool definition
	* @param1: path of files
	* @param2: way of load ("cover"/"add"), "cover" is default
	*/
	void loadActiveToolAROM(std::string path, std::string parm = "cover");

	/**
	* @brief: load ARMD active tool definition
	* @param1: vector of files
	* @param2: way of load ("cover"/"add"), "cover" is default
	*/
	void loadActiveToolAROM(std::vector<std::string> paths, std::string parm = "cover");

	/**
	* @brief: load non-ARMD tool definition (will be forbidden in future)
	* @param1: path of files
	* @param2: way of load ("cover"/"add"), "cover" is default
	* @warning: this function is risky and not official file parsing of the corresponding type
	*/
	void loadPassiveToolNAROM(std::string path, std::string parm = "cover");

	/**
	* @brief: load non-ARMD tool definition (will be forbidden in future)
	* @param1: vector of files
	* @param2: way of load ("cover"/"add"), "cover" is default
	* @warning: this function is risky and not official file parsing of the corresponding type
	*/
	void loadPassiveToolNAROM(std::vector<std::string> paths, std::string parm = "cover");

	/**
	* @brief: draw area on the images from the cameras
	* @param1: width (pixels) of area
	* @param2: height (pixels)of area
	* @param3: start position of area in the x-direction of the image from left camera
	* @param4: start position of area in the y-direction of the image from left camera
	* @param5: start position of area in the x-direction of the image from right camera
	* @param6: start position of area in the y-direction of the image from right camera
	*/
	void setAreaDisplay(short width, short height, short leftX, short leftY, short rightX, short rightY);

	/**
	* @brief: hide the marked area on the images from the cameras
	*/
	void setAreaHidden();

	/**
	* @brief: get the size of images from the cameras
	* @return: vector of image width and height
	*/
	std::vector<int> getImageSize();

	/**
	* @brief: get tool information
	* @return: calibration information of all tools
	*/
	std::vector<ToolCalibrationData> getToolStorage() { return this->trackTools; };

	/**
	* @brief: get number of tools
	* @return: number of tools
	*/
	int getTrackToolsNum() { return this->trackTools.size(); };

	/**
	* @brief: start tracking
	*/
	void startTracking();

	/**
	* @brief: stop tracking
	*/
	void stopTracking();

	/**
	* @brief: start monitor
	*/
	void startVideoMonitor();

	/**
	* @brief: stop monitor
	*/
	void stopVideoMonitor();

	/**
	* @brief: start imaging
	*/
	void startImaging();

	/**
	* @brief: stop imaging
	*/
	void stopImaging();

	/**
	* @brief: start reconstruction
	* @param1: width (pixels) of reconstruction area
	* @param2: height (pixels)of reconstruction area
	* @param3: start position of reconstruction area in the x-direction of the image from left camera
	* @param4: start position of reconstruction area in the y-direction of the image from left camera
	* @param5: start position of reconstruction area in the x-direction of the image from right camera  
	* @param6: start position of reconstruction area in the y-direction of the image from right camera  
	* @warning: this function only works for the device of RT-PRO
	*/
	void reconstructPointCloud(short width, short height, short leftX, short leftY, short rightX, short rightY);

	/**
	* @brief: update tracking and imaging data
	*/
	void trackingUpdate();

	/**
	* @brief: update video monitor data
	*/
	void monitoringUpdate();

	/**
	* @brief: get all markers with the type
	* @param: marker type: 0-all(default), 1-passive, 2-active
	* @return: positions of all markers with the type
	*/
	std::vector<MarkerPosition> getAllMarkers(int type = 0);

	/**
	* @brief: get current tracking data
	* @param: all detected markers
	* @return: tracking data of tool
	*/
	std::vector<ToolTrackingData> getTrackingData(std::vector<MarkerPosition> cordi);

	/**
	* @brief: get stray markers
	* @return: positions of all stray markers
	*/
	std::vector<MarkerPosition> getUnMatchedMarkers() { return this->unMatchedMarkers; };

	/**
	* @brief: get current imaging data
	* @return: image from left camera
	*/
	char* getLeftImagingData();

	/**
	* @brief: get current imaging data
	* @return: image from right camera
	*/
	char* getRightImagingData();

	/**
	* @brief: get current video data
	* @return: image from video monitor
	*/
	char* getVideoMonitorData();

	/**
	* @brief: get reconstruction data
	* @return: reconstruction data
	* @warning: this function only works for the device of RT-PRO
	*/
	std::vector<std::vector<char>> getReconstructionData();

	/**
	* @brief: get system time
	* @return: system time
	*/
	std::string getSystemTime();

	/**
	* @brief: get parameters for reconstruction
	* @return: parameters
	* @warning: this function only works for the device of RT-PRO
	*/
	std::vector<double> getReconstructParameters() { return this->RCparameters; };

	/**
	* @brief: get gravity vector
	* @return: gravity vector
	*/
	std::vector<double> getGravityVector() { return this->gravity; };

	/**
	* @brief: get system status
	* @return: connection status
	*/
	uint16_t getConnectionStatus() { return this->connectionStatus; };

	/**
	* @brief: get system status
	* @return: transmission status
	*/
	uint16_t getTransmissionStatus() { return this->transmissionStatus; };

	/**
	 * @brief: get reconstruction status
	 * @return: reconstruction status
	 */
	uint16_t getReconstructionStatus() { return this->reconstructionStatus; };

	/**
	* @brief: get system status
	* @return: system alert
	*/
	uint16_t getSystemAlert() { return this->systemAlert; };

	/**
	* @brief: create an arom file
	* @param1: file directory to be saved
	* @param2: tool calibration struct
	* @return: generation status
	*/
	uint16_t generateAROM(std::string directory, ToolCalibrationData tool);

	/**
	* @brief: convert a non-arom calibration file to an standard arom file
	* @param1: file directory to be saved
	* @param2: path of non-arom calibration file
	* @return: generation status
	*/
	uint16_t convert2AROM(std::string directoryOut, std::string directoryIn);

	/**
	* @brief: pivot calibration for a tool
	* @param1: rotation matrix (3n*3) during pivot calibration
	* @param2: translation vection (3n*1) during pivot calibration
	* @param3: (output) calibrated tip (3*1)
	* @param4: (output) calibration error
	* @return: pivot calibration alert
	*/
	uint16_t pivotTipCalibration(std::vector<std::vector<float>> rot, std::vector<std::vector<float>> tran, std::vector<float>& tip, float& error);
	
	/**
	* @brief: pivot calibration for a tool
	* @param1: tool calibration data
	* @param2: tracking data during pivot calibration
	* @param3: (output) calibrated tip
	* @param4: (output) tip offset after calibration
	* @param5: (output) calibration error
	* @param6: whether update the original tip position (default is false)
	* @return: pivot calibration alert
	*/
	uint16_t pivotTipCalibration(ToolCalibrationData& tool, std::vector<ToolTrackingData> trackingdataVect, std::vector<float>& tip, std::vector<float>& offset, float& error, bool update = false);
	
	/**
	* @brief: project trakcing data of a tool (source) to an another (target)
	* @param1: tracking data of the target tool
	* @param2: tracking data of the source tool
	* @return: tracking data of the projected source tool
	* @warning: if target or source tool is missing, the return is equal to the original tracking data of the source tool
	*/
	ToolTrackingData trackingDataProjection(ToolTrackingData targetTool, ToolTrackingData sourceTool);

	/**
	* @brief: open laer
	* @param: delay time(ms)
	* @warning: this function only works for the devices of RT-PRO and RT-MAXV
	*/
	void laserOn(int delay);

private:
	Tracking* TK;
	Tracking* VM;
	ToolRegi* ToolRegFilter;

	std::vector<int> version;
	std::vector<double> RCparameters;

	std::vector<ToolCalibrationData> trackTools;
	std::vector<std::vector<ToolPlaneMarker>> toolPlaneMarker;

	std::vector<MarkerPosition> allMarkersTrackingData;
	std::vector<MarkerPosition> unMatchedMarkers;
	
	int expLeft;
	int expRight;

	int imageSize;
	char* leftimgDataforFrame;
	char* rightimgDataforFrame;
	char* videoDataforFrame;

	std::vector<std::vector<char>> reconstructDataVector;

	uint16_t transmissionStatus;
	uint16_t transmissionType;
	uint16_t reconstructionStatus;
	uint16_t connectionStatus;
	uint16_t systemAlert;

	std::vector<double> gravity{ 0.0, 0.0, 0.0 };

	//generate tool registration set
	std::vector<ToolPlaneMarker> generateRegiSet(ToolCalibrationData tooldata);

	//search calibration file
	void getFiles(std::string path, std::vector<std::string>& pic_files, std::vector<std::string>& file_names);
	
	//register tool markers to detected markers
	int toolReg(ToolCalibrationData tool, std::vector<ToolPlaneMarker> planeMarkers, std::vector<MarkerPosition> points);

	//arom file generation filter condition
	bool getMarkersMinDis(std::vector<MarkerPosition> markerData);
	bool examineCharacter(std::string name);
};
#endif