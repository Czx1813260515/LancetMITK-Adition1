/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/

#ifndef ABSTRACTROBOT_h
#define ABSTRACTROBOT_h
#include <QObject>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkSmartPointer.h>
#include <QFileDialog>
#include <vector>
#include <qstring.h>
#include <eigen3/Eigen/Dense>
#include <QApplication>
#include <QThread>
#include <MitkLancetHardwareDeviceExports.h>

#define CHECK_ERROR_AND_RETURN(func_call) \
        int error_code = (func_call); \
        std::cout << to_string(error_code) << std::endl; \

class MITKLANCETHARDWAREDEVICE_EXPORT AbstractRobot : public QObject
{
public:
	AbstractRobot(int N = 6)
	{
		m_initJoints.resize(N);
	}
	virtual void Connect() = 0;
	virtual void Disconnect() = 0;
	virtual void PowerOn() = 0;
	virtual void PowerOff() = 0;
	virtual void PowerOff() = 0;	
	virtual void Stop() = 0;
	virtual void Reset() = 0;
	virtual void SetFreeDrag() = 0;
	virtual void StopFreeDrag() = 0;


	void SetRobotIpAddress(const char* aIP)
	{
		m_IpAddress = aIP;
	}
	virtual void Translate(double x, double y, double z) = 0;
	virtual void Translate(double* aDirection, double aLength) = 0;
	virtual void Rotate(double* aDirection, double aAngle) = 0;

	virtual void RecordInitialPos() = 0;
	virtual void GoToInitialPos() = 0;
	virtual void SetTCPToFlange() = 0;
	virtual void SetTCP(vtkMatrix4x4* aMatrix, std::string TCP_NAME = "") = 0;
	

	virtual std::vector<double> GetJointAngles() = 0;
	virtual void SetJointAngles(std::vector<double> aJointAngles) = 0;

	virtual vtkSmartPointer<vtkMatrix4x4> GetBaseToTCP() = 0;
	virtual vtkSmartPointer<vtkMatrix4x4> GetFlangeToTCP() = 0;
	virtual vtkSmartPointer<vtkMatrix4x4> GetBaseToFlange() = 0;
	virtual std::vector<double> CalculateInverse(Eigen::Vector3d aTranslation, Eigen::Vector3d aEulerAngle) = 0;

	virtual void RobotTransformInBase(double* aMatrix) = 0;
	virtual void RobotTransformInTCP(double* aMatrix) = 0;

	virtual std::vector<double> GetCartDampParams() = 0;
	virtual bool SetCartDampParams(std::vector<double> aDampParams) = 0;
	virtual std::vector<double> GetCartStiffParams() = 0;
	virtual bool SetCartStiffParams(std::vector<double> aStiffParams) = 0;

	virtual bool SetVelocity(double aVelocity) = 0;
	virtual bool SetAcceleration(double aVelocity) = 0;

	virtual bool SetRadius(double aVelocity) = 0;

	virtual std::vector<std::vector<double>> GetJointAngleLimits() = 0;

	virtual void WaitMove() = 0;

	virtual void Sleep(int msec) = 0;

	virtual inline std::string to_string(int error_code) = 0;

	virtual std::vector<double> CalculateForward(std::vector<double> aJointAngles) = 0;

protected:
	const char* m_IpAddress;
	std::vector<double> m_initJoints;

};

#endif
