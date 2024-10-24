#pragma once
#ifndef LANCETDIANAROBOT_h
#define LANCETDIANAROBOT_h

#include <AbstractRobot.h>
#include <PrintDataHelper.h>
#include "DianaAPI.h"
#include "DianaAPIDef.h"
#include "Environment.h"
#include "FunctionOptDef.h"
#include "MitkLancetHardwareDeviceExports.h"
class MITKLANCETHARDWAREDEVICE_EXPORT LancetDianaRobot : public AbstractRobot
{
public:
	LancetDianaRobot();
	void Connect() override;

	void Disconnect() override;

	void PowerOn() override;
	 
	void PowerOff() override;

	void Stop()override;

	void Reset() override;

	void Translate(double x, double y, double z) override;

	void Translate(double* aDirection, double aLength) override;

	void Rotate(double* aDirection, double aAngle) override;

	void RecordInitialPos() override;

	void GoToInitialPos() override;

	void SetTCPToFlange() override;

	void SetTCP(vtkMatrix4x4* aMatrix, std::string TCP_NAME = "") override;

	std::vector<std::vector<double>> GetJointAngleLimits() override;
	std::vector<double> GetJointAngles() override;

	void SetJointAngles(std::vector<double> aJointAngles) override;

	vtkSmartPointer<vtkMatrix4x4> GetBaseToTCP() override;
	vtkSmartPointer<vtkMatrix4x4> GetFlangeToTCP() override;
	vtkSmartPointer<vtkMatrix4x4> GetBaseToFlange() override;

	void RobotTransformInBase(double* aMatrix) override;

	void RobotTransformInTCP(double* aMatrix) override;

	std::vector<double> GetCartDampParams() override;
	bool SetCartDampParams(std::vector<double> aDampParams) override;

	std::vector<double> GetCartStiffParams() override;
	bool SetCartStiffParams(std::vector<double> aStiffParams) override;

	std::vector<double> GetCartImpeda();
	bool SetCartImpeda(std::vector<double> aImpeda);

	void SetFreeDrag() override;

	void StopFreeDrag() override;

	std::vector<double> CalculateInverse(Eigen::Vector3d aTranslation, Eigen::Vector3d aEulerAngle) override;

	std::vector<double> CalculateForward(std::vector<double> aJointAngles) override;

	bool SetVelocity(double aVelocity) override;

	bool SetAcceleration(double aVelocity) override;

	bool SetRadius(double aVelocity) override;

	void WaitMove() override;

	bool SetPositionMode();

	bool SetJointImpendanceMode();

	bool SetCartImpendanceMode();

	void Sleep(int msec) override;

	inline std::string to_string(int error_code) override;
private:

	int m_JointsNum;
	// 定义运动速度
	double dVelocity = 30;
	// 定义运动加速度
	double dAcc = 50;
};
#endif
