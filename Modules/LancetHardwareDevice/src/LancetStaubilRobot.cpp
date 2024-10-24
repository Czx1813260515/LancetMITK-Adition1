#include "LancetStaubilRobot.h"

void LancetStaubilRobot::Connect()
{
	m_Robot.connectrobot();
}

void LancetStaubilRobot::Disconnect()
{
	m_Robot.disconnectrobot();
}

void LancetStaubilRobot::PowerOn()
{
	m_Robot.poweron();
}

void LancetStaubilRobot::PowerOff()
{
	m_Robot.poweroff();
}

void LancetStaubilRobot::Translate(double x, double y, double z)
{
}

void LancetStaubilRobot::Translate(double* aDirection, double aLength)
{
}

void LancetStaubilRobot::Rotate(double* aDirection, double aAngle)
{
}

void LancetStaubilRobot::RecordInitialPos()
{
}

void LancetStaubilRobot::GoToInitialPos()
{
}

void LancetStaubilRobot::SetTCPToFlange()
{
}

bool LancetStaubilRobot::SetTCP(vtkMatrix4x4* aMatrix)
{
	return false;
}

std::vector<double> LancetStaubilRobot::GetJointAngles()
{
	m_Robot.requestrealtimedata();
	auto joints = m_Robot.realtime_data.joints;
	
	std::vector<double> ret = { joints.j1, joints.j2,joints.j3,joints.j4,joints.j5,joints.j6 };

	return ret;
}

void LancetStaubilRobot::SetJointAngles(std::vector<double> aJointAngles)
{
	//TODO
}

vtkSmartPointer<vtkMatrix4x4> LancetStaubilRobot::GetBaseToTCP()
{

	return vtkSmartPointer<vtkMatrix4x4>();
}

vtkSmartPointer<vtkMatrix4x4> LancetStaubilRobot::GetFlangeToTCP()
{
	return vtkSmartPointer<vtkMatrix4x4>();
}

vtkSmartPointer<vtkMatrix4x4> LancetStaubilRobot::GetBaseToFlange()
{
	return vtkSmartPointer<vtkMatrix4x4>();
}

void LancetStaubilRobot::RobotTransformInBase(double* aMatrix)
{
	vtkSmartPointer<vtkMatrix4x4> base2target = vtkSmartPointer<vtkMatrix4x4>::New();
	base2target->DeepCopy(aMatrix);

	m_Robot.movej(1, 0, 0, 0, 0, 0);
	QThread::msleep(300);
	auto ret = GetEulerAndTranslationByMatrix(base2target);
	auto euler = ret.first;
	auto trans = ret.second;
	m_Robot.movep(euler[0], euler[1], euler[2], trans[0], trans[1], trans[2]);
}

void LancetStaubilRobot::RobotTransformInTCP(double* aMatrix)
{
}

std::vector<double> LancetStaubilRobot::GetCartDampParams()
{
	return std::vector<double>();
}

bool LancetStaubilRobot::SetCartDampParams(std::vector<double> aDampParams)
{
	return false;
}

std::vector<double> LancetStaubilRobot::GetCartStiffParams()
{
	return std::vector<double>();
}

bool LancetStaubilRobot::SetCartStiffParams(std::vector<double> aStiffParams)
{
	return false;
}

bool LancetStaubilRobot::SetVelocity(double aVelocity)
{
	return false;
}

std::vector<std::vector<double>> LancetStaubilRobot::GetJointAngleLimits()
{
	return std::vector<std::vector<double>>();
}

void LancetStaubilRobot::WaitMove()
{
}

Eigen::Vector3d LancetStaubilRobot::GetXYZEulerByMatrix(vtkMatrix4x4* aMatrix)
{
	auto data = aMatrix->GetData();
	Eigen::Matrix3d res;
	res << data[0], data[1], data[2],
		data[4], data[5], data[6],
		data[8], data[9], data[10];

	Eigen::Vector3d ret = res.eulerAngles(0, 1, 2);
	return ret;
}

Eigen::Vector3d LancetStaubilRobot::GetTranslationByMatrix(vtkMatrix4x4* aMatrix)
{
	auto array = aMatrix->GetData();
	return Eigen::Vector3d(array[3], array[7], array[11]);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> LancetStaubilRobot::GetEulerAndTranslationByMatrix(vtkMatrix4x4* aMatrix)
{
	return std::pair(GetXYZEulerByMatrix(aMatrix), GetTranslationByMatrix(aMatrix));
}

vtkSmartPointer<vtkMatrix4x4> LancetStaubilRobot::GetMatrixByEulerAndTranslation(Eigen::Vector3d euler, Eigen::Vector3d trans)
{
	return vtkSmartPointer<vtkMatrix4x4>();
}
