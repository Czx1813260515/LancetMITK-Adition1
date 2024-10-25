#include "DianaRobot.h"

DianaRobot::DianaRobot()
{
	this->SetRobotIpAddress("192.168.10.75");
}

void DianaRobot::Connect()
{
	srv_net_st* pinfo = new srv_net_st();
	memset(pinfo->SrvIp, 0x00, sizeof(pinfo->SrvIp));
	memcpy(pinfo->SrvIp, m_IpAddress, strlen(m_IpAddress));
	pinfo->LocHeartbeatPort = 0;
	pinfo->LocRobotStatePort = 0;
	pinfo->LocSrvPort = 0;
	int ret = initSrv(nullptr, nullptr, pinfo);
	if (ret < 0)
	{
		std::cout << "Couldn't connect Robot, please ping "<<m_IpAddress<< " first" << std::endl;
	}
	if (pinfo)
	{
		delete pinfo;
		pinfo = nullptr;
	}
	int jointCount = getJointCount();
	m_initJoints.resize(jointCount);
}

void DianaRobot::Disconnect()
{
	int ret = destroySrv(m_IpAddress);
	if (ret < 0)
	{
		printf("destroySrv failed! Return value = %d\n", ret);
	}
}

void DianaRobot::PowerOn()
{
	releaseBrake();
}

void DianaRobot::PowerOff()
{
	holdBrake();
}

void DianaRobot::Translate(double x, double y, double z)
{
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkMatrix4x4> vtkMatrix = vtkSmartPointer<vtkMatrix4x4>::New();

	vtkMatrix->DeepCopy(this->GetBaseToTCP());

	transform->SetMatrix(vtkMatrix);
	transform->Translate(x, y, z);

	vtkSmartPointer<vtkMatrix4x4> setMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	transform->Update();
	transform->GetMatrix(setMatrix);

	this->RobotTransformInBase(setMatrix->GetData());
}

void DianaRobot::Translate(double* aDirection, double aLength)
{
	Translate(aDirection[0] * aLength, aDirection[1] * aLength, aDirection[2] * aLength);
}

void DianaRobot::Rotate(double* aDirection, double aAngles)
{
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	vtkSmartPointer<vtkMatrix4x4> vtkMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix->DeepCopy(this->GetBaseToTCP());

	PrintDataHelper::CoutMatrix("Current Base2TCP Matrix:", vtkMatrix);

	vtkMatrix->Transpose();
	transform->SetMatrix(vtkMatrix);
	transform->RotateWXYZ(aAngles, aDirection);

	vtkSmartPointer<vtkMatrix4x4> setMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	transform->Update();
	transform->GetMatrix(setMatrix);

	this->RobotTransformInBase(setMatrix->GetData());
}

void DianaRobot::RecordInitialPos()
{
	m_initJoints = this->GetJointAngles();
}

void DianaRobot::GoToInitialPos()
{
	int count = getJointCount();
	double* joints_final = new double[count];
	for (int i = 0; i < count; ++i)
	{
		joints_final[i] = m_initJoints[i];
	}
	moveJToTarget(joints_final, 0.2, 0.4);
	WaitMove();
	delete[] joints_final;
}

void DianaRobot::SetTCPToFlange()
{
	double pose[6] = { 0,0,0,0,0,0 };
	setDefaultActiveTcpPose(pose, m_IpAddress);

	getTcpPos(pose, m_IpAddress);
}

bool DianaRobot::SetTCP(vtkMatrix4x4* aMatrix)
{
	double pose[6] = {};
	getTcpPos(pose, m_IpAddress);

	vtkSmartPointer<vtkMatrix4x4> TFlange2TCPNew = vtkSmartPointer<vtkMatrix4x4>::New();
	TFlange2TCPNew->DeepCopy(aMatrix);

	TFlange2TCPNew->SetElement(0, 3, TFlange2TCPNew->GetElement(0, 3) / 1000);
	TFlange2TCPNew->SetElement(1, 3, TFlange2TCPNew->GetElement(1, 3) / 1000);
	TFlange2TCPNew->SetElement(2, 3, TFlange2TCPNew->GetElement(2, 3) / 1000);
	TFlange2TCPNew->Transpose();

	homogeneous2Pose(TFlange2TCPNew->GetData(), pose);

	int ret = setDefaultActiveTcpPose(pose);
	std::string str = ret < 0 ? "Set TCP Failed" : "Set TCP Success";
	std::cout << str << std::endl;

	getTcpPos(pose);
	return true;
}

std::vector<std::vector<double>> DianaRobot::GetJointAngleLimits()
{
	double dblMinPos[7] = { 0 }, dblMaxPos[7] = { 0 };
	int ret = getJointsPositionRange(dblMinPos, dblMaxPos);
	std::vector<std::vector<double>> range;
	std::vector<double> minRange;
	std::vector<double> maxRange;
	for (int i = 0; i < 7; ++i)
	{
		minRange.push_back(dblMinPos[i]);
		maxRange.push_back(dblMaxPos[i]);
	}
	range.push_back(minRange);
	range.push_back(maxRange);
	return range;
}

void DianaRobot::WaitMove()
{
	QThread::msleep(20);
	while (true)
	{
		const char state = getRobotState(m_IpAddress);
		if (state != 0)
		{
			break;
		}
		else
		{
			QThread::msleep(1);
			QApplication::processEvents();
		}
	}
	stop();
}

void DianaRobot::stopRobot()
{
	stop();
}

bool DianaRobot::CleanRobotErrorInfo()
{
	int ret = cleanErrorInfo();
	return ret < 0 ? false : true;
}

bool DianaRobot::SetPositionMode()
{
	int ret = changeControlMode(T_MODE_POSITION);
	return ret < 0 ? false : true;
}

bool DianaRobot::SetJointImpendanceMode()
{
	int ret = changeControlMode(T_MODE_JOINT_IMPEDANCE);
	return ret < 0 ? false : true;
}

bool DianaRobot::SetCartImpendanceMode()
{
	int ret = changeControlMode(T_MODE_CART_IMPEDANCE);
	return ret < 0 ? false : true;
}

bool DianaRobot::SetRobotImpeda(std::vector<double> aImpeda)
{
	double arrstiff[6] = {};
	double dblDamp = 0;
	for (int i = 0; i < 6; ++i)
	{
		arrstiff[i] = aImpeda[i];
	}
	dblDamp = aImpeda[6];
	int ret = setCartImpeda(arrstiff, dblDamp);

	return ret < 0 ? false : true;
}

std::vector<double> DianaRobot::GetRobotImpeda()
{
	std::vector<double> ret;
	double arrStiff[6] = {};
	double dblDamp = 0;

	if (getCartImpeda(arrStiff, &dblDamp) < 0)
	{
		std::cout << "Get CartImpeda Failed" << std::endl;
	}

	for (int i = 0; i < 6; i++)
	{
		ret.push_back(arrStiff[i]);
	}
	ret.push_back(dblDamp);
	return ret;
}

std::vector<double> DianaRobot::GetJointAngles()
{
	double* angles = new double[m_initJoints.size()];
	std::vector<double> angleVec;
	getJointPos(angles);
	for (int i = 0; i < 7; ++i)
	{
		angleVec.push_back(angles[i]);
	}

	delete[] angles;
	return angleVec;
}

void DianaRobot::SetJointAngles(std::vector<double> aJointAngles)
{
	if (aJointAngles.size() != m_initJoints.size())
		return;
	auto range = this->GetJointAngleLimits();
	double* angles = new double[m_initJoints.size()];
	for (int i = 0; i < range[0].size(); ++i)
	{
		if (aJointAngles[i] < range[0][i] || aJointAngles[i] > range[1][i])
		{
			std::cout << "Joint Angle is beyond the limit" << std::endl;
			angles[i] = aJointAngles[i];
			return;
		}
	}

	moveJToTarget(angles, 0.2, 0.4);
	WaitMove();
	delete[] angles;
}

bool DianaRobot::SetJointAngles(double* angles)
{
	auto range = this->GetJointsPositionRange();
	for (int i = 0; i < range[0].size(); ++i)
	{
		if (angles[i] < range[0][i] || angles[i] > range[1][i])
		{
			std::cout << "Joint Angle is beyond the limit" << std::endl;
			return false;
		}
	}
	moveJToTarget(angles, 0.2, 0.4);
	return true;
}

std::vector<std::vector<double>> DianaRobot::GetJointsPositionRange()
{
	double dblMinPos[7] = { 0 }, dblMaxPos[7] = { 0 };
	int ret = getJointsPositionRange(dblMinPos, dblMaxPos);
	std::vector<std::vector<double>> range;
	std::vector<double> minRange;
	std::vector<double> maxRange;
	for (int i = 0; i < 7; ++i)
	{
		minRange.push_back(dblMinPos[i]);
		maxRange.push_back(dblMaxPos[i]);
	}
	range.push_back(minRange);
	range.push_back(maxRange);
	return range;
}

vtkSmartPointer<vtkMatrix4x4> DianaRobot::GetBaseToTCP()
{
	double pose[6] = {};
	getTcpPos(pose, m_IpAddress);
	printf(" forward succeed! Pose: %f, %f, %f, %f, %f, %f\n ", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
	double matrixArray[16] = {};
	pose2Homogeneous(pose, matrixArray);//轴角转齐次变换矩阵  获得baseToTCP
	vtkSmartPointer<vtkMatrix4x4> TBase2Tcp = vtkSmartPointer<vtkMatrix4x4>::New();
	TBase2Tcp->DeepCopy(matrixArray);
	TBase2Tcp->Transpose();
	TBase2Tcp->SetElement(0, 3, TBase2Tcp->GetElement(0, 3) * 1000);
	TBase2Tcp->SetElement(1, 3, TBase2Tcp->GetElement(1, 3) * 1000);
	TBase2Tcp->SetElement(2, 3, TBase2Tcp->GetElement(2, 3) * 1000);

	return TBase2Tcp;
}

vtkSmartPointer<vtkMatrix4x4> DianaRobot::GetFlangeToTCP()
{
	vtkSmartPointer<vtkMatrix4x4> TEnd2Base = vtkSmartPointer<vtkMatrix4x4>::New();
	TEnd2Base->DeepCopy(this->GetBaseToFlange());
	TEnd2Base->Invert();
	vtkSmartPointer<vtkMatrix4x4> TBase2TCP = vtkSmartPointer<vtkMatrix4x4>::New();
	TBase2TCP->DeepCopy(this->GetBaseToTCP());

	vtkSmartPointer<vtkMatrix4x4> TEnd2TCP = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->PreMultiply();
	transform->SetMatrix(TEnd2Base);
	transform->Concatenate(TBase2TCP);
	transform->Update();
	transform->GetMatrix(TEnd2TCP);

	return TEnd2TCP;
}

vtkSmartPointer<vtkMatrix4x4> DianaRobot::GetBaseToFlange()
{
	double joints[7] = { 0.0 };
	int ret = getJointPos(joints, m_IpAddress);
	double pose[6] = {};
	forward(joints, pose);
	double matrix[16] = {};
	pose2Homogeneous(pose, matrix);
	vtkSmartPointer<vtkMatrix4x4> base2End = vtkSmartPointer<vtkMatrix4x4>::New();
	base2End->DeepCopy(matrix);
	base2End->Transpose();
	base2End->SetElement(0, 3, base2End->GetElement(0, 3) * 1000);
	base2End->SetElement(1, 3, base2End->GetElement(1, 3) * 1000);
	base2End->SetElement(2, 3, base2End->GetElement(2, 3) * 1000);
	return base2End;
}

void DianaRobot::RobotTransformInBase(double* aMatrix)
{
	double pose[6] = {};
	vtkSmartPointer<vtkMatrix4x4> vtkMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkMatrix->DeepCopy(aMatrix);
	vtkMatrix->SetElement(0, 3, vtkMatrix->GetElement(0, 3) / 1000);
	vtkMatrix->SetElement(1, 3, vtkMatrix->GetElement(1, 3) / 1000);
	vtkMatrix->SetElement(2, 3, vtkMatrix->GetElement(2, 3) / 1000);
	vtkMatrix->Transpose();
	homogeneous2Pose(vtkMatrix->GetData(), pose);
	double joints_final[7]{};
	inverse(pose, joints_final, nullptr);
	moveJToTarget(joints_final, 0.2, 0.4);
	WaitMove();
}

void DianaRobot::RobotTransformInTCP(double* aMatrix)
{
	double pose[6] = {};
	vtkSmartPointer<vtkMatrix4x4> TBase2Tcp = vtkSmartPointer<vtkMatrix4x4>::New();
	TBase2Tcp->DeepCopy(this->GetBaseToTCP());
	PrintDataHelper::CoutMatrix("Current Base2TCP Matrix:", TBase2Tcp);
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->PreMultiply();
	transform->SetMatrix(TBase2Tcp);
	vtkSmartPointer<vtkMatrix4x4> vtkMatrix = vtkSmartPointer<vtkMatrix4x4>::New();

	vtkMatrix->DeepCopy(aMatrix);
	transform->Concatenate(vtkMatrix);

	vtkSmartPointer<vtkMatrix4x4> newBase2TCP = vtkSmartPointer<vtkMatrix4x4>::New();
	transform->GetMatrix(newBase2TCP);
	this->RobotTransformInBase(newBase2TCP->GetData());
}

std::vector<double> DianaRobot::GetCartDampParams()
{
	double arrStiff[6] = {};
	double dblDamp = 0;
	std::vector<double> ret;
	if (getCartImpeda(arrStiff, &dblDamp) < 0)
	{
		std::cout << "Get CartImpeda Failed" << std::endl;
		return std::vector<double>();
	}
	for (int i = 0; i < 6; ++i)
	{
		ret.push_back(dblDamp);
	}
	return ret;
}

bool DianaRobot::SetCartDampParams(std::vector<double> aDampParams)
{
	auto arrStiffVec = GetCartStiffParams();
	double arrStiff[6] = { 0.0 };
	for (int i = 0; i < arrStiffVec.size(); ++i)
	{
		arrStiff[i] = arrStiffVec[i];
	}
	double currentDamp = aDampParams[0];
	int ret = setCartImpeda(arrStiff, currentDamp);
	return ret < 0 ? false : true;
}

std::vector<double> DianaRobot::GetCartStiffParams()
{
	double arrStiff[6] = {};
	double dblDamp = 0;
	std::vector<double> ret;
	if (getCartImpeda(arrStiff, &dblDamp) < 0)
	{
		std::cout << "Get CartImpeda Failed" << std::endl;
		return std::vector<double>();
	}
	for (int i = 0; i < 6; ++i)
	{
		ret.push_back(arrStiff[i]);
	}
	return ret;
}

bool DianaRobot::SetCartStiffParams(std::vector<double> aStiffParams)
{
	if (aStiffParams.size() != 6)
		return false;

	double params[6] = { 0.0 };
	for (int i = 0; i < aStiffParams.size(); ++i)
	{
		params[i] = aStiffParams[i];
	}
	double damp = this->GetCartDampParams()[0];
	int ret = setCartImpeda(params, damp);

	return ret < 0 ? false : true;
}

std::vector<double> DianaRobot::GetCartImpeda()
{
	std::vector<double> Impeda;
	double arrStiff[6] = {};
	double dblDamp = 0;

	if (getCartImpeda(arrStiff, &dblDamp) < 0)
	{
		std::cout << "Get CartImpeda Failed" << std::endl;
	}

	for (int i = 0; i < 6; i++)
	{
		Impeda.push_back(arrStiff[i]);
	}
	Impeda.push_back(dblDamp);
	return Impeda;
}

bool DianaRobot::SetCartImpeda(std::vector<double> aImpeda)
{
	double arrstiff[6] = {};
	double dblDamp = 0;
	for (int i = 0; i < 6; ++i)
	{
		arrstiff[i] = aImpeda[i];
	}
	dblDamp = aImpeda[6];
	int ret = setCartImpeda(arrstiff, dblDamp);

	return ret < 0 ? false : true;
}

bool DianaRobot::SetVelocity(double aVelocity)
{
	return false;
}
