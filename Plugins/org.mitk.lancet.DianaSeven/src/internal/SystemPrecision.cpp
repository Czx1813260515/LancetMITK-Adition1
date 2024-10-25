#include "SystemPrecision.h"

lancetAlgorithm::SystemPrecision::SystemPrecision(mitk::DataStorage* aDataStorage, DianaRobot* aRobot, AimCamera* aCamera, LancetRobotRegistration* aLancetRobReg, mitk::IRenderWindowPart* aIRenderWindowPart)
{
	m_dataStorage = aDataStorage;
	m_DianaSevenRobot = aRobot;
	m_AimCamera = aCamera;
	m_LancetRobReg = aLancetRobReg;

	m_IRenderWindowPart = aIRenderWindowPart;
	std::cout << "SystemPrecision" << std::endl;
	m_LandmarkRegistrationMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	m_ICPRegistrationMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	m_FinalRegistrationMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	m_Marker2TCPMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	m_ICPPoints = mitk::PointSet::New();
	m_LandmarkPoints = mitk::PointSet::New();
	Reset();

	connect(m_AimCamera, &AimCamera::CameraUpdateClock, this, &SystemPrecision::Update);

	m_TCamera2RobotTCP = vtkSmartPointer<vtkMatrix4x4>::New();
	m_Image2Camera = vtkSmartPointer<vtkMatrix4x4>::New();

}

void lancetAlgorithm::SystemPrecision::DisplayPrecisionTool()
{
	if (!m_IsDisplayPrecisionTool)
	{
		std::string path = std::string(getenv("USERPROFILE"));
		path += "\\Desktop\\PKAModelData\\SystemPrecision\\PrecisionTool.stl";
		LoadSingleMitkFile(m_dataStorage, path, m_PrecisionToolModelStr);
		m_IsDisplayPrecisionTool = !m_IsDisplayPrecisionTool;
	}
}

void lancetAlgorithm::SystemPrecision::DisplayProbe()
{
	if (!m_IsDisplayProbe)
	{
		std::string path = std::string(getenv("USERPROFILE"));
		path += "\\Desktop\\PKAModelData\\PKAProbe.stl";
		LoadSingleMitkFile(m_dataStorage, path, m_ProbeModelStr);
		m_IsDisplayProbe = !m_IsDisplayProbe;
	}
}

void lancetAlgorithm::SystemPrecision::LandmarkRegistration(mitk::PointSet* target)
{
	vtkSmartPointer<vtkPoints> pSource =
		vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> pTarget =
		vtkSmartPointer<vtkPoints>::New();

	for (int i = 0; i < target->GetSize(); i++)
	{
		auto pointVega = m_LandmarkPoints->GetPoint(i);
		pSource->InsertNextPoint(pointVega[0], pointVega[1], pointVega[2]);
		auto pointSTL = target->GetPoint(i);
		pTarget->InsertNextPoint(pointSTL[0], pointSTL[1], pointSTL[2]);
	}
	vtkSmartPointer<vtkLandmarkTransform> landmarkTransform =
		vtkSmartPointer<vtkLandmarkTransform>::New();

	landmarkTransform->SetSourceLandmarks(pSource);
	landmarkTransform->SetTargetLandmarks(pTarget);
	landmarkTransform->SetMode(VTK_LANDMARK_RIGIDBODY);
	landmarkTransform->Update();
	
	m_LandmarkRegistrationMatrix->DeepCopy(landmarkTransform->GetMatrix());
	m_FinalRegistrationMatrix->DeepCopy(m_LandmarkRegistrationMatrix);
	PrintDataHelper::CoutMatrix("m_LandmarkRegistrationMatrix: ", m_LandmarkRegistrationMatrix);
}

void lancetAlgorithm::SystemPrecision::ICPRegistration(mitk::Surface* surface)
{
	auto icpRegistrator = mitk::SurfaceRegistration::New();

	icpRegistrator->SetIcpPoints(m_ICPPoints);
	icpRegistrator->SetSurfaceSrc(surface);
	icpRegistrator->ComputeIcpResult();
	m_ICPRegistrationMatrix->DeepCopy(icpRegistrator->GetResult());

	PrintDataHelper::CoutMatrix("m_ICPRegistrationMatrix: ", m_ICPRegistrationMatrix);
	m_FinalRegistrationMatrix->DeepCopy(PostConcatenateMatrixs(m_LandmarkRegistrationMatrix, m_ICPRegistrationMatrix));  //box2LandMark * landMark2BoxRF
	PrintDataHelper::CoutMatrix("m_FinalRegistrationMatrix: ", m_FinalRegistrationMatrix);
}

void lancetAlgorithm::SystemPrecision::Update()
{
	if (m_IsDisplayPrecisionTool)
		UpdatePrecisionTestKit();

	if (m_IsDisplayProbe)
		UpdateProbe();
}

void lancetAlgorithm::SystemPrecision::UpdatePrecisionTestKit()
{
	vtkSmartPointer<vtkMatrix4x4> TCamera2PrecisionTool = vtkSmartPointer<vtkMatrix4x4>::New();
	TCamera2PrecisionTool->DeepCopy(m_AimCamera->GetToolMatrixByName(m_RobotEndRF));
	//PrintDataHelper::CoutMatrix("TCamera2PrecisionTool", TCamera2PrecisionTool);
	vtkSmartPointer<vtkMatrix4x4> TPrecisionBoxRF2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	TPrecisionBoxRF2Camera->DeepCopy(m_AimCamera->GetToolMatrixByName(m_VerificationBlockStr));
	TPrecisionBoxRF2Camera->Invert();
	//PrintDataHelper::CoutMatrix("TPrecisionBoxRF2Camera", TPrecisionBoxRF2Camera);

	vtkSmartPointer<vtkMatrix4x4> TBox2BoxRF = vtkSmartPointer<vtkMatrix4x4>::New();
	TBox2BoxRF->DeepCopy(m_FinalRegistrationMatrix);
	auto geo = m_dataStorage->GetNamedNode(m_PrecisionToolModelStr)->GetData()->GetGeometry();
	vtkSmartPointer<vtkMatrix4x4> modelMatrix = vtkSmartPointer<vtkMatrix4x4>::New();

	if (m_IsUseImageReg)
	{
		modelMatrix->DeepCopy(PreConcatenateMatrixs(TBox2BoxRF, TPrecisionBoxRF2Camera, TCamera2PrecisionTool));
	}
	else
	{
		modelMatrix->DeepCopy(TCamera2PrecisionTool);
	}
	geo->SetIndexToWorldTransformByVtkMatrix(modelMatrix);
}

void lancetAlgorithm::SystemPrecision::UpdateProbe()
{
	vtkSmartPointer<vtkMatrix4x4> TCamera2Probe = vtkSmartPointer<vtkMatrix4x4>::New();
	TCamera2Probe->DeepCopy(m_AimCamera->GetToolMatrixByName(m_ProbeModelStr));

	vtkSmartPointer<vtkMatrix4x4> TPrecisionBoxRF2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	TPrecisionBoxRF2Camera->DeepCopy(m_AimCamera->GetToolMatrixByName(m_VerificationBlockStr));
	TPrecisionBoxRF2Camera->Invert();

	vtkSmartPointer<vtkMatrix4x4> TBox2BoxRF = vtkSmartPointer<vtkMatrix4x4>::New();
	TBox2BoxRF->DeepCopy(m_FinalRegistrationMatrix);
	//TBox2BoxRF->Invert();
	auto geo = m_dataStorage->GetNamedNode(m_ProbeModelStr)->GetData()->GetGeometry();
	vtkSmartPointer<vtkMatrix4x4> modelMatrix = vtkSmartPointer<vtkMatrix4x4>::New();

	if (m_IsUseImageReg)
	{
		modelMatrix->DeepCopy(PreConcatenateMatrixs(TBox2BoxRF, TPrecisionBoxRF2Camera, TCamera2Probe));
	}
	else
	{
		modelMatrix->DeepCopy(TCamera2Probe);
	}
	geo->SetIndexToWorldTransformByVtkMatrix(modelMatrix);
}

void lancetAlgorithm::SystemPrecision::UpdateTCPActor()
{
	if (!m_TCPActor)
		return;
	vtkSmartPointer<vtkMatrix4x4> TBaseToTCP = vtkSmartPointer<vtkMatrix4x4>::New();
	//TBaseToTCP->DeepCopy(m_DianaSevenRobot->GetBase2TCP());
	TBaseToTCP->DeepCopy(m_DianaSevenRobot->GetBaseToTCP());
	vtkSmartPointer<vtkMatrix4x4> TImage2Base = vtkSmartPointer<vtkMatrix4x4>::New(); 
	TImage2Base->DeepCopy(CalculateBase2Image());
	TImage2Base->Invert();

	vtkSmartPointer<vtkMatrix4x4> TImage2TCP = vtkSmartPointer<vtkMatrix4x4>::New();
	TImage2TCP->DeepCopy(PreConcatenateMatrixs(TImage2Base, TBaseToTCP));

	if (m_IsUseImageReg)
	{
		m_TCPActor->SetUserMatrix(TImage2TCP);
		//PrintDataHelper::CoutMatrix("TImage2TCP", TImage2TCP);
	}
	else
	{
		m_TCPActor->SetUserMatrix(TBaseToTCP);
	}
}

void lancetAlgorithm::SystemPrecision::UpdateTCPInRF()
{
	if (!m_dataStorage->GetNamedNode(m_PrecisionToolModelStr))
		return;
	if (!m_TCPInRFActor)
		return;
	auto geoMatrix = m_dataStorage->GetNamedNode(m_PrecisionToolModelStr)->GetData()->GetGeometry()->GetVtkMatrix();

	vtkSmartPointer<vtkMatrix4x4> TMarkerTCP = vtkSmartPointer<vtkMatrix4x4>::New();

	TMarkerTCP->DeepCopy(PreConcatenateMatrixs(geoMatrix, m_Marker2TCPMatrix));

	m_TCPInRFActor->SetUserMatrix(TMarkerTCP);
}

void lancetAlgorithm::SystemPrecision::UpdateFlange()
{
	if (!m_FlangeActor)
		return;

	auto node = m_dataStorage->GetNamedNode(m_PrecisionToolModelStr);
	if (!node)
		return;
	vtkSmartPointer<vtkMatrix4x4> TEndRF2Flange = vtkSmartPointer<vtkMatrix4x4>::New();
	TEndRF2Flange->DeepCopy(m_LancetRobReg->getFlangeToEndRF());
	TEndRF2Flange->Invert();
	auto geoMatrix = node->GetData()->GetGeometry()->GetVtkMatrix();
	vtkSmartPointer<vtkMatrix4x4> TImage2FlangeTCP = vtkSmartPointer<vtkMatrix4x4>::New();

	TImage2FlangeTCP->DeepCopy(PreConcatenateMatrixs(geoMatrix, TEndRF2Flange));

	m_FlangeActor->SetUserMatrix(TImage2FlangeTCP);
}


std::pair<Eigen::Vector3d, Eigen::Vector3d> lancetAlgorithm::SystemPrecision::AveragePointInBase(Eigen::Vector3d aPointA, Eigen::Vector3d aPointB)
{
	vtkSmartPointer<vtkMatrix4x4> TBase2Box = vtkSmartPointer<vtkMatrix4x4>::New();
	
	Eigen::Vector3d retA(0, 0, 0);
	Eigen::Vector3d retB(0, 0, 0);
	for (int i = 0; i < 10; ++i)
	{
		TBase2Box->DeepCopy(CalculateBase2Image());
		retA += TransformByMatrix(aPointA, TBase2Box);
		retB += TransformByMatrix(aPointB, TBase2Box);
		QThread::msleep(50);
		QApplication::processEvents();
	}
	retA = retA / 10;
	retB = retB / 10;
	return std::pair(retA, retB);
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> lancetAlgorithm::SystemPrecision::AveragePointInBase(Eigen::Vector3d aPointA, Eigen::Vector3d aPointB, Eigen::Vector3d aPointC)
{
	vtkSmartPointer<vtkMatrix4x4> TBase2Box = vtkSmartPointer<vtkMatrix4x4>::New();

	Eigen::Vector3d retA(0, 0, 0);
	Eigen::Vector3d retB(0, 0, 0);
	Eigen::Vector3d retC(0, 0, 0);
	for (int i = 0; i < 10; ++i)
	{
		TBase2Box->DeepCopy(CalculateBase2Image());
		retA += TransformByMatrix(aPointA, TBase2Box);
		retB += TransformByMatrix(aPointB, TBase2Box);
		retC += TransformByMatrix(aPointC, TBase2Box);
		QThread::msleep(50);
		QApplication::processEvents();
	}
	retA = retA / 10;
	retB = retB / 10;
	retC = retC / 10;
	return std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>(retA, retB, retC);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> lancetAlgorithm::SystemPrecision::AveragePointInTCP(Eigen::Vector3d aPointA, Eigen::Vector3d aPointB)
{
	vtkSmartPointer<vtkMatrix4x4> TBase2Box = vtkSmartPointer<vtkMatrix4x4>::New();

	Eigen::Vector3d retA(0, 0, 0);
	Eigen::Vector3d retB(0, 0, 0);
	for (int i = 0; i < 10; ++i)
	{
		TBase2Box->DeepCopy(CalculateTCP2Image());
		retA += TransformByMatrix(aPointA, TBase2Box);
		retB += TransformByMatrix(aPointB, TBase2Box);
		QThread::msleep(50);
		QApplication::processEvents();
	}
	retA = retA / 10;
	retB = retB / 10;
	return std::pair(retA, retB);
}

vtkSmartPointer<vtkMatrix4x4> lancetAlgorithm::SystemPrecision::CalculateLineTargetInBase(Eigen::Vector3d point0, Eigen::Vector3d point1)
{
	PrintDataHelper::CoutArray(point0, "GoLine Point0");
	PrintDataHelper::CoutArray(point1, "GoLine Point1");
	auto points = AveragePointInBase(point0, point1);
	PrintDataHelper::CoutArray(points.first, "GoLine point0 in Base");
	PrintDataHelper::CoutArray(points.second, "GoLine point1 in Base");
	Eigen::Vector3d targetXUnderBase = points.first - points.second;
	targetXUnderBase.normalize();

	vtkSmartPointer<vtkMatrix4x4> TBase2Flange = vtkSmartPointer<vtkMatrix4x4>::New();
	//TBase2Flange->DeepCopy(m_DianaAimHardwareService->GetRobotBase2RobotEnd());
	TBase2Flange->DeepCopy(m_DianaSevenRobot->GetBaseToTCP());
	PrintDataHelper::CoutMatrix("TBase2TCP", m_DianaSevenRobot->GetBaseToTCP());
	PrintDataHelper::CoutMatrix("TBase2Flange", TBase2Flange);
	auto TBase2FlangeArray = TBase2Flange->GetData();
	Eigen::Vector3d currentZUnderBase(TBase2FlangeArray[0], TBase2FlangeArray[4], TBase2FlangeArray[8]);//3711  159 048
	currentZUnderBase.normalize();

	Eigen::Vector3d  rotationAxis;
	rotationAxis = currentZUnderBase.cross(targetXUnderBase);

	double rotationAngle;//定义旋转角
	if (currentZUnderBase.dot(targetXUnderBase) > 0) //如果向量的内积大于0，cos大于0（锐角）
	{
		rotationAngle = 180 * asin(rotationAxis.norm()) / 3.1415926;//求向量的模长（sin[rotationAngle]）,再取反三角
	}
	else //如果向量的内积小于0，cos小于0（钝角）
	{
		rotationAngle = 180 - 180 * asin(rotationAxis.norm()) / 3.1415926;
	}

	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

	transform->PostMultiply();
	transform->Identity();
	transform->SetMatrix(TBase2Flange);
	// T_TCPtoBase
	transform->RotateWXYZ(rotationAngle, rotationAxis[0], rotationAxis[1], rotationAxis[2]);//旋转角度，和旋转向量
	transform->Update();

	vtkSmartPointer<vtkMatrix4x4> pointMatrixInBase = vtkSmartPointer<vtkMatrix4x4>::New();
	pointMatrixInBase = transform->GetMatrix();
	pointMatrixInBase->SetElement(0, 3, points.second.x());
	pointMatrixInBase->SetElement(1, 3, points.second.y());
	pointMatrixInBase->SetElement(2, 3, points.second.z());
	PrintDataHelper::CoutMatrix("pointMatrixInBase", pointMatrixInBase);
	DisplayTargetActor(pointMatrixInBase);
	return pointMatrixInBase;
}

/// <summary>
///
/// </summary>
/// <param name="aPoints"></param>
/// <param name="aMatrix"></param>
void lancetAlgorithm::SystemPrecision::TransformPoints(mitk::PointSet* aPoints, vtkMatrix4x4* aMatrix)
{
	// 检查输入的 PointSet 和 变换矩阵是否为空
	if (!aPoints || !aMatrix)
	{
		std::cerr << "Invalid PointSet or Transform Matrix!" << std::endl;
		return;
	}

	// 遍历 PointSet 中的所有点
	for (unsigned int i = 0; i < aPoints->GetSize(); ++i)
	{
		// 获取原始点
		mitk::Point3D originalPoint = aPoints->GetPoint(i);

		// 定义一个 VTK 的坐标点
		double vtkPoint[4] = { originalPoint[0], originalPoint[1], originalPoint[2], 1.0 };

		// 变换坐标点
		double transformedVtkPoint[4];
		aMatrix->MultiplyPoint(vtkPoint, transformedVtkPoint);

		// 将变换后的坐标应用到 MITK 的 Point3D
		mitk::Point3D transformedPoint;
		transformedPoint[0] = transformedVtkPoint[0];
		transformedPoint[1] = transformedVtkPoint[1];
		transformedPoint[2] = transformedVtkPoint[2];

		// 更新 PointSet 中的点
		aPoints->SetPoint(i, transformedPoint);
	}
}


vtkSmartPointer<vtkMatrix4x4> lancetAlgorithm::SystemPrecision::CalculateBase2Image()
{
	vtkSmartPointer<vtkMatrix4x4> TBaseRF2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	TBaseRF2Camera->DeepCopy(m_AimCamera->GetToolMatrixByName(m_RobotBaseRF));

	TBaseRF2Camera->Invert();

	vtkSmartPointer<vtkMatrix4x4> TBase2BaseRF = vtkSmartPointer<vtkMatrix4x4>::New();
	TBase2BaseRF->DeepCopy(m_LancetRobReg->getBaseToBaseRF());
	TBase2BaseRF->Invert();

	vtkSmartPointer<vtkMatrix4x4> TCamera2BoxRF = vtkSmartPointer<vtkMatrix4x4>::New();
	TCamera2BoxRF->DeepCopy(m_AimCamera->GetToolMatrixByName(m_VerificationBlockStr));

	vtkSmartPointer<vtkMatrix4x4> TBoxRF2Image = vtkSmartPointer<vtkMatrix4x4>::New();
	TBoxRF2Image->DeepCopy(m_FinalRegistrationMatrix);
	TBoxRF2Image->Invert();
	vtkSmartPointer<vtkMatrix4x4> TBase2Image = vtkSmartPointer<vtkMatrix4x4>::New();

	TBase2Image->DeepCopy(PreConcatenateMatrixs(TBase2BaseRF, TBaseRF2Camera, TCamera2BoxRF, TBoxRF2Image));
	//PrintDataHelper::CoutMatrix("TBase2Image", TBase2Image);

	return TBase2Image;
}

vtkSmartPointer<vtkMatrix4x4> lancetAlgorithm::SystemPrecision::CalculateTCP2Image()
{
	vtkSmartPointer<vtkMatrix4x4> TCP2Marker = vtkSmartPointer<vtkMatrix4x4>::New();
	TCP2Marker->DeepCopy(m_Marker2TCPMatrix);
	TCP2Marker->Invert();

	vtkSmartPointer<vtkMatrix4x4> TMarker2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	TMarker2Camera->DeepCopy(m_AimCamera->GetToolMatrixByName(m_PrecisionToolModelStr));
	TMarker2Camera->Invert();

	vtkSmartPointer<vtkMatrix4x4> TCamera2BoxRF = vtkSmartPointer<vtkMatrix4x4>::New();
	TCamera2BoxRF->DeepCopy(m_AimCamera->GetToolMatrixByName(m_VerificationBlockStr));

	vtkSmartPointer<vtkMatrix4x4> TBoxRF2Box = vtkSmartPointer<vtkMatrix4x4>::New();
	TBoxRF2Box->DeepCopy(m_FinalRegistrationMatrix);
	TBoxRF2Box->Invert();

	vtkSmartPointer<vtkMatrix4x4> TCP2Box = vtkSmartPointer<vtkMatrix4x4>::New();
	TCP2Box->DeepCopy(PreConcatenateMatrixs(TCP2Marker, TMarker2Camera, TCamera2BoxRF, TBoxRF2Box));
	return TCP2Box;
}

vtkSmartPointer<vtkMatrix4x4> lancetAlgorithm::SystemPrecision::CalculateBoxRF2Camera()
{
	vtkSmartPointer<vtkMatrix4x4> ret = vtkSmartPointer<vtkMatrix4x4>::New();

	vtkSmartPointer<vtkMatrix4x4> TBoxRF2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	TBoxRF2Camera->DeepCopy(m_AimCamera->GetToolMatrixByName(m_VerificationBlockStr));
	TBoxRF2Camera->Invert();

	return TBoxRF2Camera;
}

vtkSmartPointer<vtkMatrix4x4> lancetAlgorithm::SystemPrecision::CalculateLandmartBox2Camera()
{
	vtkSmartPointer<vtkMatrix4x4> TBoxRF2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	TBoxRF2Camera->DeepCopy(CalculateBoxRF2Camera());
	vtkSmartPointer<vtkMatrix4x4> boxLandmark2BoxRF = vtkSmartPointer<vtkMatrix4x4>::New();
	boxLandmark2BoxRF->DeepCopy(m_LandmarkRegistrationMatrix);
	vtkSmartPointer<vtkMatrix4x4> ret = vtkSmartPointer<vtkMatrix4x4>::New();

	ret->DeepCopy(PreConcatenateMatrixs(boxLandmark2BoxRF, TBoxRF2Camera));
	return ret;
}

Eigen::Vector3d lancetAlgorithm::SystemPrecision::TransformByMatrix(Eigen::Vector3d in, vtkMatrix4x4* matrix)
{
	Eigen::Vector4d v_homogeneous(in[0], in[1], in[2], 1.0);
	// 将 vtkMatrix4x4 转换为 Eigen::Matrix4d
	Eigen::Matrix4d M_eigen;
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			M_eigen(i, j) = matrix->GetElement(i, j);
		}
	}

	// 执行矩阵乘法
	Eigen::Vector4d result_homogeneous = M_eigen * v_homogeneous;

	// 转换回 3D 向量
	Eigen::Vector3d result(result_homogeneous[0] / result_homogeneous[3],
		result_homogeneous[1] / result_homogeneous[3],
		result_homogeneous[2] / result_homogeneous[3]);

	double vec[4] = { in[0], in[1], in[2], 1.0 };
	vtkSmartPointer<vtkMatrix4x4> mMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	mMatrix->DeepCopy(matrix);
	auto out = mMatrix->MultiplyDoublePoint(vec);
	result = Eigen::Vector3d(out[0], out[1], out[2]);
	return result;
}

vtkSmartPointer<vtkAxesActor> lancetAlgorithm::SystemPrecision::GenerateAxesActor(double axexLength)
{
	vtkSmartPointer<vtkAxesActor> axesActor = vtkSmartPointer<vtkAxesActor>::New();
	//axesActor->SetConeRadius(100);
	//axesActor->SetCylinderRadius(50);
	axesActor->SetTotalLength(axexLength, axexLength, axexLength);
	axesActor->SetAxisLabels(false);
	axesActor->Modified();

	return axesActor;
}

void lancetAlgorithm::SystemPrecision::AddAxesActor(vtkAxesActor* aActor)
{
	QmitkRenderWindow* mitkRenderWindow = m_IRenderWindowPart->GetQmitkRenderWindow("3d");

	vtkRenderWindow* renderWindow = mitkRenderWindow->GetVtkRenderWindow();

	vtkSmartPointer<vtkRenderer> renderer;

	renderer = renderWindow->GetRenderers()->GetFirstRenderer();
	renderer->AddActor(aActor);
}

int lancetAlgorithm::SystemPrecision::CollectLandMarkPoints()
{
	Eigen::Vector3d point = m_AimCamera->GetToolTipByName(m_ProbeModelStr);
	vtkSmartPointer<vtkMatrix4x4> TBoxRF2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	TBoxRF2Camera->DeepCopy(CalculateBoxRF2Camera());
	int count = m_LandmarkPoints->GetSize();
	mitk::PointSet::PointType pt;

	Eigen::Vector3d pointInBoxRF = TransformByMatrix(point, TBoxRF2Camera);
	PrintDataHelper::CoutArray(pointInBoxRF, "pointInBoxRF:");
	pt[0] = pointInBoxRF[0];
	pt[1] = pointInBoxRF[1];
	pt[2] = pointInBoxRF[2];
	m_LandmarkPoints->InsertPoint(count, pt);
	//std::cout << "m_LandmarkPoints->GetSize(): " << m_LandmarkPoints->GetSize() << std::endl;
	return m_LandmarkPoints->GetSize();
}

int lancetAlgorithm::SystemPrecision::CollectICPPoints()
{
	vtkSmartPointer<vtkMatrix4x4> TLandmark2Camera = vtkSmartPointer<vtkMatrix4x4>::New();
	TLandmark2Camera->DeepCopy(CalculateLandmartBox2Camera());

	Eigen::Vector3d pointInCamera = m_AimCamera->GetToolTipByName(m_ProbeModelStr);

	Eigen::Vector3d pointInLandMark = this->TransformByMatrix(pointInCamera, TLandmark2Camera);
	int count = m_ICPPoints->GetSize();
	mitk::PointSet::PointType pt;
	PrintDataHelper::CoutArray(pointInLandMark, "pointInLandMark:");
	pt[0] = pointInLandMark[0];
	pt[1] = pointInLandMark[1];
	pt[2] = pointInLandMark[2];
	m_ICPPoints->InsertPoint(count, pt);
	
	return m_ICPPoints->GetSize();
}

void lancetAlgorithm::SystemPrecision::Reset()
{
	m_LandmarkPoints->Clear();
	m_ICPPoints->Clear();
	m_LandmarkRegistrationMatrix->Identity();
	m_ICPRegistrationMatrix->Identity();
	m_FinalRegistrationMatrix->Identity();
	m_IsUseImageReg = false;
}

void lancetAlgorithm::SystemPrecision::UpdateDeviation()
{
	m_IsUseImageReg = true;
}

vtkSmartPointer<vtkMatrix4x4> lancetAlgorithm::SystemPrecision::GetBoxRF2BoxMatrix()
{
	vtkSmartPointer<vtkMatrix4x4> ret = vtkSmartPointer<vtkMatrix4x4>::New();
	ret->DeepCopy(m_FinalRegistrationMatrix);
	return ret;
}

void lancetAlgorithm::SystemPrecision::SetBoxRF2BoxMatrix(vtkMatrix4x4* matrix)
{
	m_FinalRegistrationMatrix->DeepCopy(matrix);
}

void lancetAlgorithm::SystemPrecision::DisplayTCPActor()
{
	if (m_TCPActor)
		return;
	m_TCPActor = this->GenerateAxesActor();
	AddAxesActor(m_TCPActor);

	connect(m_AimCamera, &AimCamera::CameraUpdateClock, this, &SystemPrecision::UpdateTCPActor);
}

void lancetAlgorithm::SystemPrecision::DisplayTargetActor(vtkMatrix4x4* aBaseToTarget)
{
	if (!m_TargetPosActor)
	{
		m_TargetPosActor = GenerateAxesActor();
		AddAxesActor(m_TargetPosActor);
	}
	vtkSmartPointer<vtkMatrix4x4> TImage2Base = vtkSmartPointer<vtkMatrix4x4>::New();
	TImage2Base->DeepCopy(this->CalculateBase2Image());
	TImage2Base->Invert();

	vtkSmartPointer<vtkMatrix4x4> TImage2Target = vtkSmartPointer<vtkMatrix4x4>::New();
	TImage2Target->DeepCopy(PreConcatenateMatrixs(TImage2Base, aBaseToTarget));
	m_TargetPosActor->SetUserMatrix(TImage2Target);
}

void lancetAlgorithm::SystemPrecision::DisplayTCPInRF()
{
	if (!m_TCPInRFActor)
	{
		m_TCPInRFActor = GenerateAxesActor();
		AddAxesActor(m_TCPInRFActor);
	}

	PrintDataHelper::CoutMatrix("m_Marker2TCPMatrix", m_Marker2TCPMatrix);
	connect(m_AimCamera, &AimCamera::CameraUpdateClock, this, &SystemPrecision::UpdateTCPInRF);
}

void lancetAlgorithm::SystemPrecision::DisplayFlangeAxesActor()
{
	if (m_FlangeActor)
		return;
	m_FlangeActor = GenerateAxesActor();
	AddAxesActor(m_FlangeActor);
	connect(m_AimCamera, &AimCamera::CameraUpdateClock, this, &SystemPrecision::UpdateFlange);
}

void lancetAlgorithm::SystemPrecision::PrintTCPInCamera(QTextBrowser* browser)
{
	vtkSmartPointer<vtkMatrix4x4> TCamera2EndRF = vtkSmartPointer<vtkMatrix4x4>::New();
	TCamera2EndRF->DeepCopy(m_AimCamera->GetToolMatrixByName(m_RobotEndRF));

	vtkSmartPointer<vtkMatrix4x4> TEndRF2TCP = vtkSmartPointer<vtkMatrix4x4>::New();
	TEndRF2TCP->DeepCopy(m_Marker2TCPMatrix);

	vtkSmartPointer<vtkMatrix4x4> TBase2TCP = vtkSmartPointer<vtkMatrix4x4>::New();
	TBase2TCP->DeepCopy(m_DianaSevenRobot->GetBaseToTCP());

	vtkSmartPointer<vtkMatrix4x4> TCamera2BaseRF = vtkSmartPointer<vtkMatrix4x4>::New();
	TCamera2BaseRF->DeepCopy(m_AimCamera->GetToolMatrixByName(m_RobotBaseRF));

	vtkSmartPointer<vtkMatrix4x4> TBaseRF2Base = vtkSmartPointer<vtkMatrix4x4>::New();
	TBaseRF2Base->DeepCopy(m_LancetRobReg->getBaseToBaseRF());

	vtkSmartPointer<vtkMatrix4x4> TCamera2TCP = vtkSmartPointer<vtkMatrix4x4>::New();
	TCamera2TCP->DeepCopy(PreConcatenateMatrixs(TCamera2EndRF, TEndRF2TCP));
	vtkSmartPointer<vtkMatrix4x4> TCamera2RobotTCP = vtkSmartPointer<vtkMatrix4x4>::New();
	TCamera2RobotTCP->DeepCopy(PreConcatenateMatrixs(TCamera2BaseRF, TBaseRF2Base, TBase2TCP));

	PrintDataHelper::AppendTextBrowserMatrix(browser, "TCamera2TCP", TCamera2TCP);

	PrintDataHelper::AppendTextBrowserMatrix(browser, "TCamera2RobotTCP", TCamera2RobotTCP);
	PrintDataHelper::AppendTextBrowserMatrix(browser, "m_TCamera2RobotTCP", m_TCamera2RobotTCP);
	PrintDataHelper::AppendTextBrowserMatrix(browser, "m_Image2Camera", m_Image2Camera);


	PrintDataHelper::AppendTextBrowserMatrix(browser, "Camera2ImageTCP", m_TCPInRFActor->GetUserMatrix());

	PrintDataHelper::AppendTextBrowserMatrix(browser, "Camera2ImageRobotTCP", m_TCPActor->GetUserMatrix());
}


void lancetAlgorithm::SystemPrecision::SetGoLinePrecisionTCP(/*Eigen::Vector3d pointA, Eigen::Vector3d pointB*/)
{
	Eigen::Vector3d pointB(-52.508, -43, -23.8);
	Eigen::Vector3d pointA(2.492, -43, 28.2);

	Eigen::Vector3d xDirection = pointB - pointA;
	xDirection.normalize();

	Eigen::Vector3d zFlange(0.0, -1.0, 0.0);

	Eigen::Vector3d yDirection = zFlange.cross(xDirection);
	yDirection.normalize();

	Eigen::Vector3d zDirection = xDirection.cross(yDirection);

	vtkSmartPointer<vtkMatrix4x4> TMarker2PrecisionTCP = vtkSmartPointer<vtkMatrix4x4>::New();
	TMarker2PrecisionTCP->Identity();
	TMarker2PrecisionTCP->SetElement(0, 0, xDirection.x());
	TMarker2PrecisionTCP->SetElement(1, 0, xDirection.y());
	TMarker2PrecisionTCP->SetElement(2, 0, xDirection.z());

	TMarker2PrecisionTCP->SetElement(0, 1, yDirection.x());
	TMarker2PrecisionTCP->SetElement(1, 1, yDirection.y());
	TMarker2PrecisionTCP->SetElement(2, 1, yDirection.z());

	TMarker2PrecisionTCP->SetElement(0, 2, zDirection.x());
	TMarker2PrecisionTCP->SetElement(1, 2, zDirection.y());
	TMarker2PrecisionTCP->SetElement(2, 2, zDirection.z());

	TMarker2PrecisionTCP->SetElement(0, 3, pointA.x());
	TMarker2PrecisionTCP->SetElement(1, 3, pointA.y());
	TMarker2PrecisionTCP->SetElement(2, 3, pointA.z());
	m_Marker2TCPMatrix->DeepCopy(TMarker2PrecisionTCP);
	PrintDataHelper::CoutMatrix("m_Marker2TCPMatrix", m_Marker2TCPMatrix);

	vtkSmartPointer<vtkMatrix4x4> TFlange2TCP = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4> TEnd2EndRF = vtkSmartPointer<vtkMatrix4x4>::New();
	TEnd2EndRF->DeepCopy(m_LancetRobReg->getFlangeToEndRF());
	TFlange2TCP->DeepCopy(PreConcatenateMatrixs(TEnd2EndRF, m_Marker2TCPMatrix));
	PrintDataHelper::CoutMatrix("TFlange2TCP", TFlange2TCP);

	m_DianaSevenRobot->SetTCP(TFlange2TCP);
}

void lancetAlgorithm::SystemPrecision::SetGoPlanePrecisionTCP()
{
	Eigen::Vector3d pointA(-52.508, -43, -23.8);
	Eigen::Vector3d pointB(2.492, -43, 28.2);
	Eigen::Vector3d pointC(2.492, -43, -61.8);

	Eigen::Vector3d xDirection = (pointC - pointB).normalized();

	Eigen::Vector3d AB = pointA - pointB;

	Eigen::Vector3d A_proj_on_X = AB.dot(xDirection) * xDirection;

	Eigen::Vector3d yDirection = (AB - A_proj_on_X).normalized();

	// 计算Z轴（X轴和Y轴的外积，并归一化）
	Eigen::Vector3d zDirection = xDirection.cross(yDirection).normalized();


	vtkSmartPointer<vtkMatrix4x4> TMarker2PrecisionTCP = vtkSmartPointer<vtkMatrix4x4>::New();
	TMarker2PrecisionTCP->Identity();
	TMarker2PrecisionTCP->SetElement(0, 0, xDirection.x());
	TMarker2PrecisionTCP->SetElement(1, 0, xDirection.y());
	TMarker2PrecisionTCP->SetElement(2, 0, xDirection.z());

	TMarker2PrecisionTCP->SetElement(0, 1, yDirection.x());
	TMarker2PrecisionTCP->SetElement(1, 1, yDirection.y());
	TMarker2PrecisionTCP->SetElement(2, 1, yDirection.z());

	TMarker2PrecisionTCP->SetElement(0, 2, zDirection.x());
	TMarker2PrecisionTCP->SetElement(1, 2, zDirection.y());
	TMarker2PrecisionTCP->SetElement(2, 2, zDirection.z());

	TMarker2PrecisionTCP->SetElement(0, 3, pointB.x());
	TMarker2PrecisionTCP->SetElement(1, 3, pointB.y());
	TMarker2PrecisionTCP->SetElement(2, 3, pointB.z());
	m_Marker2TCPMatrix->DeepCopy(TMarker2PrecisionTCP);

	vtkSmartPointer<vtkMatrix4x4> TFlange2TCP = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4> TEnd2EndRF = vtkSmartPointer<vtkMatrix4x4>::New();
	TEnd2EndRF->DeepCopy(m_LancetRobReg->getFlangeToEndRF());
	TFlange2TCP->DeepCopy(PreConcatenateMatrixs(TEnd2EndRF, m_Marker2TCPMatrix));
	m_DianaSevenRobot->SetTCP(TFlange2TCP);
}

void lancetAlgorithm::SystemPrecision::GoLine(Eigen::Vector3d point0, Eigen::Vector3d point1)
{
	vtkSmartPointer<vtkMatrix4x4> pointMatrixInBase = vtkSmartPointer<vtkMatrix4x4>::New();
	pointMatrixInBase->DeepCopy(CalculateLineTargetInBase(point0, point1));

	vtkSmartPointer<vtkMatrix4x4> tcp2Target = vtkSmartPointer<vtkMatrix4x4>::New();

	auto TCP2ImageMatrix = m_TCPInRFActor->GetUserMatrix();  //Image2TCP
	TCP2ImageMatrix->Invert();
	auto Image2TargetMatrix = m_TargetPosActor->GetUserMatrix();
	tcp2Target->DeepCopy(PreConcatenateMatrixs(TCP2ImageMatrix, Image2TargetMatrix));
	PrintDataHelper::CoutMatrix("GoLineCamera tcp2Target", tcp2Target);
	m_DianaSevenRobot->RobotTransformInTCP(tcp2Target->GetData());
}

void lancetAlgorithm::SystemPrecision::GoLineByRobot(Eigen::Vector3d point0, Eigen::Vector3d point1)
{
	vtkSmartPointer<vtkMatrix4x4> pointMatrixInBase = vtkSmartPointer<vtkMatrix4x4>::New();
	pointMatrixInBase->DeepCopy(CalculateLineTargetInBase(point0, point1));
	PrintDataHelper::CoutMatrix("GoLineByRobot base2Target", pointMatrixInBase);

	m_DianaSevenRobot->RobotTransformInBase(pointMatrixInBase->GetData());
}

void lancetAlgorithm::SystemPrecision::GoPlane(Eigen::Vector3d point0, Eigen::Vector3d point1, Eigen::Vector3d point2)
{
	auto points = AveragePointInBase(point0, point1, point2);

	Eigen::Vector3d tmpPoint0 = std::get<0>(points);
	Eigen::Vector3d tmpPoint1 = std::get<1>(points);
	Eigen::Vector3d tmpPoint2 = std::get<2>(points);
	PrintDataHelper::CoutArray(tmpPoint0, "GoPlane tmpPoint0: ");
	PrintDataHelper::CoutArray(tmpPoint1, "GoPlane tmpPoint1: ");
	PrintDataHelper::CoutArray(tmpPoint2, "GoPlane tmpPoint2: ");

	Eigen::Vector3d xDirection = (tmpPoint0 - tmpPoint1).normalized();

	// 2. 计算 tmpPoint2 相对于 tmpPoint1 的方向
	Eigen::Vector3d P12 = tmpPoint2 - tmpPoint1;

	// 3. 计算 tmpPoint2 在 X 轴方向的投影
	Eigen::Vector3d P12_proj_on_X = P12.dot(xDirection) * xDirection;

	// 4. 计算Y轴（去掉X轴分量，并归一化）
	Eigen::Vector3d yDirection = (P12 - P12_proj_on_X).normalized();

	// 5. 计算Z轴（X轴和Y轴的外积，并归一化）
	Eigen::Vector3d zDirection = xDirection.cross(yDirection).normalized();

	vtkSmartPointer<vtkMatrix4x4> targetInBase = vtkSmartPointer<vtkMatrix4x4>::New();
	targetInBase->Identity();
	targetInBase->SetElement(0, 0, xDirection.x());
	targetInBase->SetElement(1, 0, xDirection.y());
	targetInBase->SetElement(2, 0, xDirection.z());

	targetInBase->SetElement(0, 1, yDirection.x());
	targetInBase->SetElement(1, 1, yDirection.y());
	targetInBase->SetElement(2, 1, yDirection.z());

	targetInBase->SetElement(0, 2, zDirection.x());
	targetInBase->SetElement(1, 2, zDirection.y());
	targetInBase->SetElement(2, 2, zDirection.z());

	targetInBase->SetElement(0, 3, tmpPoint1.x());
	targetInBase->SetElement(1, 3, tmpPoint1.y());
	targetInBase->SetElement(2, 3, tmpPoint1.z());

	PrintDataHelper::CoutMatrix("GoPlane targetInBase: ", targetInBase);
	DisplayTargetActor(targetInBase);
	vtkSmartPointer<vtkMatrix4x4> tcp2Target = vtkSmartPointer<vtkMatrix4x4>::New();

	auto TCP2ImageMatrix = m_TCPInRFActor->GetUserMatrix();  //Image2TCP
	TCP2ImageMatrix->Invert();
	auto Image2TargetMatrix = m_TargetPosActor->GetUserMatrix();
	tcp2Target->DeepCopy(PreConcatenateMatrixs(TCP2ImageMatrix, Image2TargetMatrix));
	PrintDataHelper::CoutMatrix("tcp2Target", tcp2Target);
	m_DianaSevenRobot->RobotTransformInTCP(tcp2Target->GetData());
	//m_DianaAimHardwareService->RobotTransformInBase(targetInBase->GetData());
}

void lancetAlgorithm::SystemPrecision::LoadSingleMitkFile(mitk::DataStorage* dataStorage, std::string filePath, std::string nodeName)
{
	auto Node = dataStorage->GetNamedNode(nodeName);
	if (!Node)
	{
		mitk::DataNode::Pointer node = mitk::DataNode::New();
		node->SetData(mitk::IOUtil::Load(filePath)[0]);

		node->SetName(nodeName);

		dataStorage->Add(node);
	}
	else
	{
		Node->SetData(mitk::IOUtil::Load(filePath)[0]);
	}
}
