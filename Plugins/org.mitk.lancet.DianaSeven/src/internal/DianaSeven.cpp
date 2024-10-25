/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


// Blueberry
#include <berryISelectionService.h>
#include <berryIWorkbenchWindow.h>

// Qmitk
#include "DianaSeven.h"


// Qt
#include <QMessageBox>

// mitk image
#include <mitkImage.h>

const std::string DianaSeven::VIEW_ID = "org.mitk.views.dianaseven";

const double PI = 3.1415926;

void DianaSeven::SetFocus()
{
  //m_Controls.buttonPerformImageProcessing->setFocus();
}

void DianaSeven::CreateQtPartControl(QWidget* parent)
{
	// create GUI widgets from the Qt Designer's .ui file
	m_Controls.setupUi(parent);
	
	InitGlobalVariable();
	m_AimCamera = new AimCamera();
	m_DianaSevenRobot = new DianaRobot();
	m_LancetRobotRegistration = new LancetRobotRegistration(m_DianaSevenRobot, m_AimCamera);
	m_PrecisionTab = new PrecisionTab(m_Controls, this->GetDataStorage(), m_DianaSevenRobot,m_AimCamera,m_LancetRobotRegistration, parent);

	connect(m_Controls.pushButton_MoveZZJ, &QPushButton::clicked, this, &DianaSeven::move_zzj);

	connect(m_Controls.pushButton_AccuracyPosition, &QPushButton::clicked, this, &DianaSeven::PositionAccuracy);
	connect(m_Controls.pushButton_Repeatability, &QPushButton::clicked, this, &DianaSeven::PositionRepeatability);

	connect(m_Controls.pushButton_openFreeDriving, &QPushButton::clicked, this, &DianaSeven::OpenHandGuiding);
	connect(m_Controls.pushButton_closeFreeDriving, &QPushButton::clicked, this, &DianaSeven::closeHandGuiding);
	connect(m_Controls.pushButton_JointImpendance, &QPushButton::clicked, this, &DianaSeven::changeToJointImpendance);
	connect(m_Controls.pushButton_CartImpendance, &QPushButton::clicked, this, &DianaSeven::changeToCartImpendance);

	InitHardwareDeviceTabConnection();
	InitRobotRegistrationTabConnection();
	auto renderWindowPart = this->GetRenderWindowPart();
	if (nullptr != renderWindowPart)
		this->RenderWindowPartActivated(renderWindowPart);
}

void DianaSeven::RenderWindowPartActivated(mitk::IRenderWindowPart* renderWindowPart)
{
	m_PrecisionTab->SetIRenderWindowPart(renderWindowPart);
}

void DianaSeven::RenderWindowPartDeactivated(mitk::IRenderWindowPart* renderWindowPart)
{

}


void DianaSeven::move_zzj()
{
	double x = m_Controls.lineEdit_X->text().toDouble();
	double y = m_Controls.lineEdit_Y->text().toDouble();
	double z = m_Controls.lineEdit_Z->text().toDouble();
	m_DianaSevenRobot->Translate(x, y, z);
}

void DianaSeven::wait_move(const char* m_RobotIpAddress)
{
	QThread::msleep(20);
	while (true)
	{
		const char state = getRobotState();
		if (state != 0)
		{
			//qDebug() << "222";
			MITK_INFO << "222";
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

void DianaSeven::OpenHandGuiding()
{
	changeControlMode(T_MODE_POSITION, m_RobotIpAddress);
	freeDriving(1, m_RobotIpAddress);
}

void DianaSeven::closeHandGuiding()
{
	freeDriving(0, m_RobotIpAddress);
}

void DianaSeven::changeToCartImpendance()
{
	changeControlMode(T_MODE_CART_IMPEDANCE, m_RobotIpAddress);
}

void DianaSeven::changeToJointImpendance()
{
	changeControlMode(T_MODE_JOINT_IMPEDANCE, m_RobotIpAddress);
}

void DianaSeven::PositionAccuracy()
{

}

void DianaSeven::PositionRepeatability()
{
	double PosRepeatInitial[6] = { 0.272069,0.355601,0.347205,0.01958,-0.088089,0.01899 };
	moveJToPose(PosRepeatInitial, vel, acc, nullptr, zv_shaper_order, zv_shaper_frequency, zv_shaper_damping_ratio);
	wait_move(m_RobotIpAddress);

	QThread::msleep(3000);
}


void DianaSeven::TargetPointData()//?
{
	MITK_INFO << "TargetPointData m_probeToProbeNewMatrix";
	m_probeToProbeNewMatrix->Print(std::cout);
	Eigen::Matrix3d m_probeToprobeNew;

	m_probeToprobeNew << m_probeToProbeNewMatrix->GetElement(0, 0), m_probeToProbeNewMatrix->GetElement(0, 1), m_probeToProbeNewMatrix->GetElement(0, 2),
		m_probeToProbeNewMatrix->GetElement(1, 0), m_probeToProbeNewMatrix->GetElement(1, 1), m_probeToProbeNewMatrix->GetElement(1, 2),
		m_probeToProbeNewMatrix->GetElement(2, 0), m_probeToProbeNewMatrix->GetElement(2, 1), m_probeToProbeNewMatrix->GetElement(2, 2);

	Eigen::Vector3d eulerAngle = m_probeToprobeNew.eulerAngles(0, 1, 2);
	// 转换为角度
	eulerAngle = eulerAngle * (180.0 / PI);
	//旋转顺序z-y-x
	dTargetPoint.clear();
	dTargetPoint.push_back(m_probeToProbeNewMatrix->GetElement(0, 3));
	dTargetPoint.push_back(m_probeToProbeNewMatrix->GetElement(1, 3));
	dTargetPoint.push_back(m_probeToProbeNewMatrix->GetElement(2, 3));
	dTargetPoint.push_back(eulerAngle(2));
	dTargetPoint.push_back(eulerAngle(1));
	dTargetPoint.push_back(eulerAngle(0));
	MITK_INFO << "dTargetPoint:" << dTargetPoint[0] << "," << dTargetPoint[1] << "," << dTargetPoint[2] << "," << dTargetPoint[3]
		<< "," << dTargetPoint[4] << "," << dTargetPoint[5];

}

void DianaSeven::InitHardwareDeviceTabConnection()
{
	connect(m_Controls.ConnectKukaBtn, &QPushButton::clicked, this, &DianaSeven::ConnectRobotBtnClicked);
	connect(m_Controls.PowerOnBtn, &QPushButton::clicked, this, &DianaSeven::PowerOnBtnClicked);
	connect(m_Controls.PowerOffBtn, &QPushButton::clicked, this, &DianaSeven::PowerOffBtnClicked);
	connect(m_Controls.ConnectNDIBtn, &QPushButton::clicked, this, &DianaSeven::ConnectCameraClicked);
	connect(m_Controls.UpdateCameraBtn, &QPushButton::clicked, this, &DianaSeven::UpdateCameraBtnClicked);
	connect(m_AimCamera, &AimCamera::CameraUpdateClock, this, &DianaSeven::HandleUpdateRenderRequest);
	
}

void DianaSeven::InitRobotRegistrationTabConnection()
{
	//Translate
	connect(m_Controls.RobotXPlusBtn, &QPushButton::clicked, this, [=]() {
		double movementDirection[3] = { 1, 0,0 };
		Translate(movementDirection);
		});
	connect(m_Controls.RobotXMinusBtn, &QPushButton::clicked, this, [=]() {
		double movementDirection[3] = { -1, 0,0 };
		Translate(movementDirection);
		});
	connect(m_Controls.RobotYPlusBtn, &QPushButton::clicked, this, [=]() {
		double movementDirection[3] = { 0, 1,0 };
		Translate(movementDirection);
		});
	connect(m_Controls.RobotYMinusBtn, &QPushButton::clicked, this, [=]() {
		double movementDirection[3] = { 0, -1,0 };
		Translate(movementDirection);
		});
	connect(m_Controls.RobotZPlusBtn, &QPushButton::clicked, this, [=]() {
		double movementDirection[3] = { 0, 0,1 };
		Translate(movementDirection);
		});
	connect(m_Controls.RobotZMinusBtn, &QPushButton::clicked, this, [=]() {
		double movementDirection[3] = { 0, 0,-1 };
		Translate(movementDirection);
		});

	//Rotate
	connect(m_Controls.RobotRXPlusBtn, &QPushButton::clicked, this, [=]() {
		double rotationDirection[3] = { 1, 0,0 };
		Rotate(rotationDirection);
		});
	connect(m_Controls.RobotRXMinusBtn, &QPushButton::clicked, this, [=]() {
		double rotationDirection[3] = { -1, 0,0 };
		Rotate(rotationDirection);
		});
	connect(m_Controls.RobotRYPlusBtn, &QPushButton::clicked, this, [=]() {
		double rotationDirection[3] = { 0, 1,0 };
		Rotate(rotationDirection);
		});
	connect(m_Controls.RobotRYMinusBtn, &QPushButton::clicked, this, [=]() {
		double rotationDirection[3] = { 0, -1,0 };
		Rotate(rotationDirection);
		});
	connect(m_Controls.RobotRZPlusBtn, &QPushButton::clicked, this, [=]() {
		double rotationDirection[3] = { 0, 0,1 };
		Rotate(rotationDirection);
		});
	connect(m_Controls.RobotRZMinusBtn, &QPushButton::clicked, this, [=]() {
		double rotationDirection[3] = { 0, 0,-1 };
		Rotate(rotationDirection);
		});

	connect(m_Controls.SetTcpToFlangeBtn, &QPushButton::clicked, this, [this]() {m_DianaSevenRobot->SetTCPToFlange(); } );
	connect(m_Controls.RecordInitPosBtn, &QPushButton::clicked, this, [this]() {m_DianaSevenRobot->RecordInitialPos(); } );
	connect(m_Controls.GoToInitPosBtn, &QPushButton::clicked, this, [this]() {m_DianaSevenRobot->GoToInitialPos(); });
	connect(m_Controls.CaptureRobotBtn, &QPushButton::clicked, this, [this]() 
		{m_Controls.CaptureCountLineEdit->setText(QString::number(m_LancetRobotRegistration->captureRobot())); });
	connect(m_Controls.RobotAutoRegistationBtn, &QPushButton::clicked, this, [this]() {m_LancetRobotRegistration->autoCollection(); } );
	//connect(m_Controls.ResetRobotRegistrationBtn, &QPushButton::clicked, this, [this]()
	//	{m_Controls.CaptureCountLineEdit->setText(QString::number(m_LancetRobotRegistration->replaceRegistration())); });
	connect(m_Controls.ResetRobotRegistrationBtn, &QPushButton::clicked, this, [this]()
		{
			int newCount = m_LancetRobotRegistration->replaceRegistration(); // 获取返回值
			m_Controls.CaptureCountLineEdit->setText(QString::number(newCount)); // 设置文本
		});

	connect(m_Controls.SaveRobotRegistrationBtn, &QPushButton::clicked, this, [this]() 
		{
			std::cout << "save" << std::endl; 
			QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
			if (filename.isNull()) return;
			std::string baseRF2ToBaseFileName = "T_BaseRFToBase.txt";
			std::string flangeToEndRFFileName = "T_FlangeToEndRF.txt";

			FileIO::SaveMatrix2File(FileIO::CombinePath(filename.toStdString(), baseRF2ToBaseFileName).string(), m_LancetRobotRegistration->getBaseRFToBase());//BaseRFToBase??   Perhaps we need to reverse it
			FileIO::SaveMatrix2File(FileIO::CombinePath(filename.toStdString(), flangeToEndRFFileName).string(), m_LancetRobotRegistration->getFlangeToEndRF());
		});
	connect(m_Controls.ReuseRobotRegistationBtn, &QPushButton::clicked, this, [this]() 
		{
			std::cout << "Reuse" << std::endl; 
			QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
			if (filename.isNull()) return;
			std::string baseToBaseRFFileName = "T_BaseRFToBase.txt";
			std::string flangeToEndRFFileName = "T_FlangeToEndRF.txt";
			vtkSmartPointer<vtkMatrix4x4> baseRFToBase = vtkSmartPointer<vtkMatrix4x4>::New();
			vtkSmartPointer<vtkMatrix4x4> flangeToEndRF = vtkSmartPointer<vtkMatrix4x4>::New();
			FileIO::ReadTextFileAsvtkMatrix(FileIO::CombinePath(filename.toStdString(), baseToBaseRFFileName).string(), baseRFToBase);
			FileIO::ReadTextFileAsvtkMatrix(FileIO::CombinePath(filename.toStdString(), flangeToEndRFFileName).string(), flangeToEndRF);
			m_LancetRobotRegistration->getBaseRFToBase();
			m_LancetRobotRegistration->getFlangeToEndRF();
			PrintDataHelper::CoutMatrix("TBaseRF2Base", baseRFToBase);
			PrintDataHelper::CoutMatrix("TFlange2EndRF", flangeToEndRF);
		});

	connect(m_Controls.StopRobotMoveBtn, &QPushButton::clicked, this, [this]() {m_DianaSevenRobot->stopRobot(); });
	connect(m_Controls.ClearRobotErrorInfoBtn, &QPushButton::clicked, this, [this]() {m_DianaSevenRobot->CleanRobotErrorInfo(); });

	connect(m_Controls.ReadRobotJointAnglesBtn, &QPushButton::clicked, this, &DianaSeven::ReadRobotJointAnglesBtnClicked);
	connect(m_Controls.SetRobotJointAnglesBtn, &QPushButton::clicked, this, &DianaSeven::SetRobotJointAnglesBtnClicked);
	connect(m_Controls.GetRobotJointsLimitBtn, &QPushButton::clicked, this, &DianaSeven::GetRobotJointsLimitBtnClicked);

	connect(m_Controls.SetRobotPositionModeBtn, &QPushButton::clicked, this, [this]() {m_DianaSevenRobot->SetPositionMode(); });
	connect(m_Controls.SetRobotJointsImpedanceModelBtn, &QPushButton::clicked, this, [this]() {m_DianaSevenRobot->SetJointImpendanceMode(); });
	connect(m_Controls.SetRobotCartImpedanceModeBtn, &QPushButton::clicked, this, [this]() {m_DianaSevenRobot->SetCartImpendanceMode(); });
	
	connect(m_Controls.ReadRobotImpedaBtn, &QPushButton::clicked, this, &DianaSeven::ReadRobotImpedaBtnClicked);
	connect(m_Controls.SetRobotImpedaBtn, &QPushButton::clicked, this, &DianaSeven::SetRobotImpedaBtnClicked);
	
	connect(m_Controls.AppendToolTipBtn, &QPushButton::clicked, this, &DianaSeven::AppendToolTipBtnClicked);
	connect(m_Controls.AppendToolMatrixBtn, &QPushButton::clicked, this, &DianaSeven::AppendToolMatrixBtnClicked);
	connect(m_LancetRobotRegistration, &LancetRobotRegistration::countPose, this, &DianaSeven::upDateRegistionLineEdit);
}

bool DianaSeven::Translate(const double axis[3])
{
	double direction[3] = { axis[0], axis[1], axis[2] }; // 方向向量
	double length = m_Controls.TranslateDistanceLineEdit->text().toDouble(); // 移动长度
	m_DianaSevenRobot->Translate(direction, length); // 调用DianaRobot的Translate函数
	return true;
}

bool DianaSeven::Rotate(const double axis[3])
{
	double localAxis[3];
	for (int i = 0; i < 3; ++i) {
		localAxis[i] = axis[i];
	}

	m_DianaSevenRobot->Rotate(localAxis, m_Controls.RotateAngleLineEdit->text().toDouble());
	return true;
}

void DianaSeven::upDateRegistionLineEdit(int aCount)
{
	m_Controls.CaptureCountLineEdit->setText(QString::number(aCount));
}

void DianaSeven::ConnectRobotBtnClicked()
{
	m_DianaSevenRobot->Connect();
}

void DianaSeven::PowerOnBtnClicked()
{
	m_DianaSevenRobot->PowerOn();
}

void DianaSeven::PowerOffBtnClicked()
{
	m_DianaSevenRobot->PowerOff();
}

void DianaSeven::ConnectCameraClicked()
{
	m_AimCamera->Connect();
}

void DianaSeven::UpdateCameraBtnClicked()
{
	std::cout << "UpdateCameraBtnClicked" << std::endl;
	std::vector<QLabel*>* lables = new std::vector<QLabel*>();
	lables->push_back(m_Controls.PKARobotBaseRF);
	//lables->push_back(m_Controls.PKAFemurRF);
	lables->push_back(m_Controls.PKATibiaRF);
	//lables->push_back(m_Controls.BluntProbe);
	lables->push_back(m_Controls.PKARobotEndRF);
	lables->push_back(m_Controls.PKAProbe);
	//lables->push_back(m_Controls.PKADrill);

	/*std::vector<std::string> toolsName = {"RobotBaseRF", "PKAFemurRF", "PKATibiaRF", "BluntProbe", "RobotEndRF", "PKAProbe", "PKADrill"};*/
	std::vector<std::string> toolsName = { "RobotBaseRF",  "VerificationBlock", "RobotEndRF", "Probe"};
	m_AimCamera->InitToolsName(toolsName);
	m_AimCamera->CameraUpdateClock();
	//m_DianaAimHardwareService->UpdateCamera();
}

void DianaSeven::HandleUpdateRenderRequest()
{
	this->RequestRenderWindowUpdate();
}

void DianaSeven::ReadRobotJointAnglesBtnClicked()
{
	auto angles = m_DianaSevenRobot->GetJointAngles();
	for (int i = 0; i < angles.size(); ++i)
	{
		m_JointAngleLineEdits[i]->setText(QString::number(angles[i] * 180 / PI));
	}
}

void DianaSeven::SetRobotJointAnglesBtnClicked()
{
	double angles[7] = { 0.0 };
	for (int i = 0; i < m_JointAngleLineEdits.size(); ++i)
	{
		angles[i] = m_JointAngleLineEdits[i]->text().toDouble() / 180 * PI;
	}
	bool ret = m_DianaSevenRobot->SetJointAngles(angles);

}

void DianaSeven::GetRobotJointsLimitBtnClicked()
{
	auto range = m_DianaSevenRobot->GetJointAngles();

	auto convertAndPrint = [](const std::vector<double>& values, const std::string& label) {
		std::cout << label;
		for (const auto& val : values)
		{
			std::cout << (val * 180 / PI) << " ";
		}
		std::cout << std::endl;
	};

	//convertAndPrint(range[0], "min Range: ");
	//convertAndPrint(range[1], "max Range: ");//不知道咋改，emo了？？？
}

void DianaSeven::ReadRobotImpedaBtnClicked()
{
	auto Impeda = m_DianaSevenRobot->GetRobotImpeda();
	for (int i = 0; i < m_ImpedaLineEdits.size(); ++i)
	{
		m_ImpedaLineEdits[i]->setText(QString::number(Impeda[i]));
	}
}

void DianaSeven::SetRobotImpedaBtnClicked()//这个地方存疑，帮我瞅瞅这样改对不对
{
	std::vector<double> data;
	for (int i = 0; i < m_ImpedaLineEdits.size(); ++i)
	{
		data.push_back(m_ImpedaLineEdits.size());
	}

	m_DianaSevenRobot->SetRobotImpeda(data);
}

void DianaSeven::AppendToolTipBtnClicked()
{
	auto name = m_Controls.ToolNameComboBox->currentText();
	auto tip = m_AimCamera->GetToolTipByName(name.toStdString());
	
	QString str;
	for (int i = 0; i < tip.size(); ++i)
	{
		str += QString::number(tip[i]) + " ";
	}
	str = QString(name) + " " + str;
	m_Controls.textBrowser->append(str);
}

void DianaSeven::AppendToolMatrixBtnClicked()
{
	auto name = m_Controls.ToolNameComboBox->currentText();
	auto matrix = m_AimCamera->GetToolMatrixByName(name.toStdString())->GetData();

	m_Controls.textBrowser->append(name);
	for (int i = 0; i < 4; ++i)
	{
		QString row;
		for (int j = 0; j < 4; ++j)
		{
			row += QString::number(matrix[i * 4 + j]) + " ";
		}
		m_Controls.textBrowser->append(row);
	}
}

void DianaSeven::OnSelectionChanged(berry::IWorkbenchPart::Pointer /*source*/,
                                                const QList<mitk::DataNode::Pointer> &nodes)
{
  //// iterate all selected objects, adjust warning visibility
  foreach (mitk::DataNode::Pointer node, nodes)
  {
    if (node.IsNotNull() && dynamic_cast<mitk::Image *>(node->GetData()))
    {
      return;
    }
  }
}

void DianaSeven::DoImageProcessing()
{
}

void DianaSeven::InitGlobalVariable()
{
	m_JointAngleLineEdits.push_back(m_Controls.RobotJoint1AngleLineEdit);
	m_JointAngleLineEdits.push_back(m_Controls.RobotJoint2AngleLineEdit);
	m_JointAngleLineEdits.push_back(m_Controls.RobotJoint3AngleLineEdit);
	m_JointAngleLineEdits.push_back(m_Controls.RobotJoint4AngleLineEdit);
	m_JointAngleLineEdits.push_back(m_Controls.RobotJoint5AngleLineEdit);
	m_JointAngleLineEdits.push_back(m_Controls.RobotJoint6AngleLineEdit);
	m_JointAngleLineEdits.push_back(m_Controls.RobotJoint7AngleLineEdit);

	m_ImpedaLineEdits.push_back(m_Controls.arrStiffXLineEdit);
	m_ImpedaLineEdits.push_back(m_Controls.arrStiffYLineEdit);
	m_ImpedaLineEdits.push_back(m_Controls.arrStiffZLineEdit);
	m_ImpedaLineEdits.push_back(m_Controls.arrStiffRXLineEdit);
	m_ImpedaLineEdits.push_back(m_Controls.arrStiffRYLineEdit);
	m_ImpedaLineEdits.push_back(m_Controls.arrStiffRZLineEdit);
	m_ImpedaLineEdits.push_back(m_Controls.DampingRatioLineEdit);
}


