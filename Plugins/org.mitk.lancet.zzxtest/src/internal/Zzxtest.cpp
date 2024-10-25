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
#include "Zzxtest.h"

// Qt
#include <QMessageBox>
#include<qbuttongroup.h>
// mitk image
#include <mitkImage.h>
#include "AimPositionAPI.h"
#include "AimPositionDef.h"
const std::string Zzxtest::VIEW_ID = "org.mitk.views.zzxtest";

using   namespace   std;
void Zzxtest::SetFocus()
{

}

void Zzxtest::CreateQtPartControl(QWidget* parent)
{
	// create GUI widgets from the Qt Designer's .ui file
	m_Controls.setupUi(parent);
	//Robot 类实例化
	//Robot = std::make_unique<Robot_Hans>("192.168.0.10", 10003);
	Robot = new Robot_Hans("192.168.0.10", 10003); 
	//按钮绑定
	initSlot();
	//check_IsMoving();
	//thread test
	//初始化mitk选择的框
	InitSurfaceSelector(m_Controls.mitkNodeSelectWidget_surface_regis); // Select a icp surface
	InitPointSetSelector(m_Controls.mitkNodeSelectWidget_landmark_src); // Select a landmark pointset
	InitPointSetSelector(m_Controls.mitkNodeSelectWidget_imageTargetPlane);
	InitPointSetSelector(m_Controls.mitkNodeSelectWidget_imageTargetLine);
	
}
void Zzxtest::initSlot()
{
	//tab1
	connect(m_Controls.pushButton_powerOn, &QPushButton::clicked, this, &Zzxtest::powerOn);//简单的进行上使能
	connect(m_Controls.pushButton_powerOff, &QPushButton::clicked, this, &Zzxtest::powerOff);//简单的进行下使能
	connect(m_Controls.pushButton_selfCheck, &QPushButton::clicked, this, &Zzxtest::PrintToolMatrix);
	connect(m_Controls.pushButton_connectCamrea, &QPushButton::clicked, this, &Zzxtest::connectCamera);//连接设置工具文件路径、对所有的工具名进行遍历并打印
	connect(m_Controls.pushButton_updataData, &QPushButton::clicked, this, &Zzxtest::openCameraQtTimer);//通过定时器触发进行相机更新，拿到数据指针通过工件名称对数据进行索引。通过memcpy进行矩阵的赋值
	connect(m_Controls.pushButton_SuddenStop, &QPushButton::clicked, this, &Zzxtest::SuddenStop);
	connect(m_Controls.pushButton_connectArm, &QPushButton::clicked, this, &Zzxtest::connectArm);//连接机械臂并对一些报错进行打印
	connect(m_Controls.pushButton_cleantext, &QPushButton::clicked, m_Controls.textBrowser, &QTextBrowser::clear);//

	//机械臂移动检测槽函数
	//运动坐标系选择
	connect(m_Controls.comboBox_robotMoveType_select, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &Zzxtest::onComboBoxIndexChanged);
	//移动
	connect(m_Controls.pushButton_xp, &QPushButton::clicked, this, &Zzxtest::xp);
	connect(m_Controls.pushButton_yp, &QPushButton::clicked, this, &Zzxtest::yp);
	connect(m_Controls.pushButton_zp, &QPushButton::clicked, this, &Zzxtest::zp);
	connect(m_Controls.pushButton_xm, &QPushButton::clicked, this, &Zzxtest::xm);
	connect(m_Controls.pushButton_ym, &QPushButton::clicked, this, &Zzxtest::ym);
	connect(m_Controls.pushButton_zm, &QPushButton::clicked, this, &Zzxtest::zm);
	connect(m_Controls.pushButton_rxp, &QPushButton::clicked, this, &Zzxtest::rxp);
	connect(m_Controls.pushButton_ryp, &QPushButton::clicked, this, &Zzxtest::ryp);
	connect(m_Controls.pushButton_rzp, &QPushButton::clicked, this, &Zzxtest::rzp);
	connect(m_Controls.pushButton_rxm, &QPushButton::clicked, this, &Zzxtest::rxm);
	connect(m_Controls.pushButton_rym, &QPushButton::clicked, this, &Zzxtest::rym);
	connect(m_Controls.pushButton_rzm, &QPushButton::clicked, this, &Zzxtest::rzm);
	//table2 Robot registration
	connect(m_Controls.pushButton_setTCPToFlange, &QPushButton::clicked, this, &Zzxtest::setTCPToFlange);//现在为0值
	connect(m_Controls.pushButton_setInitialPoint, &QPushButton::clicked, this, &Zzxtest::setInitialPoint);//读取机械臂当前各种信息
	connect(m_Controls.pushButton_goToInitial, &QPushButton::clicked, this, &Zzxtest::goToInitial);//路点运动？？
	connect(m_Controls.pushButton_captureRobot, &QPushButton::clicked, this, &Zzxtest::captureRobot);
	connect(m_Controls.pushButton_replaceRegistration, &QPushButton::clicked, this, &Zzxtest::replaceRegistration);
	connect(m_Controls.pushButton_saveArmMatrix, &QPushButton::clicked, this, &Zzxtest::saveArmMatrix);
	connect(m_Controls.pushButton_reuseArmMatrix, &QPushButton::clicked, this, &Zzxtest::reuseArmMatrix);
	connect(m_Controls.pushButton_JiontAngleMotion, &QPushButton::clicked, this, &Zzxtest::JiontAngleMotion);
	connect(m_Controls.pushButton_openIsMoving, &QPushButton::clicked, this, &Zzxtest::openIsMoing);
	connect(m_Controls.pushButton_CloseIsMoving, &QPushButton::clicked, this, &Zzxtest::CloseIsMoving);
	//移动方式选择
	connect(m_Controls.comboBox_robotMoveType, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &Zzxtest::onComboBox_moveIndexchange);
	connect(m_Controls.pushButton_AutoMoveJ, &QPushButton::clicked, this, &Zzxtest::AutoMoveJ);

	//精度测试
	connect(m_Controls.pushButton_ResetImageConfiguration, &QPushButton::clicked, this, &Zzxtest::ResetImageConfiguration);
	connect(m_Controls.pushButton_saveImageMatrix, &QPushButton::clicked, this, &Zzxtest::saveImageMatrix);
	connect(m_Controls.pushButton_reuseImageMatrix, &QPushButton::clicked, this, &Zzxtest::reuseImageMatrix);
	connect(m_Controls.pushButton_collectLandmark, &QPushButton::clicked, this, &Zzxtest::collectLandmark);
	connect(m_Controls.pushButton_landmarkRegistration, &QPushButton::clicked, this, &Zzxtest::landmarkRegistration);
	connect(m_Controls.pushButton_collectICP, &QPushButton::clicked, this, &Zzxtest::collectICP);
	connect(m_Controls.pushButton_ICPRegistration, &QPushButton::clicked, this, &Zzxtest::ICPRegistration);
	//系统精度测试--走面---走线
	connect(m_Controls.pushButton_setPlanePrecisionTestTcp, &QPushButton::clicked, this, &Zzxtest::SetPlanePrecisionTestTcp);
	connect(m_Controls.pushButton_confirmImageTargetPlane, &QPushButton::clicked, this, &Zzxtest::InterpretImagePlane);
	connect(m_Controls.pushButton_goToFakePlane, &QPushButton::clicked, this, &Zzxtest::On_pushButton_goToFakePlane_clicked);
	connect(m_Controls.pushButton_gotoPlaneEdge, &QPushButton::clicked, this, &Zzxtest::OnAutoPositionStart);

	connect(m_Controls.pushButton_setTcpPrecisionTest, &QPushButton::clicked, this, &Zzxtest::SetPrecisionTestTcp);
	connect(m_Controls.pushButton_confirmImageTargetLine, &QPushButton::clicked, this, &Zzxtest::InterpretImageLine);
}
//void Zzxtest::thread_test(const std::array<double, 6>& jointValue, const std::array<double, 6>& endPosition)
//{
//	// 假设 UI 中有两个 QLabel 来显示关节值和末端位置
//	m_Controls.label_2->setText(QString("Joint Value: [%1, %2, %3, %4, %5, %6]")
//		.arg(jointValue[0]).arg(jointValue[1]).arg(jointValue[2])
//		.arg(jointValue[3]).arg(jointValue[4]).arg(jointValue[5]));
//
//	m_Controls.label_2->setText(QString("End Position: [%1, %2, %3, %4, %5, %6]")
//		.arg(endPosition[0]).arg(endPosition[1]).arg(endPosition[2])
//		.arg(endPosition[3]).arg(endPosition[4]).arg(endPosition[5]));
//}
void Zzxtest::InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget)
{
	widget->SetDataStorage(GetDataStorage());
	widget->SetNodePredicate(mitk::NodePredicateAnd::New(
		mitk::TNodePredicateDataType<mitk::PointSet>::New(),
		mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(mitk::NodePredicateProperty::New("helper object"),
			mitk::NodePredicateProperty::New("hidden object")))));

	widget->SetSelectionIsOptional(true);
	widget->SetAutoSelectNewNodes(true);
	widget->SetEmptyInfo(QString("Please select a point set"));
	widget->SetPopUpTitel(QString("Select point set"));
}
void Zzxtest::InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget)
{
	widget->SetDataStorage(GetDataStorage());
	widget->SetNodePredicate(mitk::NodePredicateAnd::New(
		mitk::TNodePredicateDataType<mitk::Surface>::New(),
		mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(mitk::NodePredicateProperty::New("helper object"),
			mitk::NodePredicateProperty::New("hidden object")))));

	widget->SetSelectionIsOptional(true);
	widget->SetAutoSelectNewNodes(true);
	widget->SetEmptyInfo(QString("Please select a surface"));
	widget->SetPopUpTitel(QString("Select surface"));
}
void Zzxtest::openIsMoing()
{
	open_QTimercheck_IsMoving();
}

void Zzxtest::CloseIsMoving()
{
	if (QTimer_IsMoving != nullptr)
	{
		QTimer_IsMoving->stop();
		delete QTimer_IsMoving;
		QTimer_IsMoving = nullptr;
	}
}

void Zzxtest::open_QTimercheck_IsMoving()
{
	// 创建定时器
	if (QTimer_IsMoving == nullptr)
	{
		QTimer_IsMoving = new QTimer(this);
		// 连接定时器超时信号到槽函数，而不是直接调用
		connect(QTimer_IsMoving, &QTimer::timeout, this, &Zzxtest::check_IsMoving);
	}

	// 启动定时器，每秒执行一次
	QTimer_IsMoving->start(1000);
}

void Zzxtest::check_IsMoving()
{
	// 检查机器人是否在移动，更新按钮状态
	if (Robot->isMoving(movingstatue))
	{
		m_Controls.pushButton_SuddenStop->setStyleSheet("background-color: green; color: white;");
	}
	else
	{
		m_Controls.pushButton_SuddenStop->setStyleSheet("background-color: red; color: white;");
	}
}
//---------------------------------------------------------------------------------------------------------------
/**
 * @brief 连接机械臂
 */
 //---------------------------------------------------------------------------------------------------------------
void Zzxtest::connectArm()
{
	std::string error;
	this->Robot->connect(error);
	std::cout << error << std::endl;
}

//---------------------------------------------------------------------------------------------------------------
/**
 * @brief 打开机械臂电源
 */
 //---------------------------------------------------------------------------------------------------------------
void Zzxtest::powerOn()
{
	std::string error;
	this->Robot->PowerOn(error);
	std::cout << "error" << std::endl;
}

//---------------------------------------------------------------------------------------------------------------
/**
 * @brief 关闭机械臂电源
 */
 //---------------------------------------------------------------------------------------------------------------
void Zzxtest::powerOff()
{
	std::string error;
	this->Robot->PowerOff(error);
	std::cout << "error" << std::endl;
}
void Zzxtest::connectCamera()
{
	T_AIMPOS_DATAPARA mPosDataPara;
	Aim_API_Initial(aimHandle);
	Aim_SetEthernetConnectIP(aimHandle, 192, 168, 31, 10);
	rlt = Aim_ConnectDevice(aimHandle, I_ETHERNET, mPosDataPara);

	m_Controls.textBrowser->append("-------------------------------------------------------------");
	if (rlt == AIMOOE_OK)
	{
		m_Controls.textBrowser->append("Aimooe Connect Success");
		std::cout << "connect success";
	}
	else
	{
		m_Controls.textBrowser->append("Aimooe Connect Failed");
		std::cout << "connect failed";
	}

	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;
	filename.append("/");
	std::cout << "The selected folder address :" << filename;
	rlt = Aim_SetToolInfoFilePath(aimHandle, filename.toLatin1().data());

	if (rlt == AIMOOE_OK)
	{
		m_Controls.textBrowser->append("set filenemae success");
		std::cout << "set filenemae success";
	}
	else
	{
		std::cout << "set filenemae failed";
		m_Controls.textBrowser->append("set filenemae failed");
	}

	int size = 0;
	Aim_GetCountOfToolInfo(aimHandle, size);
	//对所有的工具名进行遍历并打印
	if (size != 0)
	{
		t_ToolBaseInfo* toolarr = new t_ToolBaseInfo[size];

		rlt = Aim_GetAllToolFilesBaseInfo(aimHandle, toolarr);

		if (rlt == AIMOOE_OK)
		{
			for (int i = 0; i < size; i++)
			{
				char* ptool = toolarr[i].name;
				//QString toolInfo = QString::fromUtf8(ptool); 解决乱码
				QString toolInfo = QString(ptool);
				m_Controls.textBrowser->append(toolInfo);
			}
		}
		delete[] toolarr;
	}
	else
	{
		std::cout << "There are no tool identification files in the current directory:";
		m_Controls.textBrowser->append("There are no tool identification files in the current directory:");

	}

	std::cout << "End of connection";
	m_Controls.textBrowser->append("End of connection");

	rlt = AIMOOE_OK;
	m_Controls.textBrowser->append("-------------------------------------------------------------");
}
/**
 * @brief 更新相机数据
 */
 //---------------------------------------------------------------------------------------------------------------
void Zzxtest::openCameraQtTimer()
{
	if (m_AimoeVisualizeTimer == nullptr)
	{
		m_AimoeVisualizeTimer = new QTimer(this);
	}

	connect(m_AimoeVisualizeTimer, SIGNAL(timeout()), this, SLOT(updateCameraData())); //拿数据
	m_AimoeVisualizeTimer->start(100);
}
void Zzxtest::updateCameraData()
{
	auto prlt = GetNewToolData();//ptr指针用来获取数据

	if (rlt == AIMOOE_OK)//判断是否采集成功
	{
		do
		{
			//获取数据
			UpdateCameraToToolMatrix(prlt, "HTO_RobotBaseRF", T_CamToBaseRF, m_Controls.Spine_RobotBaseRFDataLabel);
			UpdateCameraToToolMatrix(prlt, "HTO_RobotEndRF", T_CamToEndRF, m_Controls.Spine_RobotEndRFDataLabel);
			UpdateCameraToToolMatrix(prlt, "HTO_PatientRF", T_CamToPatientRF, m_Controls.Spine_PatientRFDataLabel);
			UpdateCameraToToolMatrix(prlt, "HTO_Probe", T_CamToProbe, m_Controls.Spine_ProbeDataLabel);

			//获取Spine_Probe数据
			if (strcmp(prlt->toolname, "Oral_Probe") == 0)
			{
				if (prlt->validflag)
				{
					ProbeTop[0] = prlt->tooltip[0];
					ProbeTop[1] = prlt->tooltip[1];
					ProbeTop[2] = prlt->tooltip[2];
				}
			}

			T_AimToolDataResult* pnext = prlt->next;
			delete prlt;
			prlt = pnext;
		} while (prlt != NULL);
		/*cout << endl;*/

	}
	else
	{
		delete prlt;
	}
}
/**
 * @brief 获取新的工具数据
 *
 * @return T_AimToolDataResult* 指向新工具数据的指针
 */
 //---------------------------------------------------------------------------------------------------------------
T_AimToolDataResult* Zzxtest::GetNewToolData()
{
	rlt = Aim_GetMarkerAndStatusFromHardware(aimHandle, I_ETHERNET, markerSt, statusSt);
	if (rlt == AIMOOE_NOT_REFLASH)
	{
		std::cout << "camera get data failed";
	}
	T_AimToolDataResult* mtoolsrlt = new T_AimToolDataResult;//新建一个值指，将指针清空用于存数据
	mtoolsrlt->next = NULL;
	mtoolsrlt->validflag = false;

	rlt = Aim_FindToolInfo(aimHandle, markerSt, mtoolsrlt, 0);//获取数据
	T_AimToolDataResult* prlt = mtoolsrlt;//将获取完数据的从mtoolsrlt给prlt指针

	return prlt;
}
void Zzxtest::UpdateCameraToToolMatrix(T_AimToolDataResult* ToolData, const char* Name, double* aCamera2Tool, QLabel* label)
{
	/*std::cout << "run UpdateCameraToToolMatrix" << std::endl;*/
	if (strcmp(ToolData->toolname, Name) == 0)
	{
		if (ToolData->validflag)
		{
			//获取相机数据
			t_tran[0] = ToolData->Tto[0];
			t_tran[1] = ToolData->Tto[1];
			t_tran[2] = ToolData->Tto[2];
			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					R_tran[i][j] = ToolData->Rto[i][j];
				}
			}
			//拼接矩阵
			vtkNew<vtkMatrix4x4> matrix;
			CombineRotationTranslation(R_tran, t_tran, matrix);
			memcpy_s(aCamera2Tool, sizeof(double) * 16, matrix->GetData(), sizeof(double) * 16);


			if (label != nullptr)
			{
				QString str = "x:" + QString::number(ToolData->tooltip[0]) + " "
					+ "y:" + QString::number(ToolData->tooltip[1]) + " "
					+ "z:" + QString::number(ToolData->tooltip[2]);
				label->setText(str);
				label->setStyleSheet("QLabel { color : green; }");
				/*std::cout << str << std::endl;*/
			}
		}
		else
		{
			if (label != nullptr)
			{
				QString str = "x:nan  y:nan   z:nan";
				label->setText(str);
				/*std::cout << str << std::endl;*/
				label->setStyleSheet("QLabel { color : red; }");
			}
		}
	}
}
/**
 * @brief 合并旋转和平移矩阵
 * @param Rto 旋转矩阵
 * @param Tto 平移向量
 * @param resultMatrix 结果矩阵指针
 */
 //---------------------------------------------------------------------------------------------------------------
void Zzxtest::CombineRotationTranslation(float Rto[3][3], float Tto[3], vtkMatrix4x4* resultMatrix)
{
	// Set rotation part
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			resultMatrix->SetElement(i, j, Rto[i][j]);
		}
	}
	for (int i = 0; i < 3; ++i)
	{
		resultMatrix->SetElement(i, 3, Tto[i]);
	}
}
 void Zzxtest::PrintToolMatrix()
{

	callCount++;
	std::cout << "Function has been called " << callCount << " times." << std::endl;


	PrintArray16ToMatrix("T_CamToEndRF", T_CamToEndRF);//打印T_CamToEndRF
	PrintArray16ToMatrix("T_CamToBaseRF", T_CamToBaseRF);//打印T_CamToBaseRF
	PrintArray16ToMatrix("T_CamToProbe", T_CamToProbe);//打印T_CamToProbe
	std::cout << "-------------------------------------------" << std::endl;
	std::cout << "ProbeTop: " << ProbeTop[0] << "/" << ProbeTop[1] << "/" << ProbeTop[2] << std::endl;//探针针尖
	std::cout << "-------------------------------------------" << std::endl;
	PrintArray16ToMatrix("T_CamToPatientRF", T_CamToPatientRF);//打印T_CamToPatientRF
	PrintArray16ToMatrix("T_BaseToBaseRF", T_BaseToBaseRF);//打印T_CamToPatientRF
	PrintArray16ToMatrix("T_FlangeToEdnRF", T_FlangeToEndRF);//打印T_FlangeToEdnRF

	PrintArray16ToMatrix("T_PatientRFToimage", T_PatientRFtoImage);//
	PrintArray16ToMatrix("T_ImageToImage_icp", T_ImageToImage_icp);//
}
void Zzxtest::PrintArray16ToMatrix(const std::string& title, double* Array)
 {
	 std::cout << "+-----------------------------------------------+" << std::endl;
	 std::cout << "|               " << title << "              |" << std::endl;
	 std::cout << "+-----------+-----------+----------+------------+" << std::endl;
	 for (int i = 0; i < 4; i++)
	 {
		 std::cout << "|  ";
		 for (int j = 0; j < 4; j++)
		 {
			 std::cout << std::fixed << std::setprecision(6) << Array[i * 4 + j] << " ";
		 }
		 std::cout << "|" << std::endl;
	 }
	 std::cout << "+-----------+-----------+----------+------------+" << std::endl;
 }
void Zzxtest::Print_Matrix(const std::string& title, vtkMatrix4x4* matrix)
 {
	 std::cout << title << std::endl;
	 std::cout << "+-----------------------------------------------+" << std::endl;
	 std::cout << "|       Matrix: " << title << "              |" << std::endl;
	 std::cout << "+-----------+-----------+----------+------------+" << std::endl;
	 for (int i = 0; i < 4; i++)
	 {
		 std::cout << "|  ";
		 for (int j = 0; j < 4; j++)
		 {
			 std::cout << std::fixed << std::setprecision(6) << matrix->GetElement(i, j) << " ";
		 }
		 std::cout << "|" << std::endl;
	 }
	 std::cout << "+-----------+-----------+----------+------------+" << std::endl;
 }
void Zzxtest::SuddenStop()
{
	std::string error;
	this->Robot->Stop(error);
	m_Controls.textBrowser->append(QString::fromStdString(error));
}
void Zzxtest::onComboBoxIndexChanged(int index)
{
	QString selectedOption = m_Controls.comboBox_robotMoveType_select->itemText(index);
	qDebug() << "User selected option:" << selectedOption;
	nToolMotion = index;
}
void Zzxtest::xp()
{
	std::string error;
	int Distance = m_Controls.lineEdit_intuitiveValue->text().toInt();
	this->Robot->RelMove(error, 0, 1, Distance, nToolMotion);	
}
void Zzxtest::yp()
{
	std::string error;
	int Distance = m_Controls.lineEdit_intuitiveValue->text().toInt();
	this->Robot->RelMove(error, 1, 1, Distance, nToolMotion);
}
void Zzxtest::zp()
{
	std::string error;
	int Distance = m_Controls.lineEdit_intuitiveValue->text().toInt();
	this->Robot->RelMove(error, 2, 1, Distance, nToolMotion);
}
void Zzxtest::xm()
{
	std::string error;
	int Distance = m_Controls.lineEdit_intuitiveValue->text().toInt();
	this->Robot->RelMove(error, 0, 0, Distance, nToolMotion);
}
void Zzxtest::ym()
{
	std::string error;
	int Distance = m_Controls.lineEdit_intuitiveValue->text().toInt();
	this->Robot->RelMove(error, 1, 0, Distance, nToolMotion);
}
void Zzxtest::rxp()
{
	std::string error;
	int Distance = m_Controls.lineEdit_intuitiveValue_degree->text().toInt();
	this->Robot->RelMove(error, 3, 1, Distance, nToolMotion);
}
void Zzxtest::ryp()
{
	std::string error;
	int Distance = m_Controls.lineEdit_intuitiveValue_degree->text().toInt();
	this->Robot->RelMove(error, 4, 1, Distance, nToolMotion);
}
void Zzxtest::rzp()
{
	std::string error;
	int Distance = m_Controls.lineEdit_intuitiveValue_degree->text().toInt();
	this->Robot->RelMove(error, 5, 1, Distance, nToolMotion);
}
void Zzxtest::rzm()
{
	std::string error;
	int Distance = m_Controls.lineEdit_intuitiveValue_degree->text().toInt();
	this->Robot->RelMove(error, 5, 0, Distance, nToolMotion);
}
void Zzxtest::rym()
{
	std::string error;
	int Distance = m_Controls.lineEdit_intuitiveValue_degree->text().toInt();
	this->Robot->RelMove(error, 4, 0, Distance, nToolMotion);
}
void  Zzxtest::zm()
{
	std::string error;
	int Distance = m_Controls.lineEdit_intuitiveValue->text().toInt();
	this->Robot->RelMove(error, 2, 0, Distance, nToolMotion);
}
void  Zzxtest::rxm()
{
	std::string error;
	int Distance = m_Controls.lineEdit_intuitiveValue_degree->text().toInt();
	this->Robot->RelMove(error, 3, 0, Distance, nToolMotion);
}
void  Zzxtest::setTCPToFlange()
{
	std::string error;
	std::array<double, 6>TCP = { 0,0,0,0,0,0 };
	this->Robot->setTCP(TCP, error);
}
void Zzxtest::configTCP2Flange()
{
	std::string error;
	std::array<double, 6>TCP = { 0,0,0,0,0,0 };
	std::string TCP_Name = "TCP_1";
	this->Robot->configTcp(TCP, TCP_Name, error);

}
void Zzxtest::setInitialPoint()
{
	std::string error;
	Init_JointPose = this->Robot->getJointValue();
	automove_target_init = this->Robot->getEndPosition();
	double distance = m_Controls.lineEdit_intuitiveValue->text().toDouble();
    double digree = m_Controls.lineEdit_intuitiveValue_degree->text().toDouble();
	calcute_automove_list(automove_target_init,distance,digree);
}
void Zzxtest::calcute_automove_list(const std::array<double, 6> P, int Distance, double degree)
{
	// 基于 automove_target_init 计算偏移点位
	std::array<double, 6> init = automove_target_init;

	// 6个偏移方向
	automove_target_map["x+"] = { init[0] + Distance, init[1], init[2], init[3], init[4], init[5] };
	automove_target_map["x-"] = { init[0] - Distance, init[1], init[2], init[3], init[4], init[5] };
	automove_target_map["y+"] = { init[0], init[1] + Distance, init[2], init[3], init[4], init[5] };
	automove_target_map["y-"] = { init[0], init[1] - Distance, init[2], init[3], init[4], init[5] };
	automove_target_map["z+"] = { init[0], init[1], init[2] + Distance, init[3], init[4], init[5] };
	automove_target_map["z-"] = { init[0], init[1], init[2] - Distance, init[3], init[4], init[5] };

	// 6个旋转方向
	automove_target_map["rx+"] = { init[0], init[1], init[2], init[3] + degree, init[4], init[5] };
	automove_target_map["rx-"] = { init[0], init[1], init[2], init[3] - degree, init[4], init[5] };
	automove_target_map["ry+"] = { init[0], init[1], init[2], init[3], init[4] + degree, init[5] };
	automove_target_map["ry-"] = { init[0], init[1], init[2], init[3], init[4] - degree, init[5] };
	automove_target_map["rz+"] = { init[0], init[1], init[2], init[3], init[4], init[5] + degree };
	automove_target_map["rz-"] = { init[0], init[1], init[2], init[3], init[4], init[5] - degree };

	// 打印计算结果（可选）
	for (const auto& [key, value] : automove_target_map)
	{
		std::cout << key << ": ";
		for (double v : value)
		{
			std::cout << v << " ";
		}
		std::cout << std::endl;
	}
}
void Zzxtest::goToInitial()
{
	std::string error;
	this->Robot->movej(Init_JointPose,error);
}
/**
 * @brief 机械臂重新配准
 */
 //---------------------------------------------------------------------------------------------------------------
void Zzxtest::replaceRegistration()
{
	m_Controls.textBrowser->append("Replace Registration");
	m_RobotRegistration.RemoveAllPose();
	m_IndexOfRobotCapture = 0;
	m_Controls.lineEdit_collectedRoboPose->setText(QString::number(0));
	auto_move_index = 0;
}
void  Zzxtest::saveArmMatrix()
{
	std::ofstream robotMatrixFile(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_BaseToBaseRF.txt");
	for (int i = 0; i < 16; i++)
	{
		robotMatrixFile << T_BaseToBaseRF[i];
		if (i != 15)
		{
			robotMatrixFile << ",";
		}
		else
		{
			robotMatrixFile << ";";
		}
	}
	robotMatrixFile << std::endl;
	robotMatrixFile.close();


	std::ofstream robotMatrixFile1(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_FlangeToEndRF.txt");
	for (int i = 0; i < 16; i++)
	{
		robotMatrixFile1 << T_FlangeToEndRF[i];
		if (i != 15)
		{
			robotMatrixFile1 << ",";
		}
		else
		{
			robotMatrixFile1 << ";";
		}
	}
	robotMatrixFile1 << std::endl;
	robotMatrixFile1.close();
	m_Controls.textBrowser->append("saveArmMatrix");
}
/**
 * @brief 读取机械臂配准矩阵T_BaseToBaseRF、T_FlangeToEndRF
 * @note 读取在桌面save文件夹的T_BaseToBaseRF.txtT_FlangeToEndRF.txt
 */
 //---------------------------------------------------------------------------------------------------------------
void Zzxtest::reuseArmMatrix()
{

	std::ifstream inputFile(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_BaseToBaseRF.txt");
	if (inputFile.is_open())
	{
		std::string line;
		if (std::getline(inputFile, line))
		{
			std::stringstream ss(line);
			std::string token;
			int index = 0;
			while (std::getline(ss, token, ','))
			{
				T_BaseToBaseRF[index] = std::stod(token);
				index++;
			}
		}
		inputFile.close();
	}
	else
	{

		m_Controls.textBrowser->append("无法打开文件:T_BaseToBaseRF.txt");
	}

	PrintArray16ToMatrix("T_BaseToBaseRF", T_BaseToBaseRF);



	//导入T_FlangeToEdnRF
	std::ifstream inputFile2(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_FlangeToEndRF.txt");
	if (inputFile2.is_open())
	{
		std::string line2;
		if (std::getline(inputFile2, line2))
		{
			std::stringstream ss2(line2);
			std::string token2;
			int index2 = 0;
			while (std::getline(ss2, token2, ','))
			{
				T_FlangeToEndRF[index2] = std::stod(token2);
				index2++;
			}
		}
		inputFile2.close();
	}
	else
	{
		m_Controls.textBrowser->append("无法打开文件：T_FlangeToEndRF.txt");
	}

	//打印T_FlangeToEdnRF
	PrintArray16ToMatrix("T_FlangeToEdnRF", T_FlangeToEndRF);

}
void Zzxtest::captureRobot()
{
	m_Controls.textBrowser->append("captureRobot");
	if (m_IndexOfRobotCapture < 5) //The first five translations, 
	{
		m_IndexOfRobotCapture++;
		std::cout << "m_IndexOfRobotCapture: " << m_IndexOfRobotCapture << std::endl;
		m_Controls.lineEdit_collectedRoboPose->setText(QString::number(m_IndexOfRobotCapture));
		CapturePose(true);


	}
	else if (m_IndexOfRobotCapture < 10) //the last five rotations
	{


		m_IndexOfRobotCapture++;
		m_Controls.lineEdit_collectedRoboPose->setText(QString::number(m_IndexOfRobotCapture));
		std::cout << "m_IndexOfRobotCapture: " << m_IndexOfRobotCapture << std::endl;

		CapturePose(false);
	}
	else
	{
		//MITK_INFO << "OnRobotCapture finish: " << m_IndexOfRobotCapture;
		vtkNew<vtkMatrix4x4> robotEndToFlangeMatrix;
		m_RobotRegistration.GetTCPmatrix(robotEndToFlangeMatrix);

		vtkMatrix4x4* matrix4x4 = vtkMatrix4x4::New();
		m_RobotRegistration.GetRegistraionMatrix(matrix4x4);


		double x = robotEndToFlangeMatrix->GetElement(0, 3);
		double y = robotEndToFlangeMatrix->GetElement(1, 3);
		double z = robotEndToFlangeMatrix->GetElement(2, 3);
		std::cout << "X: " << x << std::endl;
		std::cout << "Y: " << y << std::endl;
		std::cout << "Z: " << z << std::endl;

		robotEndToFlangeMatrix->Invert();

		//
		m_Controls.textBrowser->append("Registration RMS: " + QString::number(m_RobotRegistration.RMS()));
		std::cout << "Registration RMS: " << m_RobotRegistration.RMS() << std::endl;


		vtkMatrix4x4* vtkT_BaseToBaseRF = vtkMatrix4x4::New();
		m_RobotRegistration.GetRegistraionMatrix(vtkT_BaseToBaseRF);
		vtkT_BaseToBaseRF->Invert();
		memcpy_s(T_BaseToBaseRF, sizeof(double) * 16, vtkT_BaseToBaseRF->GetData(), sizeof(double) * 16);


		vtkMatrix4x4* vtkT_FlangeToEndRF = vtkMatrix4x4::New();
		m_RobotRegistration.GetTCPmatrix(vtkT_FlangeToEndRF);
		memcpy_s(T_FlangeToEndRF, sizeof(double) * 16, vtkT_FlangeToEndRF->GetData(), sizeof(double) * 16);


	}
	
}
void Zzxtest::CapturePose(bool translationOnly)
{
	//Read TCP get T_BaseToFlanger
	std::string error;
	std::array<double, 6> T_BaseToFlanger_fromRobot = this->Robot->getEndPosition();
	//先平移再 zyx 旋转
	auto tmpTrans = vtkTransform::New();
	tmpTrans->PostMultiply();
	tmpTrans->RotateX(T_BaseToFlanger_fromRobot[3]);
	tmpTrans->RotateY(T_BaseToFlanger_fromRobot[4]);
	tmpTrans->RotateZ(T_BaseToFlanger_fromRobot[5]);
	tmpTrans->Translate(T_BaseToFlanger_fromRobot[0], T_BaseToFlanger_fromRobot[1], T_BaseToFlanger_fromRobot[2]);
	tmpTrans->Update();
	//VTKT_BaseToFlanger
	vtkSmartPointer<vtkMatrix4x4> VTKT_BaseToFlanger = tmpTrans->GetMatrix();

	//camera_to_end
	auto VTKT_CameratoEndRF = vtkSmartPointer<vtkMatrix4x4>::New();
	VTKT_CameratoEndRF->DeepCopy(T_CamToEndRF);

	//baserf_to_camera
	auto VTKT_BaseRFToCamera = vtkSmartPointer<vtkMatrix4x4>::New();
	VTKT_BaseRFToCamera->DeepCopy(T_CamToBaseRF);
	VTKT_BaseRFToCamera->Invert();

	//VTKT_BassRFToEndRF
	vtkNew<vtkTransform> tmpTransform;
	tmpTransform->PostMultiply();//设置连乘，首先set设置初始的，后面Concatenate 放在左边
	tmpTransform->Identity();//初始化单位阵
	tmpTransform->SetMatrix(VTKT_CameratoEndRF);
	tmpTransform->Concatenate(VTKT_BaseRFToCamera);
	tmpTransform->Update();
	auto vtkBaseRFtoRoboEndRFMatrix = tmpTransform->GetMatrix();

	//Robotic arm registration
	m_RobotRegistration.AddPoseWithVtkMatrix(VTKT_BaseToFlanger, vtkBaseRFtoRoboEndRFMatrix, translationOnly);
}
void Zzxtest::onComboBox_moveIndexchange(int index)
{
	QString selectedOption = m_Controls.comboBox_robotMoveType->itemText(index);
	qDebug() << "User selected option:" << selectedOption;
	moveType = index;
	std::cout << index << std::endl;
}
void Zzxtest::JiontAngleMotion()
{
	std::array<double, 6>Pose = { 0 };
	textEditProcess(Pose, m_Controls.lineEdit);
	std::string error;
	if (moveType == 0)
		this->Robot->moveP(Pose, error);
	else
		this->Robot->movej(Pose, error);
}
void Zzxtest::AutoMoveJ()
{
	auto_move_index++;
	goToInitial();  // 重置初始位置

	bool statue;

	// 等待机械臂停止运动
	while (this->Robot->isMoving(statue))
	{
		QThread::msleep(100);
		QApplication::processEvents();
	}

	double distance = m_Controls.lineEdit_intuitiveValue->text().toDouble();
	double degree = m_Controls.lineEdit_intuitiveValue_degree->text().toDouble();
	std::string error;

	// 基于 auto_move_index 选择目标移动点
	switch (auto_move_index)
	{
	case 1:
		// 移动到 x+ 方向
		xp();
		break;
	case 2:
		// 移动到 x- 方向
		xm();
		break;
	case 3:
		// 移动到 y+ 方向
		yp();
		break;
	case 4:
		// 移动到 y- 方向
		ym();
		break;
	case 5:
		// 移动到 z+ 方向
		zp();
		break;
	case 6:
		// 移动到 rx+ 方向
		rxp();
		break;
	case 7:
		// 移动到 ry+ 方向
		ryp();
		break;
	case 8:
		// 移动到 rz+ 方向
		rzp();
		break;
	case 9:
		// 移动到 rx- 方向
		rxm();
		break;
	case 10:
		// 移动到 ry- 方向
		rym();
		break;
	default:
		m_Controls.textBrowser->append(QString("Current AutoMoveJ_id: ") + QString::number(auto_move_index));
		m_Controls.textBrowser->append("robot is move 10 point,automove_id is clear");
		break;
	}

	// 再次等待机械臂完成运动
	while (this->Robot->isMoving(statue))
	{
		QThread::msleep(100);
		QApplication::processEvents();
	}

	// 暂停 0.5 秒
	QThread::msleep(500);
	QApplication::processEvents();
	// 更新摄像头数据和捕捉机械臂状态
	updateCameraData();
	captureRobot();




}
void Zzxtest::textEditProcess(std::array<double, 6>& Pose, QLineEdit* LineEdit)
{
	QString text = LineEdit->text();
	//去除空白字符
	QStringList values = text.split(',');

	if (values.size() == 6)
	{
		bool ok;
		for (int i = 0; i < 6; ++i)
		{
			Pose[i] = values[i].trimmed().toDouble(&ok);
			if (!ok)
			{
				qDebug() << "Conversion error at index" << i;
				return;
			}
		}

		// 打印数组内容以验证
		for (int i = 0; i < 6; ++i)
		{
			qDebug() << "[" << i << "] =" << Pose[i];
		}
	}
	else
	{
		qDebug() << "Input does not contain 6 values";
	}
}

void Zzxtest::ResetImageConfiguration()
{
	m_IndexOfLandmark = 0;
	m_IndexOfICP = 0;

	m_Controls.lineEdit_collectedLandmark->setText(QString::number(0));
	m_Controls.lineEdit_collectedICP->setText(QString::number(0));

	vtkProbeTip_onObjRf->Reset();
	vtkProbeTip_onObjRf_icp->Reset();

	if (vtkProbeTip_onObjRf->GetNumberOfPoints() == 0 && vtkProbeTip_onObjRf_icp->GetNumberOfPoints() == 0)
	{
		m_Controls.textBrowser->append("Replace image configuration");
	}
	else
	{
		m_Controls.textBrowser->append("Replace image configuration failed");
	}
}
void Zzxtest::saveImageMatrix()
{
	//��T_PatientRFtoImage
	std::ofstream robotMatrixFile(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_PatientRFtoImage.txt");
	for (int i = 0; i < 16; i++)
	{
		robotMatrixFile << T_PatientRFtoImage[i];
		if (i != 15)
		{
			robotMatrixFile << ",";
		}
		else
		{
			robotMatrixFile << ";";
		}
	}
	robotMatrixFile << std::endl;
	robotMatrixFile.close();

	//T_ImageToImage_icp
	std::ofstream robotMatrixFile1(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_ImageToImage_icp.txt");
	for (int i = 0; i < 16; i++)
	{
		robotMatrixFile1 << T_ImageToImage_icp[i];
		if (i != 15)
		{
			robotMatrixFile1 << ",";
		}
		else
		{
			robotMatrixFile1 << ";";
		}
	}
	robotMatrixFile1 << std::endl;
	robotMatrixFile1.close();
	m_Controls.textBrowser->append("saveImageMatrix");
}
void Zzxtest::reuseImageMatrix()
{
	//T_PatientRFtoImage
	std::ifstream inputFile(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_PatientRFtoImage.txt");
	if (inputFile.is_open())
	{
		std::string line;
		if (std::getline(inputFile, line))
		{
			std::stringstream ss(line);
			std::string token;
			int index = 0;
			while (std::getline(ss, token, ','))
			{
				T_PatientRFtoImage[index] = std::stod(token);
				index++;
			}
		}
		inputFile.close();
	}
	else
	{

		m_Controls.textBrowser->append("reuseImageMatrix failed:T_PatientRFtoImage.txt");
	}
	PrintArray16ToMatrix("T_PatientRFtoImage", T_PatientRFtoImage);


	std::ifstream inputFile2(std::string(getenv("USERPROFILE")) + "\\Desktop\\save\\T_ImageToImage_icp.txt");
	if (inputFile2.is_open())
	{
		std::string line2;
		if (std::getline(inputFile2, line2))
		{
			std::stringstream ss2(line2);
			std::string token2;
			int index2 = 0;
			while (std::getline(ss2, token2, ','))
			{
				T_ImageToImage_icp[index2] = std::stod(token2);
				index2++;
			}
		}
		inputFile2.close();
	}
	else
	{
		m_Controls.textBrowser->append("reuseImageMatrix failed:T_ImageToImage_icp.txt");
	}
	PrintArray16ToMatrix("T_ImageToImage_icp", T_ImageToImage_icp);
}
void Zzxtest::collectLandmark()
{
	m_IndexOfLandmark++;
	m_Controls.lineEdit_collectedLandmark->setText(QString::number(m_IndexOfLandmark));
	m_Controls.textBrowser->append(QString(" m_IndexOfLandmark: ") + QString::number(m_IndexOfLandmark));
	//ȡT_patientToProbeRF
	auto vtkT_CameraToProbe = vtkMatrix4x4::New();
	auto vtkT_PatientRFToCamera = vtkMatrix4x4::New();
	vtkT_CameraToProbe->DeepCopy(T_CamToProbe);
	vtkT_PatientRFToCamera->DeepCopy(T_CamToPatientRF);
	vtkT_PatientRFToCamera->Invert();

	auto transform = vtkTransform::New();
	transform->Identity();
	transform->PostMultiply();
	transform->SetMatrix(vtkT_CameraToProbe);
	transform->Concatenate(vtkT_PatientRFToCamera);
	transform->Update();
	auto T_patientToProbeRF = transform->GetMatrix();

	//T_patientrfToProbeRF
	memcpy_s(T_PatientRFToProbe, sizeof(double) * 16, T_patientToProbeRF->GetData(), sizeof(double) * 16);

	//
	auto tmptrans = vtkTransform::New();
	tmptrans->Identity();
	tmptrans->PostMultiply();
	tmptrans->SetMatrix(vtkT_PatientRFToCamera);
	//
	tmptrans->MultiplyPoint(ProbeTop, nd_tip_FpatientRF);
	vtkProbeTip_onObjRf->InsertNextPoint(nd_tip_FpatientRF[0], nd_tip_FpatientRF[1], nd_tip_FpatientRF[2]);

	m_Controls.textBrowser->append(QString("Probe Point Landmark: (") + QString::number(nd_tip_FpatientRF[0]) + ", " + QString::number(nd_tip_FpatientRF[1]) + ", "
		+ QString::number(nd_tip_FpatientRF[2]) + ")");

}
void Zzxtest::landmarkRegistration()
{

	auto landmarkSrcNode = m_Controls.mitkNodeSelectWidget_landmark_src->GetSelectedNode();
	mitk::PointSet::Pointer pointSet_Src = dynamic_cast<mitk::PointSet*>(landmarkSrcNode->GetData());
	if (pointSet_Src.IsNotNull())
	{
		unsigned int numberOfPoints = pointSet_Src->GetSize();
		for (unsigned int i = 0; i < numberOfPoints; ++i)
		{
			// get position
			mitk::PointSet::PointType point = pointSet_Src->GetPoint(i);

			QString pointText = QString("Point %1: (%2, %3, %4)\n").arg(i + 1).arg(point[0]).arg(point[1]).arg(point[2]);
			m_Controls.textBrowser->append(pointText);
		}
	}


	mitk::PointSet::Pointer pointSet_Tar = mitk::PointSet::New();
	vtkIdType numPoints = vtkProbeTip_onObjRf->GetNumberOfPoints();
	for (vtkIdType i = 0; i < numPoints; ++i)
	{
		double point[3];
		vtkProbeTip_onObjRf->GetPoint(i, point);

		mitk::PointSet::PointType mitkPoint;
		mitkPoint[0] = point[0];
		mitkPoint[1] = point[1];
		mitkPoint[2] = point[2];
		pointSet_Tar->SetPoint(i, mitkPoint);
	}

	if (pointSet_Tar.IsNotNull())
	{
		unsigned int numberOfPoints = pointSet_Tar->GetSize();

		for (unsigned int i = 0; i < numberOfPoints; ++i)
		{
			mitk::PointSet::PointType point = pointSet_Tar->GetPoint(i);

			QString pointText2 = QString("Point %1: (%2, %3, %4)\n").arg(i + 1).arg(point[0]).arg(point[1]).arg(point[2]);
			m_Controls.textBrowser->append(pointText2);
		}
	}

	
	
	auto vtkT_patientRFtoImage = ComputeTransformMartix(pointSet_Src, pointSet_Tar);
	memcpy_s(T_PatientRFtoImage, sizeof(double) * 16, vtkT_patientRFtoImage->GetData(), sizeof(double) * 16);

	// Create a Data Node for the second point set
	// show point in image space
	mitk::DataNode::Pointer pointSetNode2 = mitk::DataNode::New();
	pointSetNode2->SetData(pointSet_Tar);
	pointSetNode2->SetName("Simulated_PointSet_Target");

	GetDataStorage()->Add(pointSetNode2);
	QString matrixText;



}
vtkSmartPointer<vtkMatrix4x4> Zzxtest::ComputeTransformMartix(mitk::PointSet::Pointer points_set1, mitk::PointSet::Pointer points_set2)
{
	// Creat vtkPointset
	vtkSmartPointer<vtkPoints> vtkPoints1 = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> vtkPoints2 = vtkSmartPointer<vtkPoints>::New();
	// Convert mitk::PointSet to vtkPoints
	for (unsigned int i = 0; i < points_set1->GetSize(); ++i)
	{
		mitk::PointSet::PointType point = points_set1->GetPoint(i);
		vtkPoints1->InsertNextPoint(point[0], point[1], point[2]);
	}

	for (unsigned int i = 0; i < points_set2->GetSize(); ++i)
	{
		mitk::PointSet::PointType point = points_set2->GetPoint(i);
		vtkPoints2->InsertNextPoint(point[0], point[1], point[2]);
	}


	vtkSmartPointer<vtkLandmarkTransform> tmpTrans = vtkSmartPointer<vtkLandmarkTransform>::New();
	tmpTrans->SetModeToRigidBody();

	// Set source point and target point into vtkLandmarkTransform
	tmpTrans->SetSourceLandmarks(vtkPoints1);
	tmpTrans->SetTargetLandmarks(vtkPoints2);

	tmpTrans->Update();

	vtkSmartPointer<vtkMatrix4x4> transformationMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
	transformationMatrix->DeepCopy(tmpTrans->GetMatrix());

	return transformationMatrix;
}
void Zzxtest::collectICP()
{

	m_IndexOfICP++;
	m_Controls.lineEdit_collectedICP->setText(QString::number(m_IndexOfICP));
	m_Controls.textBrowser->append(QString(" m_IndexOfICP: ") + QString::number(m_IndexOfICP));

	//ȡT_patientToProbeRF
	auto vtkT_cameraToprobeRF = vtkMatrix4x4::New();
	vtkT_cameraToprobeRF->DeepCopy(T_CamToProbe);

	auto vtkT_patientRFTocamera = vtkMatrix4x4::New();
	vtkT_patientRFTocamera->DeepCopy(T_CamToPatientRF);
	vtkT_patientRFTocamera->Invert();

	auto T_ImageTopatientRF_landmark = vtkMatrix4x4::New();
	T_ImageTopatientRF_landmark->DeepCopy(T_PatientRFtoImage);
	T_ImageTopatientRF_landmark->Invert();

	auto transform = vtkTransform::New();
	transform->Identity();
	transform->PostMultiply();
	transform->SetMatrix(vtkT_cameraToprobeRF);
	transform->Concatenate(vtkT_patientRFTocamera);
	transform->Concatenate(T_ImageTopatientRF_landmark);
	transform->Update();
	auto T_ImageToProbeRF = transform->GetMatrix();


	auto tmptrans = vtkTransform::New();
	tmptrans->Identity();
	tmptrans->PostMultiply();
	//tmptrans->SetMatrix(T_ImageToProbeRF);
	tmptrans->SetMatrix(vtkT_patientRFTocamera);
	tmptrans->Concatenate(T_ImageTopatientRF_landmark);
	tmptrans->MultiplyPoint(ProbeTop, nd_tip_FImage_icp);

	vtkProbeTip_onObjRf_icp->InsertNextPoint(nd_tip_FImage_icp[0], nd_tip_FImage_icp[1], nd_tip_FImage_icp[2]);
	m_Controls.textBrowser->append(QString("Probe Point ICP: (") + QString::number(nd_tip_FImage_icp[0]) + ", " + QString::number(nd_tip_FImage_icp[1]) + ", "
		+ QString::number(nd_tip_FImage_icp[2]) + ")");

}
void Zzxtest::ICPRegistration()
{
	auto icpRegistrator = mitk::SurfaceRegistration::New();

	m_IcpSourceSurface = m_Controls.mitkNodeSelectWidget_surface_regis->GetSelectedNode();

	auto icpTargetPointset = mitk::PointSet::New();

	for (vtkIdType i = 0; i < vtkProbeTip_onObjRf_icp->GetNumberOfPoints(); ++i)
	{
		double* pointCoords = vtkProbeTip_onObjRf_icp->GetPoint(i);

		mitk::Point3D point;
		point[0] = pointCoords[0];
		point[1] = pointCoords[1];
		point[2] = pointCoords[2];


		icpTargetPointset->InsertPoint(i, point);
	}


	if (m_IcpSourceSurface != nullptr && icpTargetPointset != nullptr)
	{
		auto icpSrcSurface = dynamic_cast<mitk::Surface*>(m_IcpSourceSurface->GetData());
		icpRegistrator->SetIcpPoints(icpTargetPointset);
		icpRegistrator->SetSurfaceSrc(icpSrcSurface);
		icpRegistrator->ComputeIcpResult();

		double rms = GetRegisrationRMS(icpTargetPointset, icpSrcSurface, icpRegistrator->GetResult());
		m_Controls.textBrowser->append("rms" + QString::number(rms));
		Eigen::Matrix4d tmpRegistrationResult{ icpRegistrator->GetResult()->GetData() };
		tmpRegistrationResult.transposeInPlace();

		auto T_imageToImage_icp = vtkMatrix4x4::New();
		for (int row = 0; row < 4; ++row)
		{
			for (int col = 0; col < 4; ++col)
			{
				T_imageToImage_icp->SetElement(row, col, tmpRegistrationResult(row, col));
			}
		}

		QString output;

		output.append("Eigen::Matrix4d content:\n");
		for (int row = 0; row < tmpRegistrationResult.rows(); ++row)
		{
			for (int col = 0; col < tmpRegistrationResult.cols(); ++col)
			{
				output.append(QString::number(tmpRegistrationResult(row, col)) + " ");
			}
			output.append("\n");
		}

		m_Controls.textBrowser->append(output);

		memcpy_s(T_ImageToImage_icp, sizeof(double) * 16, T_imageToImage_icp->GetData(), sizeof(double) * 16);
	}
}

double Zzxtest::GetRegisrationRMS(mitk::PointSet* points, mitk::Surface* surface, vtkMatrix4x4* matrix)
{

	vtkNew<vtkImplicitPolyDataDistance> implicitPolyDataDistance;
	vtkTransform* tmpTrans1 = vtkTransform::New();
	tmpTrans1->Identity();
	tmpTrans1->PostMultiply();
	tmpTrans1->Concatenate(surface->GetGeometry()->GetVtkMatrix());
	//tmpTrans1->SetMatrix(GetResult()->in);
	auto tmpMatrix = vtkMatrix4x4::New();
	//vtkMatrix4x4::Invert(GetResult(), tmpMatrix);
	//tmpTrans1->Concatenate(tmpMatrix);
	tmpTrans1->Concatenate(matrix);

	vtkNew<vtkTransformFilter> transformFilter1;
	transformFilter1->SetInputData(surface->GetVtkPolyData());
	transformFilter1->SetTransform(tmpTrans1);
	transformFilter1->Update();

	int pointNum = points->GetSize();
	double maxIcpError{ 0 };
	double sumIcpError = 0;
	for (int i = 0; i < pointNum; i++)
	{
		auto currentPoint = points->GetPoint(i);
		double tmpPoint[3]{ currentPoint[0],currentPoint[1],currentPoint[2] };

		implicitPolyDataDistance->SetInput(transformFilter1->GetPolyDataOutput());

		double currentError = implicitPolyDataDistance->EvaluateFunction(tmpPoint);

		MITK_INFO << "ICP point error: " << currentError;
		sumIcpError = sumIcpError + fabs(currentError);
		if (fabs(currentError) > fabs(maxIcpError))
		{
			maxIcpError = fabs(currentError);
		}
	}

	double m_maxIcpError = maxIcpError;
	double m_avgIcpError = sumIcpError / pointNum;
	return m_avgIcpError;


}
bool  Zzxtest::SetPlanePrecisionTestTcp()
{
	

	Eigen::Vector3d p1;
	p1[0] = m_Controls.lineEdit_plane_p2_x->text().toDouble();
	p1[1] = m_Controls.lineEdit_plane_p2_y->text().toDouble();
	p1[2] = m_Controls.lineEdit_plane_p2_z->text().toDouble();

	Eigen::Vector3d p2;
	p2[0] = m_Controls.lineEdit_plane_p3_x->text().toDouble();
	p2[1] = m_Controls.lineEdit_plane_p3_y->text().toDouble();
	p2[2] = m_Controls.lineEdit_plane_p3_z->text().toDouble();

	Eigen::Vector3d p3;
	p3[0] = m_Controls.lineEdit_plane_p4_x->text().toDouble();
	p3[1] = m_Controls.lineEdit_plane_p4_y->text().toDouble();
	p3[2] = m_Controls.lineEdit_plane_p4_z->text().toDouble();

	Eigen::Vector3d x_tcp;
	x_tcp = p3 - p1;
	x_tcp.normalize();


	Eigen::Vector3d z_flange;
	z_flange[0] = 0.0;
	z_flange[1] = 0.0;
	z_flange[2] = 1;

	Eigen::Vector3d y_tcp;
	y_tcp = z_flange.cross(x_tcp);

	y_tcp.normalize();

	Eigen::Vector3d z_tcp;
	z_tcp = x_tcp.cross(y_tcp);

	Eigen::Matrix3d Re;

	Re << x_tcp[0], y_tcp[0], z_tcp[0],
		x_tcp[1], y_tcp[1], z_tcp[1],
		x_tcp[2], y_tcp[2], z_tcp[2];
	Eigen::Matrix4d T;
	T << Re(0, 0), Re(0, 1), Re(0, 2), p2[0],
		Re(1, 0), Re(1, 1), Re(1, 2), p2[1],
		Re(2, 0), Re(2, 1), Re(2, 2), p2[2],
		0, 0, 0, 1;
	for (int row = 0; row < 4; ++row)
	{
		for (int col = 0; col < 4; ++col)
		{
			T_FlangerTofinger->SetElement(row, col, T(row, col));
		}
	}
	Eigen::Vector3d eulerAngle = Re.eulerAngles(2, 1, 0);
	//------------------------------------------------
	double tcp[6];
	tcp[0] = p1[0]; // tx
	tcp[1] = p1[1]; // ty
	tcp[2] = p1[2]; // tz
	tcp[3] = eulerAngle(2) * radius2degree; //-0.81;// -0.813428203; // rx
	tcp[4] = eulerAngle(1) * radius2degree; // ry
	tcp[5] = eulerAngle(0) * radius2degree; // rz
	std::string error;
	std::array<double, 6>TCP;
	for (size_t i = 0; i < 6; i++)
	{
		TCP[i] = tcp[i];
	}
	this->Robot->setTCP(TCP, error);

	return true;
}
//前往目标平面边界
bool Zzxtest::InterpretImagePlane()
{
	//获取图像坐标系立柱工装上的点
	auto targetPlanePoints = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_imageTargetPlane->GetSelectedNode()->GetData());
	auto targetPoint_0 = targetPlanePoints->GetPoint(0); // TCP frame origin should move to this point
	auto targetPoint_1 = targetPlanePoints->GetPoint(1);
	auto targetPoint_2 = targetPlanePoints->GetPoint(2);
	std::cout << "get gongzhuang point" << std::endl;
	//打印点位
	std::cout << "targetPoint_0: (" << targetPoint_0[0] << ", " << targetPoint_0[1] << ", " << targetPoint_0[2] << ")" << std::endl;
	std::cout << "targetPoint_1: (" << targetPoint_1[0] << ", " << targetPoint_1[1] << ", " << targetPoint_1[2] << ")" << std::endl;
	std::cout << "targetPoint_1: (" << targetPoint_2[0] << ", " << targetPoint_2[1] << ", " << targetPoint_2[2] << ")" << std::endl;
	double targetPointUnderBase_0[3]{ 0 };
	//采样部分，取20个数值取平均，这部分有待调试具体sleep部分会出错，在for中确保能拿到20个数值不同的点
	vtkMatrix4x4* samples[20];
	vtkMatrix4x4* vtkT_BaseToBaseRF = vtkMatrix4x4::New();
	auto vtkT_BaseRFToCamera = vtkMatrix4x4::New();
	auto vtkT_CameraToPatientRF = vtkMatrix4x4::New();
	auto vtkT_PatientRFToImage = vtkMatrix4x4::New();
	auto vtkT_BaseToImage = vtkMatrix4x4::New();
	for (size_t i = 0; i < 20; i++)
	{
		QThread::msleep(100);
		QApplication::processEvents();
		updateCameraData();
		//获取机械臂配准矩阵T_BaseToBaseRF
		vtkT_BaseToBaseRF->DeepCopy(T_BaseToBaseRF);

		//获取T_BaseRFToCamera
		vtkT_BaseRFToCamera->DeepCopy(T_CamToBaseRF);
		vtkT_BaseRFToCamera->Invert();

		//获取T_CameraToPatientRF
		vtkT_CameraToPatientRF->DeepCopy(T_CamToPatientRF);
		//获取T_PatientRFToImage
		vtkT_PatientRFToImage->DeepCopy(T_PatientRFtoImage);
		//计算T_BaseToImage
		vtkNew<vtkTransform> Transform;
		Transform->Identity();
		Transform->PostMultiply();
		Transform->SetMatrix(vtkT_PatientRFToImage);
		Transform->Concatenate(vtkT_CameraToPatientRF);
		Transform->Concatenate(vtkT_BaseRFToCamera);
		Transform->Concatenate(vtkT_BaseToBaseRF);
		Transform->Update();
		// 创建一个新的 vtkMatrix4x4 并复制 Transform 的矩阵
		samples[i] = vtkMatrix4x4::New();
		samples[i]->DeepCopy(Transform->GetMatrix());
	}
	vtkMatrix4x4* average = vtkMatrix4x4::New();
	computeAverageTransform(samples, average);
	//得到平均值矩阵
		//vtkT_BaseToImage->DeepCopy(Transform->GetMatrix());
	vtkT_BaseToImage->DeepCopy(average);



	//get surface to target plane matrix建立图像目标点坐标系
	Eigen::Vector3d x_surfaceToPlane;
	x_surfaceToPlane << targetPoint_2[0] - targetPoint_0[0],
		targetPoint_2[1] - targetPoint_0[1],
		targetPoint_2[2] - targetPoint_0[2];
	x_surfaceToPlane.normalize();

	Eigen::Vector3d tmp_y;
	tmp_y << targetPoint_0[0] - targetPoint_1[0],
		targetPoint_0[1] - targetPoint_1[1],
		targetPoint_0[2] - targetPoint_1[2];
	tmp_y.normalize();

	Eigen::Vector3d z_surfaceToPlane = x_surfaceToPlane.cross(tmp_y);
	z_surfaceToPlane.normalize();

	Eigen::Vector3d y_surfaceToPlane = z_surfaceToPlane.cross(x_surfaceToPlane);
	y_surfaceToPlane.normalize();

	double array_surfaceToPlane[16]
	{
			x_surfaceToPlane[0], y_surfaceToPlane[0], z_surfaceToPlane[0], targetPoint_0[0],
			x_surfaceToPlane[1], y_surfaceToPlane[1], z_surfaceToPlane[1], targetPoint_0[1],
			x_surfaceToPlane[2], y_surfaceToPlane[2], z_surfaceToPlane[2], targetPoint_0[2],
			0,0,0,1
	};
	PrintMatrix("array_surfaceToPlane", array_surfaceToPlane);
	vtkNew<vtkMatrix4x4> vtkimageSurfaceToPlane;
	vtkimageSurfaceToPlane->DeepCopy(array_surfaceToPlane);

	// get base to target plane matrix
	vtkNew<vtkTransform> Trans;
	Trans->Identity();
	Trans->PostMultiply();
	Trans->SetMatrix(vtkimageSurfaceToPlane);
	Trans->Concatenate(vtkT_BaseToImage);
	Trans->Update();
	vtkBaseToTargetPlaneTransform = Trans->GetMatrix();


	Eigen::Matrix3d Re;
	Re << vtkBaseToTargetPlaneTransform->GetElement(0, 0), vtkBaseToTargetPlaneTransform->GetElement(0, 1), vtkBaseToTargetPlaneTransform->GetElement(0, 2),
		vtkBaseToTargetPlaneTransform->GetElement(1, 0), vtkBaseToTargetPlaneTransform->GetElement(1, 1), vtkBaseToTargetPlaneTransform->GetElement(1, 2),
		vtkBaseToTargetPlaneTransform->GetElement(2, 0), vtkBaseToTargetPlaneTransform->GetElement(2, 1), vtkBaseToTargetPlaneTransform->GetElement(2, 2);

	Eigen::Vector3d eulerAngle = Re.eulerAngles(2, 1, 0);
	double now_x = vtkBaseToTargetPlaneTransform->GetElement(0, 3);
	double now_y = vtkBaseToTargetPlaneTransform->GetElement(1, 3);
	double now_z = vtkBaseToTargetPlaneTransform->GetElement(2, 3);
	double now_rx = eulerAngle[2] * radius2degree;
	double now_ry = eulerAngle[1] * radius2degree;
	double now_rz = eulerAngle[0] * radius2degree;
	m_Controls.textBrowser->append("-------------------------------------------------------------------------------------------");
	m_Controls.textBrowser->append("target plane now");
	m_Controls.textBrowser->append("dx=" + QString::number(now_x));
	m_Controls.textBrowser->append("dy=" + QString::number(now_y));
	m_Controls.textBrowser->append("dz=" + QString::number(now_z));
	m_Controls.textBrowser->append("dRx=" + QString::number(now_rx));
	m_Controls.textBrowser->append("dRy=" + QString::number(now_ry));
	m_Controls.textBrowser->append("dRz=" + QString::number(now_rz));
	("-------------------------------------------------------------------------------------------");
	std::cout << " target pose" << now_x << " " << " " << now_y << " " << now_z << " " << now_rx << " " << now_ry << " " << now_rz << " " << std::endl;


	return true;
}
void Zzxtest::PrintMatrix(std::string matrixName, double* matrix)
{
	m_Controls.textBrowser->append("---------------------------------------------------");
	m_Controls.textBrowser->append(QString::fromStdString(matrixName + ":"));
	/*std::cout << matrixName + ": " << std::endl;*/
	for (int i = 0; i < 4; ++i)
	{
		std::string row;
		for (int j = 0; j < 4; ++j)
		{
			row += std::to_string(matrix[i * 4 + j]) + " ";
		}
		m_Controls.textBrowser->append(QString::fromStdString(row) + "\n");
	}
	m_Controls.textBrowser->append("---------------------------------------------------");
}
void Zzxtest::computeAverageTransform(vtkMatrix4x4* samples[], vtkMatrix4x4* average)
{
	double tmp_x[3]{ 0,0,0 };
	double tmp_y[3]{ 0,0,0 };
	double tmp_translation[3]{ 0,0,0 };
	for (int i{ 0 }; i < 20; i++)
	{


		auto tmpMatrix = samples[i];

		std::cout << "Averaging NavigationData and print the 1st element:" << tmpMatrix->GetElement(0, 0);

		tmp_x[0] += tmpMatrix->GetElement(0, 0);
		tmp_x[1] += tmpMatrix->GetElement(1, 0);
		tmp_x[2] += tmpMatrix->GetElement(2, 0);

		tmp_y[0] += tmpMatrix->GetElement(0, 1);
		tmp_y[1] += tmpMatrix->GetElement(1, 1);
		tmp_y[2] += tmpMatrix->GetElement(2, 1);

		tmp_translation[0] += tmpMatrix->GetElement(0, 3);
		tmp_translation[1] += tmpMatrix->GetElement(1, 3);
		tmp_translation[2] += tmpMatrix->GetElement(2, 3);

	}
	Eigen::Vector3d x;
	x[0] = tmp_x[0];
	x[1] = tmp_x[1];
	x[2] = tmp_x[2];
	x.normalize();

	Eigen::Vector3d h;
	h[0] = tmp_y[0];
	h[1] = tmp_y[1];
	h[2] = tmp_y[2];
	h.normalize();

	Eigen::Vector3d z;
	z = x.cross(h);
	z.normalize();

	Eigen::Vector3d y;
	y = z.cross(x);
	y.normalize();

	tmp_translation[0] = tmp_translation[0] / 20;
	tmp_translation[1] = tmp_translation[1] / 20;
	tmp_translation[2] = tmp_translation[2] / 20;
	average->Identity();
	average->SetElement(0, 0, x[0]);
	average->SetElement(0, 1, y[0]);
	average->SetElement(0, 2, z[0]);
	average->SetElement(1, 0, x[1]);
	average->SetElement(1, 1, y[1]);
	average->SetElement(1, 2, z[1]);
	average->SetElement(2, 0, x[2]);
	average->SetElement(2, 1, y[2]);
	average->SetElement(2, 2, z[2]);

	average->SetElement(0, 3, tmp_translation[0]);
	average->SetElement(1, 3, tmp_translation[1]);
	average->SetElement(2, 3, tmp_translation[2]);
}
//前往初始位置，对一个轴做偏移，也就是到达起点
void Zzxtest::On_pushButton_goToFakePlane_clicked()
{
	vtkSmartPointer<vtkMatrix4x4> t = vtkSmartPointer<vtkMatrix4x4>::New();
	t->DeepCopy(vtkBaseToTargetPlaneTransform);

	auto tmpVtkTrans = vtkTransform::New();
	//存疑 z轴偏还是x轴偏
	auto offsetMatrix = vtkMatrix4x4::New();
	offsetMatrix->Identity();
	offsetMatrix->SetElement(1, 3, 50);
	offsetMatrix->SetElement(2, 3, -50);
	tmpVtkTrans->PostMultiply();
	tmpVtkTrans->Identity();
	tmpVtkTrans->SetMatrix(offsetMatrix);
	tmpVtkTrans->Concatenate(t);
	tmpVtkTrans->Update();

	auto resultMatrix = tmpVtkTrans->GetMatrix();


	Eigen::Matrix3d Re;

	Re << resultMatrix->GetElement(0, 0), resultMatrix->GetElement(0, 1), resultMatrix->GetElement(0, 2),
		resultMatrix->GetElement(1, 0), resultMatrix->GetElement(1, 1), resultMatrix->GetElement(1, 2),
		resultMatrix->GetElement(2, 0), resultMatrix->GetElement(2, 1), resultMatrix->GetElement(2, 2);

	Eigen::Vector3d eulerAngle = Re.eulerAngles(2, 1, 0);
	std::array<double, 6>target;
	target[0] = resultMatrix->GetElement(0, 3);
	target[1] = resultMatrix->GetElement(1, 3);
	target[2] = resultMatrix->GetElement(2, 3);
	target[3] = eulerAngle[2] * radius2degree;
	target[4] = eulerAngle[1] * radius2degree;
	target[5] = eulerAngle[0] * radius2degree;
	m_Controls.textBrowser->append("-------------------------------------------------------------------------------------------");
	m_Controls.textBrowser->append("fake plan");
	m_Controls.textBrowser->append("dx=" + QString::number(target[0]));
	m_Controls.textBrowser->append("dy=" + QString::number(target[1]));
	m_Controls.textBrowser->append("dz=" + QString::number(target[2]));
	m_Controls.textBrowser->append("dRx=" + QString::number(target[3]));
	m_Controls.textBrowser->append("dRy=" + QString::number(target[4]));
	m_Controls.textBrowser->append("dRz=" + QString::number(target[5]));
	("-------------------------------------------------------------------------------------------");
	std::string error;
	this->Robot->moveP(target, error);


}
//前往目标位置
void Zzxtest::OnAutoPositionStart()
{
	Eigen::Matrix3d Re;

	Eigen::Vector3d offset;
	/*offset[0] = m_Controls.lineEdit_ax->text().toDouble();
	offset[1] = m_Controls.lineEdit_ay->text().toDouble();
	offset[2] = m_Controls.lineEdit_az->text().toDouble();*/

	Re << vtkBaseToTargetPlaneTransform->GetElement(0, 0), vtkBaseToTargetPlaneTransform->GetElement(0, 1), vtkBaseToTargetPlaneTransform->GetElement(0, 2),
		vtkBaseToTargetPlaneTransform->GetElement(1, 0), vtkBaseToTargetPlaneTransform->GetElement(1, 1), vtkBaseToTargetPlaneTransform->GetElement(1, 2),
		vtkBaseToTargetPlaneTransform->GetElement(2, 0), vtkBaseToTargetPlaneTransform->GetElement(2, 1), vtkBaseToTargetPlaneTransform->GetElement(2, 2);
	Eigen::Vector3d eulerAngle = Re.eulerAngles(2, 1, 0);
	std::array<double, 6>target;
	target[0]= vtkBaseToTargetPlaneTransform->GetElement(0, 3) + offset[0];
	target[1] = vtkBaseToTargetPlaneTransform->GetElement(1, 3) + offset[1];
	target[2] = vtkBaseToTargetPlaneTransform->GetElement(2, 3) + offset[2];
	target[3] = eulerAngle[2] * radius2degree;
	target[4] = eulerAngle[1] * radius2degree;
	target[5] = eulerAngle[0] * radius2degree;
	m_Controls.textBrowser->append("-------------------------------------------------------------------------------------------");
	m_Controls.textBrowser->append("target plane now");
	m_Controls.textBrowser->append("dx=" + QString::number(target[0]));
	m_Controls.textBrowser->append("dy=" + QString::number(target[1]));
	m_Controls.textBrowser->append("dz=" + QString::number(target[2]));
	m_Controls.textBrowser->append("dRx=" + QString::number(target[3]));
	m_Controls.textBrowser->append("dRy=" + QString::number(target[4]));
	m_Controls.textBrowser->append("dRz=" + QString::number(target[5]));
	("-------------------------------------------------------------------------------------------");
	//move
	std::string error;
	this->Robot->moveP(target, error);
}
bool Zzxtest::SetPrecisionTestTcp()
{

	//设置走线TCP
	Eigen::Vector3d p2;
	p2[0] = m_Controls.lineEdit_p2_x->text().toDouble();
	p2[1] = m_Controls.lineEdit_p2_y->text().toDouble();
	p2[2] = m_Controls.lineEdit_p2_z->text().toDouble();

	Eigen::Vector3d p3;
	p3[0] = m_Controls.lineEdit_p3_x->text().toDouble();
	p3[1] = m_Controls.lineEdit_p3_y->text().toDouble();
	p3[2] = m_Controls.lineEdit_p3_z->text().toDouble();

	Eigen::Vector3d x_tcp;
	x_tcp = p3 - p2;
	x_tcp.normalize();

	Eigen::Vector3d z_flange;
	z_flange[0] = 0.0;
	z_flange[1] = 0.0;
	z_flange[2] = 1;

	Eigen::Vector3d y_tcp;
	y_tcp = z_flange.cross(x_tcp);

	y_tcp.normalize();

	Eigen::Vector3d z_tcp;
	z_tcp = x_tcp.cross(y_tcp);

	Eigen::Matrix3d Re;

	Re << x_tcp[0], y_tcp[0], z_tcp[0],
		x_tcp[1], y_tcp[1], z_tcp[1],
		x_tcp[2], y_tcp[2], z_tcp[2];


	//TODO:Test
	Eigen::Matrix4d T;
	T << Re(0, 0), Re(0, 1), Re(0, 2), p2[0],
		Re(1, 0), Re(1, 1), Re(1, 2), p2[1],
		Re(2, 0), Re(2, 1), Re(2, 2), p2[2],
		0, 0, 0, 1;
	/*vtkSmartPointer<vtkMatrix4x4> TcpMatrix = vtkSmartPointer<vtkMatrix4x4>::New();*/
	for (int row = 0; row < 4; ++row)
	{
		for (int col = 0; col < 4; ++col)
		{
			T_FlangerTofinger->SetElement(row, col, T(row, col));
		}
	}

	Eigen::Vector3d eulerAngle = Re.eulerAngles(2, 1, 0);

	//------------------------------------------------
	double tcp[6];
	tcp[0] = p2[0];
	tcp[1] = p2[1];
	tcp[2] = p2[2];
	tcp[3] = eulerAngle(2) * radius2degree; //-0.81;// -0.813428203; // rx
	tcp[4] = eulerAngle(1) * radius2degree; // ry
	tcp[5] = eulerAngle(0) * radius2degree; // rz


	std::string error;
	std::array<double, 6>TCP;
	for (size_t i = 0; i < 6; i++)
	{
		TCP[i] = tcp[i];
	}
	this->Robot->setTCP(TCP, error);

	return true;
}
bool Zzxtest::InterpretImageLine()
{
	//确定目标线：会读取线数据导入到P2和P3中
	auto targetLinePoints = dynamic_cast<mitk::PointSet*>(m_Controls.mitkNodeSelectWidget_imageTargetLine->GetSelectedNode()->GetData());
	auto targetPoint_0 = targetLinePoints->GetPoint(0); // TCP frame origin should move to this point
	auto targetPoint_1 = targetLinePoints->GetPoint(1);
	std::cout << "targetPoint_0: (" << targetPoint_0[0] << ", " << targetPoint_0[1] << ", " << targetPoint_0[2] << ")" << std::endl;
	std::cout << "targetPoint_1: (" << targetPoint_1[0] << ", " << targetPoint_1[1] << ", " << targetPoint_1[2] << ")" << std::endl;


	double targetPointUnderBase_0[3]{ 0 };
	double targetPointUnderBase_1[3]{ 0 };
	vtkSmartPointer<vtkMatrix4x4> vtkT_BaseToImage = vtkSmartPointer<vtkMatrix4x4>::New();;


	QThread::msleep(100);
	QApplication::processEvents();
	updateCameraData();
	//获取机械臂配准矩阵T_BaseToBaseRF
	vtkMatrix4x4* vtkT_BaseToBaseRF = vtkMatrix4x4::New();
	vtkT_BaseToBaseRF->DeepCopy(T_BaseToBaseRF);


	//获取T_BaseRFToCamera
	auto vtkT_BaseRFToCamera = vtkMatrix4x4::New();
	vtkT_BaseRFToCamera->DeepCopy(T_CamToBaseRF);
	vtkT_BaseRFToCamera->Invert();

	//获取T_CameraToPatientRF
	auto vtkT_CameraToPatientRF = vtkMatrix4x4::New();
	vtkT_CameraToPatientRF->DeepCopy(T_CamToPatientRF);

	//获取T_PatientRFToImage
	auto vtkT_PatientRFToImage = vtkMatrix4x4::New();
	vtkT_PatientRFToImage->DeepCopy(T_PatientRFtoImage);

	//计算T_BaseToImage
	vtkNew<vtkTransform> Transform;
	Transform->Identity();
	Transform->PostMultiply();
	Transform->SetMatrix(vtkT_PatientRFToImage);
	Transform->Concatenate(vtkT_CameraToPatientRF);
	Transform->Concatenate(vtkT_BaseRFToCamera);
	Transform->Concatenate(vtkT_BaseToBaseRF);
	Transform->Update();
	vtkT_BaseToImage = Transform->GetMatrix();



	//获取P0点的坐标
	vtkSmartPointer<vtkMatrix4x4> TargetMatrix_0 = vtkSmartPointer<vtkMatrix4x4>::New();
	TargetMatrix_0->SetElement(0, 3, targetPoint_0[0]);
	TargetMatrix_0->SetElement(1, 3, targetPoint_0[1]);
	TargetMatrix_0->SetElement(2, 3, targetPoint_0[2]);

	vtkNew<vtkTransform> Trans;
	Trans->Identity();
	Trans->PostMultiply();
	Trans->SetMatrix(TargetMatrix_0);
	Trans->Concatenate(vtkT_BaseToImage);
	Trans->Update();
	vtkSmartPointer<vtkMatrix4x4>  vtkT_BaseToTarget_0 = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkT_BaseToTarget_0 = Trans->GetMatrix();


	//获取P1点的坐标
	/*auto TargetMatrix_1 = vtkMatrix4x4::New();*/
	vtkSmartPointer<vtkMatrix4x4> TargetMatrix_1 = vtkSmartPointer<vtkMatrix4x4>::New();
	TargetMatrix_1->SetElement(0, 3, targetPoint_1[0]);
	TargetMatrix_1->SetElement(1, 3, targetPoint_1[1]);
	TargetMatrix_1->SetElement(2, 3, targetPoint_1[2]);

	vtkNew<vtkTransform> Trans1;
	Trans1->Identity();
	Trans1->PostMultiply();
	Trans1->SetMatrix(TargetMatrix_1);
	Trans1->Concatenate(vtkT_BaseToImage);
	Trans1->Update();
	vtkSmartPointer<vtkMatrix4x4>  vtkT_BaseToTarget_1 = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkT_BaseToTarget_1 = Trans1->GetMatrix();


	targetPointUnderBase_0[0] = vtkT_BaseToTarget_0->GetElement(0, 3);
	targetPointUnderBase_0[1] = vtkT_BaseToTarget_0->GetElement(1, 3);
	targetPointUnderBase_0[2] = vtkT_BaseToTarget_0->GetElement(2, 3);

	targetPointUnderBase_1[0] = vtkT_BaseToTarget_1->GetElement(0, 3);
	targetPointUnderBase_1[1] = vtkT_BaseToTarget_1->GetElement(1, 3);
	targetPointUnderBase_1[2] = vtkT_BaseToTarget_1->GetElement(2, 3);



	//get surface to target plane matrix建立图像目标点坐标系
	Eigen::Vector3d x;
	x << targetPointUnderBase_1[0] - targetPointUnderBase_0[0],
		targetPointUnderBase_1[1] - targetPointUnderBase_0[1],
		targetPointUnderBase_1[2] - targetPointUnderBase_0[2];
	x.normalize();

	Eigen::Vector3d tmp_z;
	tmp_z << 0,
		0,
		1;


	Eigen::Vector3d y = x.cross(tmp_z);
	y.normalize();

	Eigen::Vector3d z = y.cross(x);
	z.normalize();

	double target[16]
	{
			x[0], y[0], z[0], targetPointUnderBase_0[0],
			x[1], y[1], z[1], targetPointUnderBase_0[1],
			x[2], y[2], z[2], targetPointUnderBase_0[2],
			0,0,0,1
	};
	PrintMatrix("array_surfaceToPlane", target);
	vtkSmartPointer<vtkMatrix4x4> vtkimageSurfaceToPlane = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkimageSurfaceToPlane->DeepCopy(target);



	Eigen::Matrix3d Re;
	Re << target[0], target[1], target[2],   // 第一行元素
		target[4], target[5], target[6],   // 第二行元素
		target[8], target[9], target[10];  // 第三行元素

	Eigen::Vector3d eulerAngle = Re.eulerAngles(2, 1, 0);
	std::array<double, 6> line;
	line[0] = target[3];
	line[1] = target[7];
	line[2] = target[11];
	line[3] = eulerAngle[2] * radius2degree;
	line[4] = eulerAngle[1] * radius2degree;
	line[5] = eulerAngle[0] * radius2degree;
	m_Controls.textBrowser->append("-------------------------------------------------------------------------------------------");
	m_Controls.textBrowser->append("line test  now");
	m_Controls.textBrowser->append("dx=" + QString::number(line[0]));
	m_Controls.textBrowser->append("dy=" + QString::number(line[1]));
	m_Controls.textBrowser->append("dz=" + QString::number(line[2]));
	m_Controls.textBrowser->append("dRx=" + QString::number(line[3]));
	m_Controls.textBrowser->append("dRy=" + QString::number(line[4]));
	m_Controls.textBrowser->append("dRz=" + QString::number(line[5]));
	("-------------------------------------------------------------------------------------------");
	

	std::string error;
	this->Robot->moveP(line, error);


	return true;
}