/*============================================================================

The Medical Imaging Interaction Toolkit (MITK)

Copyright (c) German Cancer Research Center (DKFZ)
All rights reserved.

Use of this source code is governed by a 3-clause BSD license that can be
found in the LICENSE file.

============================================================================*/


#ifndef Zzxtest_h
#define Zzxtest_h

#include <berryISelectionListener.h>

#include <QmitkAbstractView.h>

#include "ui_ZzxtestControls.h"




//
#include "windows.h"
#include "direct.h"
#include "io.h"
#include <fstream>
#include <string>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <thread>
#include <mutex>
#include "AimPositionAPI.h"
#include "AimPositionDef.h"
#include <iostream>
#include <string>
#include <vector>
#include<qinputdialog.h>
#include<qtimer.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include<qdatetime.h>

//
#include<mitkMatrixConvert.h>
#include<Transform.h>
#include<eigen3/Eigen/src/Geometry/Transform.h>
#include <itkAffineTransform.h>
#include<vtkMatrix4x4.h>
//
#define GET_DATA_FROM_HARDWARE 1//aim
//
#include"Robot_Hans.h"
#include<memory>
#include <QFileDialog>
#include "robotRegistration.h"
#include<QtConcurrent>
#include <QmitkSingleNodeSelectionWidget.h>
//mitk image
#include <mitkImage.h>
#include "mitkPointSet.h"
#include "mitkNodePredicateAnd.h"
#include "mitkNodePredicateDataType.h"
#include "mitkNodePredicateNot.h"
#include "mitkNodePredicateOr.h"
#include "mitkNodePredicateProperty.h"
#include "mitkImageToSurfaceFilter.h"
#include<vtkLandmarkTransform.h>
#include<surfaceregistraion.h>
#include<vtkTransformFilter.h>
#include<vtkImplicitPolyDataDistance.h>

class Zzxtest : public QmitkAbstractView
{
  // this is needed for all Qt objects that should have a Qt meta-object
  // (everything that derives from QObject and wants to have signal/slots)
  Q_OBJECT

public:
  static const std::string VIEW_ID;
  

protected:
  virtual void CreateQtPartControl(QWidget *parent) override;

  virtual void SetFocus() override;



  Ui::ZzxtestControls m_Controls;


public:
	//tab1
	
//函数
protected:
	//unique_ptr<Robot_Hans> Robot;
	Robot_Hans* Robot;
	void initSlot();
	T_AimToolDataResult* GetNewToolData();
	void UpdateCameraToToolMatrix(T_AimToolDataResult* ToolData, const char* Name, double* aCamera2Tool, QLabel* label);
	void CombineRotationTranslation(float Rto[3][3], float Tto[3], vtkMatrix4x4* resultMatrix);
	void open_QTimercheck_IsMoving();
	//
	void PrintArray16ToMatrix(const std::string& title, double* Array);
	void Print_Matrix(const std::string& title, vtkMatrix4x4* matrix);

	//
	void textEditProcess(std::array<double,6>&Pose, QLineEdit* LineEdit);
	//
	void calcute_automove_list(const std::array<double, 6> P,int Distance,double degree);
	//
	void InitSurfaceSelector(QmitkSingleNodeSelectionWidget* widget);
	void InitPointSetSelector(QmitkSingleNodeSelectionWidget* widget);
	vtkSmartPointer<vtkMatrix4x4> ComputeTransformMartix(mitk::PointSet::Pointer points_set1, mitk::PointSet::Pointer points_set2);
	double Zzxtest::GetRegisrationRMS(mitk::PointSet* points, mitk::Surface* surface, vtkMatrix4x4* matrix);
	void computeAverageTransform(vtkMatrix4x4* samples[], vtkMatrix4x4* average);
	void PrintMatrix(std::string matrixName, double* matrix);

public slots:
	//tab1
	void powerOn();
	void powerOff();
	void PrintToolMatrix();
	void connectCamera();
	void openCameraQtTimer();
	void SuddenStop();
	void connectArm();
	void check_IsMoving();
	void updateCameraData();

	//positionAccuracy
	void positionAccuracy();
	//

	void onComboBoxIndexChanged(int index);
	void xp();
	void yp();
	void zp();
	void xm();
	void ym();
	void zm();
	void rxp();
	void ryp();
	void rzp();
	void rxm();
	void rym();
	void rzm();
	//
	void openIsMoing();
	void CloseIsMoving();
	void setTCPToFlange();
	void configTCP2Flange();
	void setInitialPoint();
	void goToInitial();
	void replaceRegistration();
	void saveArmMatrix();
	void reuseArmMatrix();
	void captureRobot();
	void CapturePose(bool translationOnly);
	void JiontAngleMotion();
	void onComboBox_moveIndexchange(int index);
	void AutoMoveJ();

	//精度测试
	void ResetImageConfiguration();
	void saveImageMatrix();
	void reuseImageMatrix();
	void collectLandmark();
	void landmarkRegistration();
	void collectICP();
	void ICPRegistration();
	//走线走面
	bool  SetPlanePrecisionTestTcp();
	bool InterpretImagePlane();
	void On_pushButton_goToFakePlane_clicked();
	void OnAutoPositionStart();
	bool SetPrecisionTestTcp();
	bool InterpretImageLine();
//变量
private:
	E_ReturnValue rlt;
	AimHandle aimHandle = NULL;
	QTimer* m_AimoeVisualizeTimer{ nullptr };//相机拿数据的QT定时器
	T_MarkerInfo markerSt;
	T_AimPosStatusInfo statusSt;

	QTimer* QTimer_IsMoving{ nullptr };
	bool movingstatue;
	bool lastMovingstatue=false;

	//坐标选择默认值
	int nToolMotion=1;
	//初始点（关节角度值）
	std::array<double, 6>Init_JointPose;
	//初始化机械臂配准的类
	RobotRegistration m_RobotRegistration;
	//计数的全局变量
	unsigned int m_IndexOfRobotCapture{ 0 };
	//运动模式默认值
	int moveType = 0;
	//自动移动计数
	int auto_move_index = 0;
	std::map<std::string, std::array<double, 6>> automove_target_map;
	std::array<double, 6> automove_target_init = { 0, 0, 0, 0, 0, 0 };
	const double radius2degree = 57.29578049044297;

//数据
private:
	double ProbeTop[4] = { 0.0f, 0.0f, 0.0f, 1.0f };//在相机坐标系下的探针尖端
	//用采集数据的时候转VTK矩阵使用
	float R_tran[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
	float t_tran[3] = { 0.0f, 0.0f, 0.0f };

	//相机到机械臂末端Mark
	float R_CamToEndRF[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
	float t_CamToEndRF[3] = { 0.0f, 0.0f, 0.0f };
	double T_CamToEndRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

	//相机到机械基座Mark
	float R_CamToBaseRF[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
	float t_CamToBaseRF[3] = { 0.0f, 0.0f, 0.0f };
	double T_CamToBaseRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	//相机到病人Mark
	float R_CamToPatientRF[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
	float t_CamToPatientRF[3] = { 0.0f, 0.0f, 0.0f };
	double T_CamToPatientRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	//相机到探针的Mark
	float R_CamToProbe[3][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f} };
	float t_CamToProbe[3] = { 0.0f, 0.0f, 0.0f };
	double T_CamToProbe[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

	int callCount = 0; // 添加一个静态变量来记录函数调用次数
		//机械臂配准数据
	double T_BaseToBaseRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	double T_FlangeToEndRF[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

	//
	unsigned int m_IndexOfLandmark{ 0 };
	unsigned int m_IndexOfICP{ 0 };
	vtkSmartPointer<vtkPoints> vtkProbeTip_onObjRf = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkPoints> vtkProbeTip_onObjRf_icp = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkMatrix4x4> T_PatientRFToProbeTip = vtkSmartPointer<vtkMatrix4x4>::New();
	double T_PatientRFtoImage[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	double T_ImageToImage_icp[16]{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
	double nd_tip_FpatientRF[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
	double T_PatientRFToProbe[16]{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
	double nd_tip_FImage_icp[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
	mitk::DataNode* m_IcpSourceSurface{ nullptr };
	vtkSmartPointer<vtkMatrix4x4> T_FlangerTofinger = vtkSmartPointer<vtkMatrix4x4>::New();
	vtkSmartPointer<vtkMatrix4x4> vtkBaseToTargetPlaneTransform = vtkSmartPointer<vtkMatrix4x4>::New();
	//
	QThread* robotThread;
};

#endif 
