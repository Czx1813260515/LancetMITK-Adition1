#pragma once
#include <qwidget.h>
#include <ui_DianaSevenControls.h>
#include "SystemPrecision.h"
#include "FileIO.h"
#include <QmitkSingleNodeSelectionWidget.h>
#include <mitkNodePredicateAnd.h>
#include <mitkNodePredicateNot.h>
#include <mitkNodePredicateProperty.h>
#include <mitkNodePredicateCompositeBase.h>
#include <mitkSurface.h>
#include <mitkNodePredicateBase.h>
#include <mitkNodePredicateDataType.h>
#include <mitkNodePredicateOr.h>
#include <mitkColorProperty.h>

class PrecisionTab :  public QWidget//, public Ui_DianaSevenControls
{
	Q_OBJECT
public:
	explicit PrecisionTab(Ui::DianaSevenControls ui, mitk::DataStorage* aDataStorage, DianaRobot* aRobot, AimCamera* aCamera, LancetRobotRegistration* aLancetRobReg, QWidget* parent = nullptr);

public slots:
	void InitConnection();
	void DisplayPrecisionToolBtnClicked();
	void DisplayProbeBtnClicked();
	void SetGoLineTCPBtnClicked();
	void SetGoPlaneTCPBtnClicked();
	void CollectLandmarkBtnClicked();
	void LandmarkRegistrationBtnClicked();
	void CollectICPBtnClicked();
	void ICPRegistrationBtnClicked();
	void AddICPBtnClicked();
	void UpdateDeviationBtnClicked();
	void ResetImageRegBtnClicked();
	void SaveImageRegBtnClicked();
	void ReuseImageRegBtnClicked();
	void DisplayTCPAxesActorBtnClicked();
	void SetIRenderWindowPart(mitk::IRenderWindowPart* aPart);
	void DisplayTCPInRFBtnClicked();
	void DisplayFlangeAxesActorBtnClicked();
	void PrintTCPInCameraBtnClicked();
	void GoLineByRobotBtnClicked();

private:
	void InitSurfaceSelector(mitk::DataStorage* dataStorage, QmitkSingleNodeSelectionWidget* widget);


	void InitImageSelector(mitk::DataStorage* dataStorage, QmitkSingleNodeSelectionWidget* widget);


	void InitPointSetSelector(mitk::DataStorage* dataStorage, QmitkSingleNodeSelectionWidget* widget);

private:
	QWidget* m_TabPage;
	Ui::DianaSevenControls m_ui;
	DianaRobot* m_DianaSevenRobot;
	AimCamera* m_AimCamera;
	LancetRobotRegistration* m_LancetRobReg;

	lancetAlgorithm::SystemPrecision* m_SystemPrecision;

	mitk::IRenderWindowPart* m_IRenderWindowPart;
	mitk::DataStorage* m_dataStorage;

};

