#pragma once
#include <qwidget.h>
#include <qobject.h>
#include "ui_HansRobotControls.h"
#include "LancetHansRobot.h"
#include "LancetDianaRobot.h"
#include "FileIO.h"
#include "LancetAimCamera.h"

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

class ConnectionTab : public QWidget
{

	Q_OBJECT
public:
	explicit ConnectionTab(Ui::HansRobotControls ui, mitk::DataStorage* aDataStorage, AbstractRobot* aRobot, AbstractCamera* aCamera, QWidget* parent = nullptr);

public slots:
	void InitConnection();
	void connectRobot();
	void connectCamera();
	void updateData();
	void powerOff();
	void powerOn();
	void upDateUi();

private:


private:
	QWidget* m_TabPage;
	Ui::HansRobotControls m_ui;
	AbstractRobot * m_Robot;
	mitk::DataStorage* m_dataStorage;
	AbstractCamera* m_Camera;

	void flashLable(Eigen::Vector3d tempTip, QLabel* label);
};