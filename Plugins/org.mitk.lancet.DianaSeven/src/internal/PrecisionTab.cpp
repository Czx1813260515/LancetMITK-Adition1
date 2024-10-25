#include "PrecisionTab.h"

PrecisionTab::PrecisionTab(Ui::DianaSevenControls ui,mitk::DataStorage* aDataStorage,
	DianaRobot* aRobot, AimCamera* aCamera, LancetRobotRegistration* aLancetRobReg, QWidget*  parent) : QWidget(parent)
{
	m_ui = ui;
	m_DianaSevenRobot = aRobot;
	m_AimCamera = aCamera;
	m_LancetRobReg = aLancetRobReg;
	m_dataStorage = aDataStorage;
	//m_IRenderWindowPart = aIRenderWindowPart;
	if (!m_dataStorage)
	{
		std::cout << "m_dataStorage is nullptr" << std::endl;
	}
	
	InitSurfaceSelector(m_dataStorage, m_ui.SurfaceRegSelectWidget);
	InitPointSetSelector(m_dataStorage, m_ui.LandmarkPointRegSelectWidget);
	InitPointSetSelector(m_dataStorage, m_ui.ImageTargetLineSelectWidget);
	InitConnection();
}



void PrecisionTab::InitConnection()
{
	connect(m_ui.DisplayPrecisionToolBtn, &QPushButton::clicked, this, &PrecisionTab::DisplayPrecisionToolBtnClicked);
	connect(m_ui.DisplayProbeBtn, &QPushButton::clicked, this, &PrecisionTab::DisplayProbeBtnClicked);
	connect(m_ui.SetGoLineTCPBtn, &QPushButton::clicked, this, &PrecisionTab::SetGoLineTCPBtnClicked);
	connect(m_ui.GoLineBtn, &QPushButton::clicked, this, [this]() {
		auto targetLinePoints = dynamic_cast<mitk::PointSet*>(m_ui.ImageTargetLineSelectWidget->GetSelectedNode()->GetData());
		auto targetPoint_0 = targetLinePoints->GetPoint(0); // TCP frame origin should move to this point
		auto targetPoint_1 = targetLinePoints->GetPoint(1);
		Eigen::Vector3d point0(targetPoint_0[0], targetPoint_0[1], targetPoint_0[2]);
		Eigen::Vector3d point1(targetPoint_1[0], targetPoint_1[1], targetPoint_1[2]);
		m_SystemPrecision->GoLine(point0, point1); });
	connect(m_ui.GoPlaneBtn, &QPushButton::clicked, this, [this]() {
		auto targetLinePoints = dynamic_cast<mitk::PointSet*>(m_ui.ImageTargetLineSelectWidget->GetSelectedNode()->GetData());
		if (targetLinePoints->GetSize() != 3)
		{
			std::cout << "Select Wrong Points" << std::endl;
			return;
		}
		auto targetPoint_0 = targetLinePoints->GetPoint(0); // TCP frame origin should move to this point
		auto targetPoint_1 = targetLinePoints->GetPoint(1);
		auto targetPoint_2 = targetLinePoints->GetPoint(2);
		Eigen::Vector3d point0(targetPoint_0[0], targetPoint_0[1], targetPoint_0[2]);
		Eigen::Vector3d point1(targetPoint_1[0], targetPoint_1[1], targetPoint_1[2]);
		Eigen::Vector3d point2(targetPoint_2[0], targetPoint_2[1], targetPoint_2[2]);
		m_SystemPrecision->GoPlane(point0, point1, point2); });
	connect(m_ui.CollectLandmarkBtn, &QPushButton::clicked, this, &PrecisionTab::CollectLandmarkBtnClicked);
	connect(m_ui.LandmarkRegistrationBtn, &QPushButton::clicked, this, &PrecisionTab::LandmarkRegistrationBtnClicked);
	connect(m_ui.CollectICPBtn, &QPushButton::clicked, this, &PrecisionTab::CollectICPBtnClicked);
	connect(m_ui.ICPRegistrationBtn, &QPushButton::clicked, this, &PrecisionTab::ICPRegistrationBtnClicked);
	connect(m_ui.UpdateDeviationBtn, &QPushButton::clicked, this, &PrecisionTab::UpdateDeviationBtnClicked);
	connect(m_ui.ResetImageRegBtn, &QPushButton::clicked, this, &PrecisionTab::ResetImageRegBtnClicked);
	connect(m_ui.SaveImageRegBtn, &QPushButton::clicked, this, &PrecisionTab::SaveImageRegBtnClicked);
	connect(m_ui.ReuseImageRegBtn, &QPushButton::clicked, this, &PrecisionTab::ReuseImageRegBtnClicked);
	connect(m_ui.SetGoPlaneTCPBtn, &QPushButton::clicked, this, &PrecisionTab::SetGoPlaneTCPBtnClicked);
	connect(m_ui.DisplayTCPAxesActorBtn, &QPushButton::clicked, this, &PrecisionTab::DisplayTCPAxesActorBtnClicked);
	connect(m_ui.DisplayTCPInRFBtn, &QPushButton::clicked, this, &PrecisionTab::DisplayTCPInRFBtnClicked);
	connect(m_ui.DisplayFlangeAxesActorBtn, &QPushButton::clicked, this, &PrecisionTab::DisplayFlangeAxesActorBtnClicked);
	connect(m_ui.PrintTCPInCameraBtn, &QPushButton::clicked, this, &PrecisionTab::PrintTCPInCameraBtnClicked);
	connect(m_ui.GoLineByRobotBtn, &QPushButton::clicked, this, &PrecisionTab::GoLineByRobotBtnClicked);
}

void PrecisionTab::DisplayPrecisionToolBtnClicked()
{
	std::cout << "DisplayPrecisionToolBtn Clicked" << std::endl;
	m_SystemPrecision->DisplayPrecisionTool();
}

void PrecisionTab::DisplayProbeBtnClicked()
{
	m_SystemPrecision->DisplayProbe();
}

void PrecisionTab::SetGoLineTCPBtnClicked()
{
	m_SystemPrecision->SetGoLinePrecisionTCP();
}

void PrecisionTab::SetGoPlaneTCPBtnClicked()
{
	m_SystemPrecision->SetGoPlanePrecisionTCP();
}

void PrecisionTab::CollectLandmarkBtnClicked()
{
	m_ui.CollectLandmarkLineEdit->setText(QString::number(m_SystemPrecision->CollectLandMarkPoints()));
}

void PrecisionTab::LandmarkRegistrationBtnClicked()
{
	m_SystemPrecision->LandmarkRegistration(dynamic_cast<mitk::PointSet*>(m_ui.LandmarkPointRegSelectWidget->GetSelectedNode()->GetData()));
}

void PrecisionTab::CollectICPBtnClicked()
{
	m_ui.CollectedICPLineEdit->setText(QString::number(m_SystemPrecision->CollectICPPoints()));
}

void PrecisionTab::ICPRegistrationBtnClicked()
{
	m_SystemPrecision->ICPRegistration(dynamic_cast<mitk::Surface*>(m_ui.SurfaceRegSelectWidget->GetSelectedNode()->GetData()));
}

void PrecisionTab::AddICPBtnClicked()
{
	
}

void PrecisionTab::UpdateDeviationBtnClicked()
{
	m_SystemPrecision->UpdateDeviation();
}

void PrecisionTab::ResetImageRegBtnClicked()
{
	m_SystemPrecision->Reset();
	m_ui.CollectLandmarkLineEdit->setText(QString::number(0));
	m_ui.CollectedICPLineEdit->setText(QString::number(0));
}

void PrecisionTab::SaveImageRegBtnClicked()
{
	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;
	std::string boxRFToBoxFileName = "T_BoxRFToBox.txt";

	FileIO::SaveMatrix2File(FileIO::CombinePath(filename.toStdString(), boxRFToBoxFileName).string(), m_SystemPrecision->GetBoxRF2BoxMatrix());
}

void PrecisionTab::ReuseImageRegBtnClicked()
{
	QString filename = QFileDialog::getExistingDirectory(nullptr, "Select the Tools store folder", "");
	if (filename.isNull()) return;
	std::string boxRFToBoxFileName = "T_BoxRFToBox.txt";
	std::string path =  FileIO::CombinePath(filename.toStdString(), boxRFToBoxFileName).string();

	vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
	FileIO::ReadTextFileAsvtkMatrix(path, matrix);
	PrintDataHelper::CoutMatrix("ReuseImageRegBtnClicked", matrix);
	m_SystemPrecision->SetBoxRF2BoxMatrix(matrix);
}

void PrecisionTab::DisplayTCPAxesActorBtnClicked()
{
	std::cout << "DisplayTCPAxesActorBtnClicked" << std::endl;
	m_SystemPrecision->DisplayTCPActor();
}

void PrecisionTab::SetIRenderWindowPart(mitk::IRenderWindowPart* aPart)
{
	m_IRenderWindowPart = aPart;
	m_SystemPrecision = new lancetAlgorithm::SystemPrecision(m_dataStorage, m_DianaSevenRobot, m_AimCamera, m_LancetRobReg, m_IRenderWindowPart);
}

void PrecisionTab::DisplayTCPInRFBtnClicked()
{
	m_SystemPrecision->DisplayTCPInRF();
}

void PrecisionTab::DisplayFlangeAxesActorBtnClicked()
{
	m_SystemPrecision->DisplayFlangeAxesActor();
}

void PrecisionTab::PrintTCPInCameraBtnClicked()
{
	m_SystemPrecision->PrintTCPInCamera(m_ui.textBrowser);
}

void PrecisionTab::GoLineByRobotBtnClicked()
{
	auto targetLinePoints = dynamic_cast<mitk::PointSet*>(m_ui.ImageTargetLineSelectWidget->GetSelectedNode()->GetData());
	auto targetPoint_0 = targetLinePoints->GetPoint(0); // TCP frame origin should move to this point
	auto targetPoint_1 = targetLinePoints->GetPoint(1);
	Eigen::Vector3d point0(targetPoint_0[0], targetPoint_0[1], targetPoint_0[2]);
	Eigen::Vector3d point1(targetPoint_1[0], targetPoint_1[1], targetPoint_1[2]);
	m_SystemPrecision->GoLineByRobot(point0, point1);
}

void PrecisionTab::InitSurfaceSelector(mitk::DataStorage* dataStorage, QmitkSingleNodeSelectionWidget* widget)
{
	widget->SetDataStorage(dataStorage);
	widget->SetNodePredicate(mitk::NodePredicateAnd::New(
		mitk::TNodePredicateDataType<mitk::Surface>::New(),
		mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(mitk::NodePredicateProperty::New("helper object"),
			mitk::NodePredicateProperty::New("hidden object")))));

	widget->SetSelectionIsOptional(true);
	widget->SetAutoSelectNewNodes(true);
	widget->SetEmptyInfo(QString("Please select a surface"));
	widget->SetPopUpTitel(QString("Select surface"));
}

void PrecisionTab::InitImageSelector(mitk::DataStorage* dataStorage, QmitkSingleNodeSelectionWidget* widget)
{
	widget->SetDataStorage(dataStorage);
	widget->SetNodePredicate(mitk::NodePredicateAnd::New(
		mitk::TNodePredicateDataType<mitk::Image>::New(),
		mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(mitk::NodePredicateProperty::New("helper object"),
			mitk::NodePredicateProperty::New("hidden object")))));

	widget->SetSelectionIsOptional(true);
	widget->SetAutoSelectNewNodes(true);
	widget->SetEmptyInfo(QString("Please select a surface"));
	widget->SetPopUpTitel(QString("Select surface"));
}

void PrecisionTab::InitPointSetSelector(mitk::DataStorage* dataStorage, QmitkSingleNodeSelectionWidget* widget)
{
	widget->SetDataStorage(dataStorage);
	widget->SetNodePredicate(mitk::NodePredicateAnd::New(
		mitk::TNodePredicateDataType<mitk::PointSet>::New(),
		mitk::NodePredicateNot::New(mitk::NodePredicateOr::New(mitk::NodePredicateProperty::New("helper object"),
			mitk::NodePredicateProperty::New("hidden object")))));

	widget->SetSelectionIsOptional(true);
	widget->SetAutoSelectNewNodes(true);
	widget->SetEmptyInfo(QString("Please select a point set"));
	widget->SetPopUpTitel(QString("Select point set"));
}
