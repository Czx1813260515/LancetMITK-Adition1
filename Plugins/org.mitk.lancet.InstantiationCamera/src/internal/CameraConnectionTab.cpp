#include "CameraConnectionTab.h"

CameraConnectionTab::CameraConnectionTab(Ui::InstantiationCameraControls ui, mitk::DataStorage* aDataStorage,
	AriemediCamera* aAriemediCamera, QWidget* parent)
{
	m_UI = ui;
	m_DataStorage = aDataStorage;
	m_Camera = aAriemediCamera;
	//m_UI.RightImageLabel->setMouseTracking(true);
	//m_UI.RightImageLabel->setScaledContents(true);
	//m_UI.RightImageWidget->setScaleContents(true);
	//m_CameraRectLabel = new CameraRectLabel(m_UI.RightImageLabel);

	m_RightVideoWidget = new VideoWidget(m_UI.RightImageWidget);
	//m_UI.LeftImageLabel->setAttribute(Qt::WA_TransparentForMouseEvents, true);
	//m_UI.LeftImageLabel->setMouseTracking(true);
	InitConnection();
}

void CameraConnectionTab::InitConnection()
{
	connect(m_UI.CameraConnectBtn, &QPushButton::clicked, this, &CameraConnectionTab::CameraConnectBtnClicked);
	connect(m_UI.CameraDisconnectBtn, &QPushButton::clicked, this, &CameraConnectionTab::CameraDisconnectBtnClicked);
	connect(m_UI.CameraStartBtn, &QPushButton::clicked, this, &CameraConnectionTab::CameraStartBtnClicked);
	connect(m_UI.DrawRectBtn, &QPushButton::clicked, this, &CameraConnectionTab::DrawRectBtnClicked);

	connect(m_Camera, &AriemediCamera::CameraUpdateClock, this, &CameraConnectionTab::UpdateUIDisplay);
	connect(m_Camera, &AriemediCamera::ImageUpdateClock, this, &CameraConnectionTab::UpdateUIImages);
}

void CameraConnectionTab::CameraConnectBtnClicked()
{
	m_Camera->Connect();
}

void CameraConnectionTab::CameraDisconnectBtnClicked()
{
	m_Camera->Disconnect();
}

void CameraConnectionTab::CameraStartBtnClicked()
{
	std::vector<std::string> toolsName = { "PKADrill" };
	m_Camera->InitToolsName(toolsName);
	m_Camera->Start();
	auto imageSize = m_Camera->GetImageSize();
	m_ImageWidth = imageSize.first;
	m_ImageHeight = imageSize.second;
	//m_UI.LeftImageLabel->setFixedSize(m_ImageWidth/2.0, m_ImageHeight/2.0);
	//m_UI.RightImageLabel->setFixedSize(m_ImageWidth, m_ImageHeight);
	//m_UI.RightImageLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_UI.RightImageWidget->setFixedSize(m_ImageWidth, m_ImageHeight);
	m_UI.RightImageWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	m_Camera->SetAreaDisplay(10, 20, 20, 40);
	std::cout << "m_ImageWidth: " << m_ImageWidth << std::endl;
	std::cout << "m_ImageHeight: " << m_ImageHeight << std::endl;
}

void CameraConnectionTab::DrawRectBtnClicked()
{
	m_CameraRectLabel->StartDraw();
}

void CameraConnectionTab::UpdateUIDisplay()
{
	//std::cout << "catch signals" << std::endl;
	UpdateUIToolsData();
	//UpdateUIImages();
}

void CameraConnectionTab::UpdateUIToolsData()
{
	//PrintDataHelper::CoutArray(m_Camera->GetToolTipByName("PKADrill"), "drill");
	GetAndUpdateToolTip(m_Camera->GetToolTipByName("PKADrill"), m_UI.label_2);
}

void CameraConnectionTab::UpdateUIImages()
{
	//auto images = m_Camera->GetImageData();
	//UpdateSingleImage(images.first, m_UI.LeftImageLabel);
	//UpdateSingleImage(m_Camera->GetRightImage(), m_UI.RightImageLabel);
	m_RightVideoWidget->RenderVideo(m_Camera->GetRightImage(), m_ImageWidth, m_ImageHeight);
}

void CameraConnectionTab::InitUI()
{
	m_UI.label->setText("Probe");
}

void CameraConnectionTab::GetAndUpdateToolTip(Eigen::Vector3d tempTip, QLabel* label)
{
	if (tempTip[0] == 0 && tempTip[1] == 0 && tempTip[2] == 0)
	{
		if (label != nullptr)
		{
			QString str = "x:0  y:0 z:0";
			label->setText(str);
			/*std::cout << str << std::endl;*/
			label->setStyleSheet("QLabel { color : red; }");
		}
	}
	else
	{
		if (label != nullptr)
		{
			QString str = "x:" + QString::number(tempTip[0]) + " "
				+ "y:" + QString::number(tempTip[1]) + " "
				+ "z:" + QString::number(tempTip[2]);
			label->setText(str);
			label->setStyleSheet("QLabel { color : green; }");
			/*std::cout << str << std::endl;*/

		}
	}

}

void CameraConnectionTab::UpdateSingleImage(char* aImage, QLabel* aLabel)
{
	QImage image(reinterpret_cast<uchar*>(aImage), m_ImageWidth, m_ImageHeight, QImage::Format_Grayscale8);

	aLabel->setPixmap(QPixmap::fromImage(image));
	//std::cout << "UpdateSingleImage" << std::endl;
}


