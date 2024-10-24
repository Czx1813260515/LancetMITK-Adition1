#pragma once
#include "ui_InstantiationCameraControls.h"
#include <AriemediCamera.h>
#include <FileIO.h>
#include <qsettings.h>
#include <VideoLabel.h>

class CameraTransTab: public QWidget
{
	Q_OBJECT
public:
	explicit CameraTransTab(Ui::InstantiationCameraControls ui, AriemediCamera* aAriemediCamera, QWidget* parent = nullptr);
	void InitConnection();
	void ConverRomFileBtnClicked();
	void OnSaveARomPathLineEditTextChanged();

	void SelectARomPathBtnClicked();
	void UpdateImage();
private:
	void LoadData();
private:
	Ui::InstantiationCameraControls m_UI;
	AriemediCamera* m_Camera;
	VideoLabel* m_LeftVideoLabel;
	VideoLabel* m_RightVideoLabel;
};

