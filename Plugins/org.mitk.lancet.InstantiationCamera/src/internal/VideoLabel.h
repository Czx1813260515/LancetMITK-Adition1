#pragma once
#include <qlabel.h>
#include <qpainter.h>
class VideoLabel :public QLabel
{
	Q_OBJECT
public:
	explicit VideoLabel(QWidget* parent);
	void SetImage(char* data, double aWidth, double aHeight);
protected:
	void paintEvent(QPaintEvent* event) override;

private:
	QImage m_Image;
};

