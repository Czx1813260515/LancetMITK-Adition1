#include "VideoLabel.h"

VideoLabel::VideoLabel(QWidget* parent) :QLabel(parent)
{

}

void VideoLabel::SetImage(char* data, double aWidth, double aHeight)
{
	QImage image(reinterpret_cast<uchar*>(data), aWidth, aHeight, QImage::Format_Grayscale8);
	m_Image = image;

	update();
}

void VideoLabel::paintEvent(QPaintEvent* event)
{
	Q_UNUSED(event);
	QPainter painter(this);

	painter.fillRect(rect(), Qt::black);

	if (!m_Image.isNull()) {
		//QImage scaledImage = m_Image.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
		//QImage image(reinterpret_cast<uchar*>(m_Image), videoWidth, videoHeight, QImage::Format_Grayscale8);
		painter.drawImage(rect(), m_Image);
	}
}
