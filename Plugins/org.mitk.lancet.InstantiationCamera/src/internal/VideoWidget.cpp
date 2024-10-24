#include "VideoWidget.h"
#include <iostream>

VideoWidget::VideoWidget(QWidget* parent) :QWidget(parent)
{
	m_Widget = parent;
}
void VideoWidget::SetImage(QImage aImage)
{
	m_Image = aImage;
}

void VideoWidget::paintEvent(QPaintEvent* event)
{
	Q_UNUSED(event);
	QPainter painter(this);
	painter.fillRect(rect(), Qt::black);
	
	if (!m_Image.isNull()) {
		QImage scaledImage = m_Image.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
		painter.drawImage(rect(), scaledImage);
	}
}

void VideoWidget::RenderVideo(char* data, int width, int height)
{
	QImage image(reinterpret_cast<uchar*>(data), width, height, QImage::Format_Grayscale8);
	m_Image = image;

	update();
}
