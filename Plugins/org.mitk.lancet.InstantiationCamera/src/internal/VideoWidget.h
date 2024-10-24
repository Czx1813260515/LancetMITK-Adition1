#pragma once
#include <qwidget.h>
#include <qimage.h>
#include <qpainter.h>
#include <qabstractvideosurface.h>
class VideoWidget : public QWidget
{
	Q_OBJECT
public:
	explicit VideoWidget(QWidget* parent = nullptr);
	void SetImage(QImage aImage);
	void RenderVideo(char* data, int width, int height);
	//QList<QVideoFrame::PixelFormat> supportedPixelFormats(
	//	QAbstractVideoBuffer::HandleType handleType = QAbstractVideoBuffer::NoHandle) const override;

	//bool present(const QVideoFrame& frame) override;

	//bool start(const QVideoSurfaceFormat& format) override;

	//void stop() override;
protected:
	void paintEvent(QPaintEvent* event) override;
	

private:
	QImage m_Image;
	QWidget* m_Widget;
};

