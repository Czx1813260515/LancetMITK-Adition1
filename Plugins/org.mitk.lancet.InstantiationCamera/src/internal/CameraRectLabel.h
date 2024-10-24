#pragma once
#include <iostream>
#include <qlabel.h>
#include <qevent.h>
#include <qpainter.h>
#include <AbstractCamera.h>
class CameraRectLabel : public QLabel
{
	Q_OBJECT
public:
	explicit CameraRectLabel(QWidget* parent = nullptr);

	void StartDraw();

	void RequestPaint();
protected:
	void mousePressEvent(QMouseEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;

	void mouseReleaseEvent(QMouseEvent* event) override;

	void paintEvent(QPaintEvent* event) override;

private:
	QLabel* m_Label;
	bool m_IsSelecting = false;
	QPoint m_StartPoint;  // 框选的起始点
	QPoint m_EndPoint;    // 框选的结束点
	bool m_SelectionStart = false;
	QRect m_CurrentRect;
	AbstractCamera* m_Camera;
};

