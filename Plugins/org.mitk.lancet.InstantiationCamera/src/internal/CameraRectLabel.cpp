#include "CameraRectLabel.h"


CameraRectLabel::CameraRectLabel(QWidget* parent) : QLabel(parent)
{
	setMouseTracking(true);
}

void CameraRectLabel::StartDraw()
{
	m_IsSelecting = true;
	std::cout << "StartDraw m_IsSelecting is true" << std::endl; 
	update();
}

void CameraRectLabel::RequestPaint()
{
	//if (!m_CurrentRect.isNull()) {
	//	QPainter painter(this);
	//	painter.setPen(QPen(Qt::red, 2));  // 使用红色画笔绘制矩形框
	//	painter.drawRect(m_CurrentRect);     // 绘制矩形框
	//}
}

void CameraRectLabel::mousePressEvent(QMouseEvent* event)
{
	if (m_IsSelecting && event->button() == Qt::LeftButton) {
		m_SelectionStart = true;     // 标记框选已开始
		m_StartPoint = event->pos();   // 记录起始点
		m_CurrentRect = QRect();       // 清空上次的矩形
		std::cout << "mousePressEvent" << std::endl;
	}
}

void CameraRectLabel::mouseMoveEvent(QMouseEvent* event)
{
	if (m_IsSelecting && m_SelectionStart) {
		// 更新当前矩形框的大小
		std::cout << "mouseMoveEvent" << std::endl;
		m_CurrentRect = QRect(m_StartPoint, event->pos()).normalized();
		update();  // 刷新界面，触发 paintEvent 重绘
	}
}

void CameraRectLabel::mouseReleaseEvent(QMouseEvent* event)
{
	if (m_IsSelecting && m_SelectionStart && event->button() == Qt::LeftButton) {
		m_SelectionStart = false;  // 框选结束
		m_IsSelecting = false;         // 退出框选模式
		update();                  // 刷新界面显示最终矩形
		std::cout << "mouseReleaseEvent" << std::endl;
	}
}

void CameraRectLabel::paintEvent(QPaintEvent* event)
{
	QLabel::paintEvent(event);
	//std::cout << "paintEvent" << std::endl;
	if (!m_CurrentRect.isNull()) {
		QPainter painter(this);
		painter.setPen(QPen(Qt::red, 2));  // 使用红色画笔绘制矩形框
		painter.drawRect(m_CurrentRect);     // 绘制矩形框
	
	}
}
