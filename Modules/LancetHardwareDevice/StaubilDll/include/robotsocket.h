#ifndef ROBOT_SOCKET_H
#define ROBOT_SOCKET_H

#include <QObject>
#include <QTcpSocket>
#include <QThread>
#include <QTimer>

class RobotSocket: public QTcpSocket
{
    Q_OBJECT

public:
    RobotSocket(QObject *parent, qintptr p);
    ~RobotSocket();

    QTimer *m_pTimer;
    QAbstractSocket::SocketState socket_state;

signals:

    void socket_data(QByteArray ba);

public slots:

    void displayError(QAbstractSocket::SocketError e);
    void handleTimeout();

private slots:

    void socketPopData();
    void socketStateChanged(QAbstractSocket::SocketState s);
    void socketWriteData(QString dstr);

};

#endif // ROBOT_SOCKET_H
