﻿#ifndef LABSTRACTDEVICEINTERFACE_H
#define LABSTRACTDEVICEINTERFACE_H

#include <QMap>
#include <QObject>
#include <QVariant>
#include <LToolAttitudeMessage>
#include <MitkLancetStaubliRobotExports.h>
/**
 * \inherits QObject
 * \class LAbstractDeviceInterface
 *
 * \ingroup Interface
 *
 * \brief   Abstract device interface class, the extension component mainly
 *          through the encapsulation and extension of the interface to achieve
 *          the purpose, this class is also the connection channel between the
 *          component registration machine and the component.
 *
 * \note    Maybe you want to know： LPrivateAssemblyData
 *
 * \author dabai
 *
 * \version 1.0.0
 *
 * \date 2021/03/29
 *
 * Contact: user@company.com
 *
 */
/*@{*/
class MITKLANCETSTAUBLIROBOT_EXPORT LAbstractDeviceInterface : public QObject
{
    Q_OBJECT
public:
    explicit LAbstractDeviceInterface(QObject* parent = nullptr);
    virtual ~LAbstractDeviceInterface();

    enum WorkerState
    {
        unknown,                ///<
        off_line,               ///<
        leave_unused,           ///<
        working,                ///<
        downtime,               ///<
    };
signals:

    /**
     * \brief   Abnormal signal of equipment components.
     *
     *          All the exceptions generated by the component during operation
     *          will be transmitted to the external knowledge by the change signal.
     *
     *          No exception record will be recorded in this component. This
     *          information will be passed to the outside through the interface.
     *
     * \see     class LErrorMessage. void deviceDowntime(LAbstractDeviceInterface*)
     */
    //void deviceException(LErrorMessage);

    /**
     * \brief   Component downtime signal.
     *
     *          When the component encounters a fatal situation, the component
     *          will send the signal to the outside as much as possible.
     *
     * \param   The component instance object.
     *
     * \bug     Version 1.0.0 is the initial version, and the component may not
     *          send the signal when it is down.
     *
     * \see     class LAbstractDeviceInterface.
     */
    void deviceDowntime(LAbstractDeviceInterface*);

    /**
     * \brief   Component enable signal.
     *
     *          This signal is emitted when the component is enabled (starts to
     *          work). Then, the component will actively push the tool pose
     *          information
     *
     * \param   Component name.
     *
     * \see     void deviceDisable(QString); void updateToolPosition(LToolAttitudeMessage);
     */
    void deviceEnabled(QString);

    /**
     * \brief   Disable component signal.
     *
     *          When the component signal is disabled, the status of the component
     *          is workerstate:: leave_ unused. Then, the component will stop
     *          actively pushing tool pose information.
     *
     * \param   Component name.
     *
     * \see     void deviceDisable(QString); void updateToolPosition(LToolAttitudeMessage);
     */
    void deviceDisable(QString);

    /**
     * \brief   Update tool pose information signal.
     *
     *          This signal is an event actively transmitted to the target after
     *          the component is enabled.
     *
     * \param   Tool pose information.
     *
     * \see     class LToolAttitudeMessage.
     */
    void updateToolPosition(QString, LToolAttitudeMessage);

    /**
     * \brief   Tool enable signal.
     *
     *          The signal is actively pushed.
     *          Only when the tool is enabled will the tool pose update signal
     *          be pushed.
     *
     * \see     void updateToolPosition(LToolAttitudeMessage)
     */
    void noticeToolEnabled(QString);

    /**
     * \brief   Tool disable signal.
     *
     *          The signal is actively pushed.
     *          Only when the tool is enabled will the tool pose update signal
     *          be pushed.
     *
     * \see     void toolEnabled(QString), void updateToolPosition(LToolAttitudeMessage)
     */
    void noticeToolDisable(QString);
public:
    virtual QString devicename() const;
    virtual void setDeviceName(const QString&);

    /**
     * \brief   Returns whether the device object is valid.
     * \return  True: valid; False: invalid;
     */
    virtual bool isValid() const;

    /**
     * \brief   Enable component.
     *          Active push tracking entity data.
     *
     * \see     bool isEnabled(), void disable()
     */
    virtual void enabled() = 0;
    virtual void enabledOnThread() = 0;
    virtual bool isEnabledOnThread() const = 0;

    /**
     * \brief   Returns whether the component is enabled.
     */
    virtual bool isEnabled() const;

    /**
     * \brief   Disable component.
     *          Prohibit pushing tracking entity data.
     */
    virtual void disable() = 0;

    /**
     * \brief   Returns whether the component is disable.
     */
    virtual bool isDisable() const;

    /**
     * \brief   Returns whether the target tool is valid.
     */
    virtual bool isValidTool(const QString&) const = 0;

    /**
     * \brief   Return to device name string.
     */
    virtual QString nameString() const = 0;

    /**
     * \brief   Returns the version number of the device component.
     */
    virtual QString version() const = 0;

    /**
     * \brief   Returns the TCP connection address of the device.
     * \see void setIpAddress(const QString&)
     */
    virtual QString ipAddress() const;

    /**
     * \brief   Set the TCP connection address of the device.
     *
     * \param[in] ipv4:[255.255.255.255]
     * \see QString ipAddress() const
     */
    virtual void setIpAddress(const QString&);

    /**
     * \brief   Return device TCP target port.
     * \see void setPort(const QString&)
     */
    virtual QString port() const;

    /**
     * \brief   Set device TCP target port.
     *
     * \param[in] 1-65535.
     * \see QString port() const
     */
    virtual void setPort(const QString&);

    /**
     * \brief   Return communication information.
     *
     * \retval  [255.255.255.255:port]
     * \see QString ipAddress() const, QString port() const
     */
    virtual QString hostname() const;

    /**
     * \brief   Set communication information.
     *
     * \retval  [255.255.255.255:port]
     * \see QString hostname() const
     */
    virtual void setHostname(const QString&);

    /**
     * \brief   Returns the ROM file set.
     *
     * \see void appendRomFile(const QString&, const QString&)
     */
    virtual QMap<QString, QString> romfiles() const;

    /**
     * \brief   Append ROM file to set.
     *
     * \param[in] tool name.
     * \param[in] rom file.
     *
     * \see void removeRomFile(const QString&), QMap<QString, QString> romfiles() const
     */
    virtual void appendRomFile(const QString&, const QString&);

    /**
     * \brief   Remove ROM file to set.
     *
     * \param[in] tool name.
     */
    virtual void removeRomFile(const QString&);

    /**
     * \brief   Update the ROM file path property value in the collection.
     *
     * \param[in] tool name.
     * \param[in] rom file.
     */
    virtual void updateRomFile(const QString&, const QString&);

    /**
     * \brief   Find the ROM file path of the target tool.
     *
     * \param[in] tool name.
     */
    virtual QString findRomFile(const QString&) const;

    /**
     * \brief   Returns whether the tool is enabled.
     *
     * \param[in] tool name.
     */
    virtual bool toolEnabled(const QString&) const = 0;

    /**
     * \brief   Set whether the tool is enabled.
     *
     * \param[in] tool name.
     */
    virtual void setToolEnabled(const QString&) = 0;

    /**
     * \brief   Returns whether the tool is disabled.
     *
     * \param[in] tool name.
     */
    virtual bool toolDisable(const QString&) const = 0;

    /**
     * \brief   Set whether the tool is disabled.
     *
     * \param[in] tool name.
     */
    virtual void setToolDisable(const QString&) = 0;

    /**
     * \brief   Return Does the target tool exist.
     *
     * \param[in] tool name.
     */
    virtual bool hasTool(const QString&) const = 0;

    /**
     * \brief   Returns the option property value of the target component.
     *
     * \param[in] Component option name.
     *
     * \see     void setOptionValue(const QString&, const QVariant&)
     */
    virtual QVariant optionValue(const QString&) const = 0;

    /**
     * \brief   Sets the option property value of the target component.
     *
     * \param[in] Component option name.
     * \param[in] Component option value.
     *
     * \note	For a description of the option values, see labstractdeviceinterface class.
     *
     * \see     QVariant optionValue(const QString&)
     */
    virtual void setOptionValue(const QString&, const QVariant&) = 0;

    /**
     * \brief   Request component instance object processing target requirements.
     *
     * \param[in] Component function name.
     * \param[in] Component function param.
     *
     * \note	This method only pushes the request to the component instance object, and
     *			the final interpretation right belongs to the component instance object.
     *
     *          This method is not perfect for the time being. It only realizes
     *          the target requirements in a simple way. The ideal call of this
     *          method is to associate and call the interface by the qmethondtype
     *          class of QT.
     *
     * \see     QVariant optionValue(const QString&)
     */
    virtual bool requestExecOperate(const QString&, const QStringList&) = 0;
protected:
    bool m_isValid;
    WorkerState m_workState;
    QString m_ipAddress;
    QString m_port;
    QString m_deviceName;
    QMap<QString, QString> m_romfiles;
};

// #define LAbstractDeviceInterface_iid "com.Interface.LAbstractDeviceInterface"
// Q_DECLARE_INTERFACE(LAbstractDeviceInterface, LAbstractDeviceInterface_iid)
/*@} Interface*/
#endif // LABSTRACTDEVICEINTERFACE_H
