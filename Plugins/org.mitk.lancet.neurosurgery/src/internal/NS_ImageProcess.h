/*============================================================================
Robot-assisted Surgical Navigation Extension based on MITK
Copyright (c) Hangzhou LancetRobotics,CHINA
All rights reserved.
============================================================================*/

#ifndef NS_IMAGEPROCESSING_H
#define NS_IMAGEPROCESSING_H

#include <berryISelectionListener.h>
#include <berryIStructuredSelection.h>
#include <QmitkAbstractView.h>
#include <mitkImage.h> // 假设你需要处理 MITK 图像

class NS_ImageProcessing : public QmitkAbstractView
{
public:
    NS_ImageProcessing();
    ~NS_ImageProcessing();

    //// 示例方法：加载图像
    //bool LoadImage(const std::string& filePath);

    //// 示例方法：处理图像
    //void ProcessImage();

    //// 示例方法：保存图像
    //bool SaveImage(const std::string& filePath);

private:
    mitk::Image::Pointer m_Image; // 假设你使用 MITK 的图像类
};

#endif // NS_IMAGEPROCESSING_H