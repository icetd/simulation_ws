#ifndef APPLICATION_H
#define APPLICATION_H

#include <stdlib.h>
#include <opencv2/core/hal/interface.h>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>
#include <aruco.h>
#include <functional>
#include "VideoCapture.h"
#include "thread.h"

#define FRAME_SIZE (frame_width * frame_height * 3)
#define MAKER_SIZE  0.033  // m

class Detector : public Thread 
{
public:
    Detector(const char *camera_calibration_file);
    ~Detector();


    void Init(const char *url);
    void setArucoCallback(std::function<void (int32_t x, int32_t y, int32_t z)> callback);

protected:
    virtual void run() override;

private:
    aruco::CameraParameters m_cameraParameters;
    aruco::MarkerDetector m_detector;
	float m_x, m_y, m_z;
    
    VideoCapture *m_capture;
	uint8_t *m_frame_data;

    cv::Mat uData2cvMat_8UC3(uint8_t *data, int data_width, int data_height);
    std::function <void (int32_t x, int32_t y, int32_t z)> OnArucoSend;
};


#endif
