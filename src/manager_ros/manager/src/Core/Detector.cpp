#include "Detector.h"
#include "log.h"

Detector::Detector(const char *camera_calibration_file) : 
	m_x(0.0f),
	m_y(0.0f), 
	m_z(0.0f)
{
	m_cameraParameters.readFromXMLFile(camera_calibration_file);
	m_detector.setDictionary("ARUCO_MIP_36h12");
	m_detector.getParameters().detectEnclosedMarkers(true);

	m_capture = new VideoCapture();
	m_capture->init();
}

Detector::~Detector()
{
	if (m_frame_data)
		free(m_frame_data);
}

cv::Mat Detector::uData2cvMat_8UC3(uint8_t *data, int data_width, int data_height)
{
	cv::Mat frame(data_height, data_width, CV_8UC3);
	memcpy(frame.data, data, data_width * data_height * 3);
	return frame;
}

void Detector::Init(const char *url)
{
	m_capture->open(url);
	m_frame_data = (uint8_t *)malloc(0);
}

void Detector::run()
{
	int frame_width, frame_height;

	frame_width = m_capture->getWidth();
	frame_height = m_capture->getHeight();

	while (!isStoped()) {
		int64 pts = 0;
		m_frame_data = (uint8_t *)realloc(m_frame_data, FRAME_SIZE);
		m_capture->decode(m_frame_data, &pts);
		cv::Mat frame_8uc3 = uData2cvMat_8UC3(m_frame_data, frame_width, frame_height);
		std::vector<aruco::Marker> makerList = m_detector.detect(frame_8uc3, m_cameraParameters, MAKER_SIZE);

		for (auto i : makerList) {
			aruco::CvDrawingUtils::draw3dAxis(frame_8uc3, i, m_cameraParameters);
			m_x = i.Tvec.at<float>(0, 0) * 1000;
			m_y = i.Tvec.at<float>(1, 0) * 1000;
			m_z = i.Tvec.at<float>(2, 0) * 1000;
			LOG(INFO, "[\tx: %.1f mm \ty: %.1f mm \tz: %.1f mm]", m_x, m_y, m_z);

			// rounding xyz
			if (m_x >= 0)
				m_x += 0.5;
			else
				m_x -= 0.5;

			if (m_y >= 0)
				m_y += 0.5;
			if (m_y < 0)
				m_y -= 0.5;

			m_z = m_z + 0.5;
			OnArucoSend(m_x, m_y, m_z);
		}

		this->sleepMs(10);
	}

	m_capture->close();
	this->sleepMs(100);
	free(m_frame_data);
}

void Detector::setArucoCallback(std::function<void(int32_t x, int32_t y, int32_t z)> callback)
{
	OnArucoSend = std::move(callback);
}
