
#include "WPILib.h"
#include "gmCamera.h"

namespace GM_Code
{

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Here's the way to set up the camera for image processing
// and display on the smart dashboard. >>> Note that you must
// select the correct camera for each 'CameraServer Stream
// Viewer' using edit mode on the smart dashboard. This might
// involve some trial-and-error because the FRC API is goofy.
// In particular, there's an internal connection from the
// 'CameraServer' code to the dashboard that's not obvious
// and persists whether *we* want to 'drive' or not. (!)
//............................................................

void gmCamera::StartCapturing(std::string name, int dev, int hres, int vres, int delay_ms, ImageProcFn fn)
{
    // std::this_thread::sleep_for(std::chrono::seconds(2));

	cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture(dev);
	camera.SetResolution(hres, vres);

	cs::CvSink sink = CameraServer::GetInstance()->GetVideo(camera);
    cs::CvSource outputStreamStd = CameraServer::GetInstance()->PutVideo(name, hres, vres);

    cv::Mat src, dst;

    while (true)
    {
        sink.GrabFrame(src);
        if (fn != nullptr)
        {
        	// Call image processing function, if any.
            fn(src, src);
        }
        outputStreamStd.PutFrame(src);

        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Here are a couple of image processing routines that can
// be passed to StartCapturing(). I suggest keeping such code
// right here, in the camera class (as static methods).
// See the video capture section in Robot.cpp::RobotInit()
// for how-to-call examples.
//............................................................

void gmCamera::toBW(cv::InputArray src, cv::OutputArray dst)
{
    cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
}

void gmCamera::toEdges(cv::InputArray src, cv::OutputArray dst)
{
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// Just a first test... Can be greatly improved. (!)
	//............................................................
	const int siz = 3;

	cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
	cv::blur(src, dst, cv::Size(siz, siz));
	cv::Canny(src, dst, 50, 150, siz);
}

} // namespace GM_Code
