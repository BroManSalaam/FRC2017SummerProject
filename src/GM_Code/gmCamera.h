
#pragma once

#include <string>
#include <functional>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace GM_Code
{

typedef std::function<void (cv::InputArray, cv::OutputArray)> ImageProcFn;

class gmCamera
{
  public:
    static void StartCapturing(std::string name, int dev, int hres, int vres, int delay_ms, ImageProcFn fn);

    static void toBW(cv::InputArray src, cv::OutputArray dst);
    static void toEdges(cv::InputArray src, cv::OutputArray dst);

}; // class gmCamera

} // namespace GM_Code
