#include <calibration/image_viewer.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>

namespace calibration
{

ImageViewer::ImageViewer(std::string topic)
  : is_started_(false), topic_(topic), async_spinner_(0, &callback_queue_)
{
  nh_.setCallbackQueue(&callback_queue_);
  cv::namedWindow(topic_, cv::WINDOW_NORMAL);
  cv::startWindowThread();
}

void ImageViewer::start()
{
  ROS_INFO("[%s]: Start viewer for topic '%s'.", name().c_str(), topic_.c_str());

  subscribe();
  async_spinner_.start();
  is_started_ = true;
}

void ImageViewer::stop()
{
  ROS_INFO("[%s]: Stop viewer for topic '%s'.", name().c_str(), topic_.c_str());

  sub_image_.shutdown();
  async_spinner_.stop();
  is_started_ = false;
}

void ImageViewer::save(const std::string& file)
{
  ROS_INFO("[%s]: Save image to file '%s'.", name().c_str(), file.c_str());

  async_spinner_.stop();
  subscribe();

  cv_image_ptr_.reset();
  callback_queue_.callOne(ros::WallDuration(5));

  if (cv_image_ptr_ != NULL)
  {
    boost::filesystem::path path(file);
    boost::filesystem::create_directories(path.parent_path());
    cv::imwrite(file, cv_image_ptr_->image);
  }
  else
    ROS_WARN("[%s]: Could not save image. Queue was empty.", name().c_str());

  if (!is_started_)
    sub_image_.shutdown();

  if (is_started_)
    async_spinner_.start();
}

void ImageViewer::subscribe()
{
  if (is_started_)
    return;

  image_transport::ImageTransport image_transport(nh_);
  sub_image_ = image_transport.subscribe(topic_, 1, &ImageViewer::callbackImage, this);
}

void ImageViewer::callbackImage(const sensor_msgs::ImageConstPtr& image_msg)
{
  try
  {
    cv_image_ptr_ = cv_bridge::toCvCopy(image_msg, "bgr8");
    cv::imshow(topic_, cv_image_ptr_->image);
    cv::waitKey(25);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("[%s]: Could not convert from '%s' to 'bgr8'.",
      name().c_str(), image_msg->encoding.c_str());
  }
}

}