#ifndef PR2_CALIBRATION_IMAGE_VIEWER_H
#define PR2_CALIBRATION_IMAGE_VIEWER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace calibration
{

class ImageViewer
{

public:

  typedef boost::shared_ptr<ImageViewer> Ptr;

  ImageViewer(std::string topic);

  ~ImageViewer() { sub_image_.shutdown(); async_spinner_.stop(); }

  std::string getTopic() const { return topic_; }

  void start();

  void stop();

  void save(const std::string& file);

private:

  ImageViewer(const ImageViewer&);

  ImageViewer& operator=(const ImageViewer&);

  void subscribe();

  void callbackImage(const sensor_msgs::ImageConstPtr& image_msg);

  std::string nameSpace() { return "pr2_calibration"; };

  std::string className() { return "ImageViewer"; };

  std::string name() { return nameSpace() + "::" + className(); };


  bool is_started_;
  std::string topic_;
  ros::NodeHandle nh_;
  ros::AsyncSpinner async_spinner_;
  ros::CallbackQueue callback_queue_;
  image_transport::Subscriber sub_image_;
  cv_bridge::CvImagePtr cv_image_ptr_;

};

}

#endif