#include <calibration/image_viewer.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <termios.h>
#include <iostream>
#include <fstream>
#include <time.h>

using namespace calibration;

void set_keyboard_non_blocking(struct termios& termios)
{
    struct termios termios_new;
    tcgetattr(0, &termios);

    termios_new = termios;
    termios_new.c_lflag &= ~ICANON;
    termios_new.c_lflag &= ~ECHO;
    termios_new.c_lflag &= ~ISIG;
    termios_new.c_cc[VMIN] = 0;
    termios_new.c_cc[VTIME] = 0;

    tcsetattr(0, TCSANOW, &termios_new);
}

void restore_keyboard_settings(const struct termios& termios)
{
  tcsetattr(0, TCSANOW, &termios);
}

std::string now()
{
  time_t time_raw;
  time(&time_raw);
  struct tm* time_info = localtime(&time_raw);

  char buffer[80];
  strftime(buffer, sizeof(buffer), "%F-%H-%M-%S", time_info);
  
  return buffer;
}

void save(const std::vector<ImageViewer::Ptr>& viewers, const std::string& root_dir)
{
  for (size_t i = 0; i < viewers.size(); i++)
    viewers[i]->stop();

  for (size_t i = 0; i < viewers.size(); i++)
  {
    std::stringstream ss;
    ss << root_dir << "/cam" << i << "/" << ros::Time::now().toNSec() << ".png";
    viewers[i]->save(ss.str());
  }

  for (size_t i = 0; i < viewers.size(); i++)
    viewers[i]->start();

  if (!boost::filesystem::exists(root_dir + "/cameras.txt") && boost::filesystem::exists(root_dir))
  {
    std::ofstream fs;
    fs.open((root_dir + "/cameras.txt").c_str());

    for (size_t i = 0; i < viewers.size(); i++)
      fs << "cam" << i << " " << viewers[i]->getTopic() << "\n";

    fs.close();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_viewer");
  ros::NodeHandle nh, nh_private("~");

  std::string root_dir = now();
  std::vector<std::string> topics;
  if (!nh_private.getParam("topics", topics))
  {
    ROS_ERROR("Could not find 'topics' on parameter server.");
    return EXIT_FAILURE;
  }

  std::vector<ImageViewer::Ptr> viewers;
  for (size_t i = 0; i < topics.size(); i++)
  {
    viewers.push_back(ImageViewer::Ptr(new ImageViewer(topics[i])));
    viewers.back()->start();
  }

  ROS_INFO("Options: (S)ave | (Q)uit");
  struct termios termios;
  set_keyboard_non_blocking(termios);

  while (ros::ok())
  {
    char c = getchar();

    if (c == 'q')
      break;

    if (c == 's')
      save(viewers, root_dir);

    ros::Rate(10).sleep();
  }

  restore_keyboard_settings(termios);
}