
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef GLOG_UTIL_HPP_
#define GLOG_UTIL_HPP_

#include <iostream>
#include <string>
#include <fstream>
#include <glog/logging.h>
#include <ros/ros.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace glog_helper {
void SignalHandle(const char* data, int size)
{
  std::ofstream fs("/home/hambin/ROBOT_INDOOR/logs/glog_dump.log",std::ios::app);
  std::string str = std::string(data,size);
  fs<<str;
  fs.close();
  LOG(ERROR)<<str;
}

class GlogHelper{

#ifdef Work_Path
  std::string workspace_path = Work_Path;
#else
#error Work_Path not define in CMakelists.txt.
#endif

public:
  GlogHelper(const char* cmd)
  {
    FLAGS_logbufsecs = 0;
    google::InitGoogleLogging(cmd);
    FLAGS_log_dir = workspace_path + "/logs";
    google::SetStderrLogging(google::INFO);
    FLAGS_colorlogtostderr=true;
    FLAGS_max_log_size =100;
    FLAGS_stop_logging_if_full_disk = true;
    google::InstallFailureSignalHandler();
    google::InstallFailureWriter(&SignalHandle);
  }

  ~GlogHelper(void)
  {
    google::ShutdownGoogleLogging();
  }

};
}
#endif
