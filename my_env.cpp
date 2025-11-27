#include "my_env.h"

bool MyEnvironment::Read(const std::string &env_file) {
  std::ifstream is(env_file, std::ios::binary);
  if(is.is_open()) {
    json j = json::parse(is);
    *this = j;

    ROS_INFO("read points: %zu, obstacles: %zu", points.size(), obstacles.size());
    return true;
  } else {
    return false;
  }
}

void MyEnvironment::Save(const std::string &env_file) const {
  std::ofstream os(env_file, std::ios::binary);
  if(os.is_open()) {
    json j = *this;
    os << j;
    ROS_INFO("write points: %zu, obstacles: %zu", points.size(), obstacles.size());
  }
}

void MyEnvironment::CollectResult(const std::string &env_file, const std::string &tag, double run_time) const {
  char name_buf[128] = {0}, time_buf[16] = {0};
  std::time_t t = std::time(nullptr);
  std::tm *tm = std::localtime(&t);
  std::strftime(time_buf, 16, "%y%m%d%H%M", tm);

  auto tags = tag;
  if(!last_trajectory.empty()) {
    tags += "-RP";
  }
  std::snprintf(name_buf, 128, "env-%s-%s-%.2f.json", time_buf, tags.c_str(), run_time);

  size_t pos = env_file.find_last_of('/');
  std::string path = (std::string::npos == pos) ? "." : env_file.substr(0, pos);

  Save(path + "/" + std::string(name_buf));
}
