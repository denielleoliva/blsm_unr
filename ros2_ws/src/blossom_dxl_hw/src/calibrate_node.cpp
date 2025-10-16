#include <rclcpp/rclcpp.hpp>
#include <dynamixel_sdk/port_handler.h>
#include <dynamixel_sdk/packet_handler.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <string>
#include <vector>
#include <array>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
#include <optional>

namespace dx = dynamixel;

static constexpr uint16_t ADDR_TORQUE_ENABLE     = 64;
static constexpr uint16_t ADDR_GOAL_POSITION     = 116;
static constexpr uint16_t ADDR_PRESENT_POSITION  = 132;
// XL-330 ticks ↔ degrees
static inline int   deg_to_pos(double deg){ if(deg<0)deg=0; if(deg>360)deg=360; return int(std::round(deg*4095.0/360.0)); }
static inline double pos_to_deg(int pos){ return pos * 360.0 / 4095.0; }

class RawTTY {
public:
  RawTTY() { fd_ = ::fileno(stdin); tcgetattr(fd_, &old_); termios raw = old_; raw.c_lflag &= ~(ICANON|ECHO); tcsetattr(fd_, TCSANOW, &raw); }
  ~RawTTY(){ tcsetattr(fd_, TCSANOW, &old_); }
  std::optional<char> read(double timeout_sec=0.05){
    fd_set rfds; FD_ZERO(&rfds); FD_SET(fd_, &rfds);
    timeval tv; tv.tv_sec = int(timeout_sec); tv.tv_usec = int((timeout_sec - int(timeout_sec))*1e6);
    int ret = select(fd_+1, &rfds, nullptr, nullptr, &tv);
    if(ret>0 && FD_ISSET(fd_, &rfds)){ char c; if(::read(fd_, &c, 1)==1) return c; }
    return std::nullopt;
  }
private:
  int fd_; termios old_;
};

class CalibNode : public rclcpp::Node {
public:
  CalibNode() : Node("blossom_calibrate_cpp") {
    port_   = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud_   = this->declare_parameter<int>("baud", 57600);
    ids_    = this->declare_parameter<std::vector<int64_t>>("ids", {1,2,3,4});
    names_  = this->declare_parameter<std::vector<std::string>>("names", {"base_pan","string_front","string_back_right","string_back_left"});
    outfile_= this->declare_parameter<std::string>("outfile", "blossom_calibration.yaml");

    if (names_.size() != ids_.size()) { names_.clear(); for (auto id : ids_) names_.push_back("joint_" + std::to_string(id)); }

    // open bus (Protocol 2.0)
    port = dx::PortHandler::getPortHandler(port_.c_str());
    pkt  = dx::PacketHandler::getPacketHandler(2.0);
    if(!port->openPort()) throw std::runtime_error("openPort failed: " + port_);
    if(!port->setBaudRate(baud_)) throw std::runtime_error("setBaudRate failed");

    RCLCPP_INFO(get_logger(), "Calibrating on %s @ %d, ids=%s", port_.c_str(), baud_, vec_to_str(ids_).c_str());

    // torque on all
    for(size_t i=0;i<ids_.size();++i){ uint8_t err=0; pkt->write1ByteTxRx(port, int(ids_[i]), ADDR_TORQUE_ENABLE, 1, &err); }

    run_interactive();

    // torque off & close
    for(size_t i=0;i<ids_.size();++i){ uint8_t err=0; pkt->write1ByteTxRx(port, int(ids_[i]), ADDR_TORQUE_ENABLE, 0, &err); }
    port->closePort();
  }

private:
  // params
  std::string port_;
  int baud_;
  std::vector<int64_t> ids_;
  std::vector<std::string> names_;
  std::string outfile_;

  // sdk
  dx::PortHandler* port{nullptr};
  dx::PacketHandler* pkt{nullptr};

  struct Entry { std::optional<double> neutral, minv, maxv; };
  std::vector<Entry> table;

  static std::string vec_to_str(const std::vector<int64_t>& v){
    std::ostringstream oss; oss<<"["; for(size_t i=0;i<v.size();++i){ if(i)oss<<","; oss<<v[i]; } oss<<"]"; return oss.str();
  }

  double read_deg(int id){
    uint8_t err=0; uint32_t raw=0;
    pkt->read4ByteTxRx(port, id, ADDR_PRESENT_POSITION, &raw, &err);
    return pos_to_deg(int(raw));
  }
  void set_deg(int id, double deg){
    uint8_t err=0; uint32_t ticks = (uint32_t)deg_to_pos(deg);
    pkt->write4ByteTxRx(port, id, ADDR_GOAL_POSITION, ticks, &err);
  }

  void status_line(const std::string& name, double cur, double tgt, const Entry& e){
    auto fmt=[&](const std::optional<double>& x){ return x? fmt1(*x) : std::string("--"); };
    std::cout << "\r["<<name<<"] cur≈" << fmt1(cur) << "°  tgt=" << fmt1(tgt) << "°   "
              << "NEU="<<fmt(e.neutral)<<"  MIN="<<fmt(e.minv)<<"  MAX="<<fmt(e.maxv)<<"   " << std::flush;
  }
  static std::string fmt1(double v){ std::ostringstream o; o.setf(std::ios::fixed); o.precision(1); o<<v; return o.str(); }

  void save_yaml(){
    std::ofstream f(outfile_);
    if(!f){ RCLCPP_ERROR(get_logger(), "Failed to open outfile: %s", outfile_.c_str()); return; }
    f << "blossom_calibration:\n";
    for(size_t i=0;i<ids_.size();++i){
      f<<"  "<<names_[i]<<": { ";
      bool first=true;
      if(table[i].neutral){ f<<(first?"":" ")<<"neutral_deg: "<<*table[i].neutral; first=false; }
      if(table[i].minv){    f<<(first?"":" ,")<<"min_deg: "<<*table[i].minv; first=false; }
      if(table[i].maxv){    f<<(first?"":" ,")<<"max_deg: "<<*table[i].maxv; first=false; }
      f<<" }\n";
    }
    // a tiny composer block with defaults
    f << "pose_composer:\n";
    f << "  base_neutral_deg: " << (table.size()? (table[0].neutral.value_or(180.0)) : 180.0) << "\n";
    f << "  str_neutral_deg: 180.0\n";
    f << "  k_pitch: 0.6\n";
    f << "  k_roll: 0.6\n";
    f << "  tension_bias_default: 0.0\n";
    f.close();
    std::cout << "\n✅ Saved: " << outfile_ << "\n";
  }

  void run_interactive(){
    table.assign(ids_.size(), Entry{});
    print_banner();

    // per-joint loop
    for(size_t i=0;i<ids_.size();++i){
      int id = int(ids_[i]);
      auto name = names_[i];
      RCLCPP_INFO(get_logger(), "--- Calibrating %s (ID %d) ---", name.c_str(), id);

      double cur = 180.0;
      try { cur = read_deg(id); } catch(...) {}
      double tgt = cur;

      RawTTY tty;
      status_line(name, cur, tgt, table[i]);

      while(rclcpp::ok()){
        // non-blocking read key
        auto key = tty.read(0.05);
        if(!key){
          // refresh current pos for display
          try { cur = read_deg(id); } catch(...) {}
          status_line(name, cur, tgt, table[i]);
          continue;
        }
        char c = *key;
        if(c=='<'||c=='>'||c==','||c=='.'||c=='['||c==']'){
          int step = (c=='<')?-1: (c=='>')?+1: (c==',')?-5: (c=='.')?+5: (c=='[')?-10:+10;
          tgt = std::max(0.0, std::min(360.0, tgt + step));
          set_deg(id, tgt);
          status_line(name, cur, tgt, table[i]);
        } else if(c=='n'){ table[i].neutral = cur; std::cout << "\n  • Neutral set.\n"; }
          else if(c=='1'){ table[i].minv = cur;   std::cout << "\n  • Min set.\n"; }
          else if(c=='2'){ table[i].maxv = cur;   std::cout << "\n  • Max set.\n"; }
          else if(c=='g'){ if(table[i].neutral){ tgt = *table[i].neutral; set_deg(id, tgt);} std::cout << "\n  • Go neutral.\n"; }
          else if(c=='t'){ uint8_t e=0; pkt->write1ByteTxRx(port,id,ADDR_TORQUE_ENABLE,1,&e); std::cout << "\n  • Torque ON.\n"; }
          else if(c=='o'){ uint8_t e=0; pkt->write1ByteTxRx(port,id,ADDR_TORQUE_ENABLE,0,&e); std::cout << "\n  • Torque OFF.\n"; }
          else if(c=='s'){ std::cout << "\nSaved & next.\n"; break; }
          else if(c=='q'){ std::cout << "\nAborted.\n"; throw std::runtime_error("User abort"); }
        // keep line refreshed
        status_line(name, cur, tgt, table[i]);
      }

      // normalize min/max if swapped
      if(table[i].minv && table[i].maxv && (*table[i].minv > *table[i].maxv))
        std::swap(table[i].minv, table[i].maxv);
    }

    // send all to neutral
    std::cout << "\nSending all to neutral…\n";
    for(size_t i=0;i<ids_.size();++i){
      if(table[i].neutral) set_deg(int(ids_[i]), *table[i].neutral);
    }
    save_yaml();
  }

  void print_banner(){
    std::cout <<
      "\n=== Blossom Calibration (C++ / ROS 2) ===\n"
      "Keys per joint:  < / > = ±1°   , / . = ±5°   [ / ] = ±10°\n"
      "                  n=neutral   1=min   2=max   g=go neutral\n"
      "                  t=torque ON o=torque OFF    s=save & next   q=quit\n\n";
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<CalibNode>();
    rclcpp::spin(node);
  } catch(const std::exception& e){
    std::cerr << "[calibrate_node] " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
