#include "rclcpp/publisher.hpp"
#include "godot_cpp/classes/worker_thread_pool.hpp"
#include <sstream>


namespace godot::robosim {

class Utils {
public:
  template<class MsgT>
  static void publish_deferred(rclcpp::Publisher<MsgT>::SharedPtr pub, std::unique_ptr<MsgT> msg) {
    struct Arg {
      rclcpp::Publisher<MsgT>::SharedPtr publisher;
      std::unique_ptr<MsgT> msg;
    };

    Arg* argp = memnew(Arg);
    argp->publisher = pub;
    argp->msg = move(msg);
    
    godot::WorkerThreadPool::get_singleton()->add_native_task([](void* userdata){
      Arg* argp = (Arg*)userdata;
      argp->publisher->publish(std::move(argp->msg));
      memdelete(argp);
    }, argp);
  };

  static std::string get_unique_node_name(std::string base_name, void* ptr) {
    std::ostringstream ss;
    ss << std::hex << (size_t)ptr;
    return base_name + ss.str();
  }
};

}