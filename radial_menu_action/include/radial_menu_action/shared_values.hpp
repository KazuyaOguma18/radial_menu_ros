#ifndef RADIAL_MENU_ACTION_SHARED_VALUES_HPP
#define RADIAL_MENU_ACTION_SHARED_VALUES_HPP

#include "radial_menu_action/shared.hpp"
#include "radial_menu_action/base_action.hpp"

namespace radial_menu_action {

template<typename T>
bool is_exist(const std::vector< T > vals, const T& key) {
  for (const auto& v : vals) {
    if (key == v) return true;
  }
  return false;
}

class SharedValues : public BaseAction {
public:
  SharedValues(const radial_menu_model::ActionConstPtr &action) 
   : BaseAction(action), 
     method_list_({"round_robin", "bidirectional+", "bidirectional-"}) {
  }

  virtual bool init() override {
    // Check Elements ------------------------------------------
    // key
    if (!get_attribute("key", key_)) { return false; }

    // values
    if (!get_attribute("values", values_str_)) { return false; }
    values_vec_ = values<std::string>(values_str_);

    // method
    if (!get_attribute("method", method_)) { return false; }
    if (!is_exist(method_list_, method_)) { 
      ROS_ERROR_STREAM("SharedValues::init() : the method['" << method_ << 
      "'] is not matching the list of methods. \nPlease use the method" << method_list_);
      return false; 
    }

    // initial index
    if (!get_attribute("initial", initial_)) { return false; }
    if (stoi(initial_) < (int)values_vec_.size()) {
      current_val_ = values_vec_.at(stoi(initial_));
    }
    else {
      current_val_ = values_vec_.at(0);
    }

    //////////////////////////////////////////////////////////////

    Shared::add(key_, current_val_);
    Shared::add(key_ + "_index", initial_);

    return true;
  }

  virtual void execute() const override {
    current_val_ = select();
    Shared::add(key_, current_val_);
    ROS_INFO_STREAM("SharedValues::execute(): add key='" << key_  << "', value='" << current_val_ << "'");
  }

  std::string select() const {
    const int current_index = stoi(Shared::get(key_ + "_index"));
    int next_index = current_index;
    if (method_ == "round_robin") {
      next_index++;
      if (next_index >= (int)values_vec_.size()) {
        next_index = 0;
      }
    }
    else if (method_ == "bidirectional+") {
      next_index++;
      if (next_index >= (int)values_vec_.size()) {
        next_index = (int)values_vec_.size() - 1;
      }      
    }
    else if (method_ == "bidirectional-") {
      next_index--;
      if (next_index < 0) {
        next_index = 0;
      }      
    }
    Shared::add(key_ + "_index", std::to_string(next_index));
    return values_vec_.at(next_index);
  }

private:
  std::string key_, values_str_, method_, initial_;
  std::vector<std::string> values_vec_;
  const std::vector<std::string> method_list_;

  mutable std::string current_val_;
};
} // namespace

#endif