#ifndef RADIAL_MENU_ACTION_SHARED_VALUES_HPP
#define RADIAL_MENU_ACTION_SHARED_VALUES_HPP

#include "radial_menu_action/shared.hpp"
#include "radial_menu_action/base_action.hpp"

namespace radial_menu_action {

class SharedValues : public BaseAction {
public:
  SharedValues(const radial_menu_model::ActionConstPtr &action) 
   : BaseAction(action), shared_(std::make_shared<Shared>()) {

  }

  virtual bool init() override {
    // Check Elements 


    //

    

    return true;
  }

  virtual void execute() const override {

  }

private:
  const std::shared_ptr<Shared> shared_;
};
} // namespace

#endif