#ifndef RADIAL_MENU_MODEL_ACTION_HPP
#define RADIAL_MENU_MODEL_ACTION_HPP


#include <radial_menu_model/xml_element.hpp>
#include <radial_menu_model/item.hpp>
#include <memory>

namespace radial_menu_model {

// pointer_types

class Action;
typedef std::shared_ptr<Action> ActionPtr;
typedef std::shared_ptr<const Action> ActionConstPtr;

class Action : public std::enable_shared_from_this< Action > {

protected:
  Action() : elm(){}

public:
  virtual ~Action() {}

  const std::string &type() const { return type_; }

  const std::string &topic() const { return topic_; }
  const std::string &topic_type() const { return topic_type_; }
  const int32_t &id() const { return item_id_; }
  const std::string &values() const { return values_; }

  template <typename T>
  const std::vector< T > values() const {

    std::istringstream iss(values_);
    std::vector< T > outputs;

    T v;
    while (iss >> v) {
      outputs.push_back(v);
    }
    return outputs;
  }

  // factory 
  static bool appendActions(const XmlElement &elm, std::vector< ActionConstPtr > *const actions,
                            const ItemPtr &parent_item) {
    if (elm.name() != "action") {
      ROS_ERROR_STREAM("Action::appendActions(): Unexpected element '" << elm.name()
                                                                            << "'");
      return false;      
    }
    if (!parent_item) {
      ROS_ERROR_STREAM("Action::appendActions(): No parent item '" << elm.name() << "'");
      return false;
    }

    const ActionPtr action(new Action());
    action->item_id_ = parent_item->itemId();
    actions->push_back(action);

    if (!elm.getAttribute("type", &action->type_)) {
      ROS_ERROR("Action::appendActions(): No attribute 'type'");
      return false;
    }

    action->elm.reset(new XmlElement(elm));

    if (action->type_ == "publish" || action->type_ == "service") {
      if (!elm.getAttribute("topic", &action->topic_)) {
        ROS_ERROR("Action::appendActions(): No attribute 'topic'");
        return false;
      }

      if (!elm.getAttribute("topic_type", &action->topic_type_)) {
        ROS_ERROR("Action::appendActions(): No attribute 'topic_type'");
        return false;
      }      
    }

    if (!elm.getAttribute("values", &action->values_)) {
      ROS_ERROR("Action::appendActions(): No attribute 'values'");
      return false;
    } 

    // ROS_INFO_STREAM("Appending actions : " << action->topic_);
    return true;
  }
protected:
  typedef std::weak_ptr< const Action > ActionWeakConstPtr;

  std::int32_t item_id_;
  std::string type_;

  // Publish & Servise
  std::string topic_, topic_type_;

  std::string values_;

public:
  XmlElementConstPtr elm;
};



} // namespace radial_menu_model


#endif // RADIAL_MENU_MODEL_ACTION_HPP