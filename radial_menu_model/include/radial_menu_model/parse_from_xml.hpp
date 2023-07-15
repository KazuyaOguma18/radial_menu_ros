// Pasrsing from xml to divide to items & actions
#ifndef RADIAL_MENU_MODEL_PARSE_FROM_XML_HPP
#define RADIAL_MENU_MODEL_PARSE_FROM_XML_HPP

#include <radial_menu_model/action.hpp>
#include <radial_menu_model/item.hpp>

namespace radial_menu_model {

struct Element
{
  std::vector<ActionConstPtr> actions;
  std::vector<ItemConstPtr> items;

  bool empty() {
    return (actions.empty() && items.empty());
  }

  void reset() {
    actions.clear();
    items.clear();
  }

};

static Element elementsFromDescription(const std::string &desc) {

  static Element model_elms;
  model_elms.reset();

  struct Internal {
    static bool appendElms(const XmlElement &elm, const ItemPtr &parent_item = ItemPtr(), const int default_row = 0) {
      const ItemPtr last_item(new Item());

      // ROS_INFO_STREAM("Num of Elements: " << elm.numChildElements());
      // If the element name is item
      if (elm.name() == "item") {
        if (!Item::appendItems(elm, &model_elms.items, last_item, parent_item, default_row)) {
          return false;
        }
      }      
      // If the element name is action
      else if (elm.name() == "action") {
        if (!Action::appendActions(elm, &model_elms.actions, parent_item)) {
          return false;
        }
      }

      else {
        ROS_ERROR_STREAM("elementsFromDescription() : Unexpected element '" << elm.name()
                                                                                << "'");
        return false;
      }

      // recursively update the given list
      const std::vector< XmlElementConstPtr > child_elms(elm.childElements());
      for (std::size_t i = 0; i < child_elms.size(); ++i) {
        // const ItemPtr item = std::const_pointer_cast<Item>();
        if (!appendElms(*child_elms[i], last_item, i)){
          return false;
        }
      }

      return true;
    }
  };

  // parse the given xml description
  const XmlElementConstPtr elm(XmlElement::fromString(desc));
  if (!elm) {
    return Element();
  }

  // populate items by parsing the root xml element
  if (!Internal::appendElms(*elm)) {
    return Element();
  }

  // ROS_INFO_STREAM("Elements from description , child_level : " << model_elms.items.size());
  // for (const auto& item : model_elms.items) {
  //   ROS_INFO_STREAM("Item : " << item->name());
  // }

  return model_elms;

}

} // namespace radial_menu_model

#endif