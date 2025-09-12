#ifndef CAMP_ROS_NAME_H
#define CAMP_ROS_NAME_H

#include "../map/map_item.h"

namespace camp
{
namespace ros
{

/// MapItem used to display components of the  name of a ROS entity
/// such as a node or topic.
/// Names representing a ROS entity should have one or more types associated
/// with them.
class Name: public map::MapItem
{
  Q_OBJECT
public:
  /// Constructor for the root name ("/").
  Name(map::MapItem* parent);

  /// Constructor for a name with the given basename.
  Name(Name* parent, const QString& basename);

  /// Returns the basename portion of the name.
  /// basename is that last portion of the full name after the last '/'.
  std::string basename() const;

  /// Returns the namespace portion of the name.
  /// If the name has no namespace, returns "/".
  /// Note: method is called ros_namespace() to avoid conflict with C++ keyword.
  std::string ros_namespace() const;

  /// Returns the full name including namespace and basename.
  std::string full_name() const;

  /// Returns true if this is the root name ("/").
  bool is_root() const;

  /// Finds a Name item representing the given name.
  /// Returns nullptr if not found.
  Name * find(const std::string& name);

  /// Returns a Name item representing the name creating the name components
  /// as needed.
  Name * add(const std::string& name);

  /// Sets the list of types associated with this name.
  void set_types(const std::vector<std::string>& types);

  /// Returns the list of types associated with this name.
  const std::vector<std::string>& types() const
  {
    return types_;
  }

  /// Returns children Name items.
  std::vector<Name*> child_names() const;

  /// Returns the child items that are Names representing a ROS entity
  /// (i.e. have one or more types associated with them).
  /// Search is recursive.
  std::vector<Name *> entity_names() const;

  /// Return child items that do not contain entities, i.e. are part of the namespace only.
  /// Search is recursive.
  std::vector<Name *> empty_namespaces() const;

private:
  /// List of types associated with names representing a ROS entity.
  /// Names without a type are part of the namespace only.
  std::vector<std::string> types_;

};

} // namespace ros
} // namespace camp

#endif
