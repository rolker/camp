#include "name.h"
#include <sstream>
#include "names_manager.h"


namespace camp
{
namespace ros
{

Name::Name(map::MapItem* parent):
  map::MapItem(parent, "/")
{
}

Name::Name(Name* parent, const QString& basename):
  map::MapItem(parent, basename)
{
  if(basename.isEmpty() || basename.contains('/'))
    throw std::invalid_argument("Name basename cannot be empty or contain '/'");
  Name* stack_before_target = nullptr;
  for(auto sibling: parent->child_names())
  {
    if(sibling->basename() < basename.toStdString())
    {
      stack_before_target = sibling;
      break;
    }
  }
  if(stack_before_target)
    stackBefore(stack_before_target);
}

std::string Name::basename() const
{
  return objectName().toStdString();
}

std::string Name::ros_namespace() const
{
  auto parent_item = parentMapItem();
  auto parent_name = dynamic_cast<Name*>(parent_item);
  if(parent_name)
    return parent_name->full_name();
  return "/";
}

std::string Name::full_name() const
{
  if(is_root())
    return "/";
  
  auto ns = ros_namespace();
  if(ns == "/")
    return "/"+basename();
  return ns+"/"+basename();
}

bool Name::is_root() const
{
  return objectName() == "/";
}

Name* Name::find(const std::string& name)
{
  if(name.empty())
    return nullptr;

  // Start with full name.
  if(name[0] == '/')
  {
    if(name == full_name())
      return this;

    auto ns = ros_namespace();
    if(ns.size() > 1)
      ns += '/';

    // full name must be at least 1 char
    if(name.size() < ns.size()+1)
      return nullptr;

    // name's namespace must match this name's namespace
    if(name.substr(0, ns.size()) != ns)
      return nullptr;

    // account for the '/'
    auto relative_name = name.substr(ns.size());
    return find(relative_name);
  }
  else
  {
    // is it a basename only?
    if(name.find('/') == std::string::npos)
    {
      // see if it already exists
      for(auto item: childItems())
      {
        auto child_name = dynamic_cast<Name*>(item);
        if(child_name && child_name->basename() == name)
          return child_name;
      }
    }
    else
    {
      // it's a relative name with namespace components
      auto first_part = name.substr(0, name.find('/'));
      auto rest = name.substr(name.find('/')+1);
      for(auto item: childItems())
      {
        auto child_name = dynamic_cast<Name*>(item);
        if(child_name && child_name->basename() == first_part)
          return child_name->find(rest);
      }
    }
  }
  return nullptr;
}

Name* Name::add(const std::string& name)
{
  if(name.empty())
    throw std::invalid_argument("Name cannot be empty");

  // Start with full name.
  if(name[0] == '/')
  {
    if(name == full_name())
      return this;

    auto ns = ros_namespace();
    if(ns.size() > 1)
      ns += '/';

    // full name must be at least 1 char
    if(name.size() < ns.size()+1)
      return nullptr;

    // name's namespace must match this name's namespace
    if(name.substr(0, ns.size()) != ns)
      return nullptr;

    // account for the '/'
    auto relative_name = name.substr(ns.size());
    return add(relative_name);
  }
  else
  {
    // is it a basename only?
    if(name.find('/') == std::string::npos)
    {
      // see if it already exists
      for(auto item: childItems())
      {
        auto child_name = dynamic_cast<Name*>(item);
        if(child_name && child_name->basename() == name)
          return child_name;
      }
      // create new name
      auto new_name = new Name(this, QString::fromStdString(name));
      return new_name;
    }
    else
    {
      // it's a relative name with namespace components
      auto first_part = name.substr(0, name.find('/'));
      auto rest = name.substr(name.find('/')+1);
      for(auto item: childItems())
      {
        auto child_name = dynamic_cast<Name*>(item);
        if(child_name && child_name->basename() == first_part)
          return child_name->add(rest);
      }
      // create new name for first part
      auto new_name = new Name(this, QString::fromStdString(first_part));
      return new_name->add(rest);
    }
  }
  return nullptr;
}

void Name::set_types(const std::vector<std::string>& types)
{
  types_ = types;
  if(types.empty())
  {
    setStatus("");
    return;
  }
  if(types.size() == 1 && types[0].empty())
  {
    setStatus("");
    return;
  }
  std::stringstream ss;
  ss << "[";
  for(auto it = types.begin(); it != types.end(); ++it)
  {
    if(it != types.begin())
      ss << ", ";
    ss << *it;
  }
  ss << "]";
  setStatus(ss.str().c_str());
}

std::vector<Name*> Name::child_names() const
{
  std::vector<Name*> names;
  for(auto item: childItems())
  {
    auto child_name = dynamic_cast<Name*>(item);
    if(child_name)
      names.push_back(child_name);
  }
  return names;
}

std::vector<Name *> Name::entity_names() const
{
  std::vector<Name*> names;
  if(!types_.empty())
    names.push_back(const_cast<Name*>(this));

  for(auto child_name: child_names())
  {
    auto child_entities = child_name->entity_names();
    names.insert(names.end(), child_entities.begin(), child_entities.end());
  }
  return names;
}

std::vector<Name *> Name::empty_namespaces() const
{
  std::vector<Name*> names;
  if(entity_names().empty())
  {
    names.push_back(const_cast<Name*>(this));
    return names;
  }

  for(auto child_name: child_names())
  {
    auto child_empty = child_name->empty_namespaces();
    names.insert(names.end(), child_empty.begin(), child_empty.end());
  }
  return names;
}

void Name::contextMenu(QMenu* menu)
{
  auto manager = parentOfType<NamesManager>();
  if(manager)
  {
    auto parent_of_manager = manager->parentMapItem();
    if(parent_of_manager)
      parent_of_manager->contextMenuForItem(this, menu);
  }

}

} // namespace ros
} // namespace camp
