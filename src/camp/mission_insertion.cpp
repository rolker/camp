#include "mission_insertion.h"

namespace camp::mission
{

MissionItem* resolveInsertionParent(MissionItem* requested, MissionItem* currentGroup)
{
    return requested ? requested : currentGroup;
}

}  // namespace camp::mission
