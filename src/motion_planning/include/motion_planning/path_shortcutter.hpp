#ifndef MOTION_PLANNING__PATH_SHORTCUTTER_HPP_
#define MOTION_PLANNING__PATH_SHORTCUTTER_HPP_

#include "motion_planning/path_types.hpp"

#include <functional>

class PathShortcutter
{
public:
  GridPath shortcut(
    const GridPath & raw_path,
    const std::function<bool(const GridCell &, const GridCell &)> & has_line_of_sight) const;
};

#endif  // MOTION_PLANNING__PATH_SHORTCUTTER_HPP_
