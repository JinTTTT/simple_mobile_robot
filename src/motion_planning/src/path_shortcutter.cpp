#include "motion_planning/path_shortcutter.hpp"

GridPath PathShortcutter::shortcut(
  const GridPath & raw_path,
  const std::function<bool(const GridCell &, const GridCell &)> & has_line_of_sight) const
{
  if (raw_path.size() <= 2) {
    return raw_path;
  }

  GridPath shortcut_path;
  shortcut_path.reserve(raw_path.size());
  shortcut_path.push_back(raw_path.front());

  std::size_t anchor_index = 0;
  while (anchor_index < raw_path.size() - 1) {
    std::size_t furthest_visible_index = anchor_index + 1;

    for (std::size_t candidate_index = anchor_index + 1; candidate_index < raw_path.size();
      ++candidate_index)
    {
      if (!has_line_of_sight(raw_path[anchor_index], raw_path[candidate_index])) {
        break;
      }
      furthest_visible_index = candidate_index;
    }

    shortcut_path.push_back(raw_path[furthest_visible_index]);
    anchor_index = furthest_visible_index;
  }

  return shortcut_path;
}
