#ifndef RVIZ_WIDGET_HPP
#define RVIZ_WIDGET_HPP

#include <string>

namespace rviz_embed {

void launch_rviz(int64_t parent_win_id, 
                 const std::string& urdf_path, 
                 const std::string& description_topic, 
                 const std::string& fixed_frame);

void add_robot(const std::string& description_topic);

void cleanup_rviz();
void emergency_cleanup();  // ← ADD THIS LINE

} // namespace rviz_embed

#endif // RVIZ_WIDGET_HPP
