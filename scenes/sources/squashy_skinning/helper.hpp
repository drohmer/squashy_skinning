#pragma once

#include "scenes/base/base.hpp"
#ifdef SCENE_SQUASHY_SKINNING

#include "skinning_loader.hpp"


vcl::buffer<joint_geometry> interpolate_skeleton_at_time(float time, const vcl::buffer< vcl::buffer<joint_geometry_time> >& animation, bool interpolate);
vcl::buffer<joint_geometry> local_to_global(const vcl::buffer<joint_geometry>& local, const vcl::buffer<joint_connectivity>& connectivity);


void display_frames(const vcl::buffer<joint_geometry>& skeleton_geometry,
                    const scene_structure& scene,
                    vcl::mesh_drawable& frame);
void display_joints(const vcl::buffer<joint_geometry>& skeleton_geometry,
                    const scene_structure& scene,
                    vcl::mesh_drawable& sphere);
void display_skeleton(const vcl::buffer<joint_geometry>& skeleton_geometry,
                      const vcl::buffer<joint_connectivity>& skeleton_connectivity,
                      const std::map<std::string,GLuint>& shaders,
                      const scene_structure& scene,
                      vcl::segment_drawable_immediate_mode& segment_drawer);

#endif
