#pragma once

#include "scenes/base/base.hpp"
#ifdef SCENE_SQUASHY_SKINNING

#include "quaternion.hpp"
#include "velocity_tracker.hpp"





// Connectivity information of a joint in the hierarchy
//  Store parent index, and the current name
struct joint_connectivity
{
    int parent;
    std::string name;
};

// 3D Geometry of a joint (position p, and rotation r)
struct joint_geometry
{
    vcl::vec3 p;
    quaternion r;
};

// Key pose of a joint (Key-time, and geometry at this time)
struct joint_geometry_time
{
    float time;
    joint_geometry geometry;
};

// Storage of the influence of a joint for a given vertex
struct skinning_influence
{
    int joint;    // index of the corresponding joint
    float weight; // skinning weight of this joint
};

// Structure storing skeleton data to perform skinning afterward
struct skeleton_structure
{
    vcl::buffer<joint_connectivity> connectivity;           // Connectivity of the skeleton
    vcl::buffer<joint_geometry>     rest_pose;              // Skeleton of the rest pose expressed in local coordinates
    vcl::buffer<vcl::buffer<joint_geometry_time> > anim;    // Skeleton animation expressed in local coordinates (N_joint x N_time)
};

// Storage structure to perform skinning deformation of a surface
struct skinning_structure
{
    vcl::buffer< vcl::buffer<skinning_influence> > influence; // Skinning weights: for each vertex, store all influence values (bone+weight)
    vcl::buffer<vcl::vec3> rest_pose;                         // 3D position of the mesh in rest pose
    vcl::buffer<vcl::vec3> rest_pose_normal;                  // 3D normals of the mesh in rest pose
    vcl::mesh deformed;                                       // Deformed mesh
};

enum gui_parameters_display_type {display_sphere, display_character, display_cylinder_bending, display_cylinder_translate, display_rondinella, display_bar};
struct gui_parameters
{
    bool display_skeleton_bones;
    bool display_skeleton_joints;
    bool display_skeleton_frames;
    bool display_mesh;
    bool display_rest_pose;
    bool display_wireframe;
    bool display_texture;
    int display_type;
    bool dual_quaternion;
    bool interpolate;

};

struct scene_model : scene_base
{

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void set_gui();
    void mouse_move(scene_structure& scene, GLFWwindow* window);


    skeleton_structure skeleton;
    skinning_structure skinning;
    vcl::mesh_drawable character_visual;

    vcl::buffer<joint_geometry> skeleton_local_current;
    vcl::buffer<joint_geometry> skeleton_current;
    vcl::buffer<joint_geometry> skeleton_rest_pose;

    // Squashy
    vcl::buffer<vcl::vec3> skeleton_speed;
    vcl::buffer<vcl::vec3> skeleton_acceleration;

    vcl::buffer<vcl::velocity_tracker_structure<vcl::vec3> > skeleton_velocity_tracker;
    vcl::buffer<vcl::velocity_tracker_structure<quaternion> > skeleton_angular_velocity_tracker;

    bool is_interactive = true;
    bool is_flapping = true;
    bool is_speed = false;
    bool is_acceleration = true;
    vcl::buffer<float> weight_flappy;
    vcl::buffer<float> weight_squashy;

    float flapping_power  = 1.0f;
    float squashing_power = 1.0f;

    void compute_skinning();
    void compute_skinning_dual_quaternion();
    void squashy_skinning();
    void flappy_skinning();
    void resize_structure(); // resize all skeleton when changing states

    vcl::segment_drawable_immediate_mode segment_drawer;
    vcl::mesh_drawable sphere;
    GLuint shader_mesh;

    vcl::mesh_drawable frame;

    gui_parameters gui_param;

    vcl::timer_interval timer;
    vcl::vec2 cursor_prev;

    int picked_object = 0;
};








#endif
