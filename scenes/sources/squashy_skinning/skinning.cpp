
#include "skinning.hpp"
#ifdef SCENE_SQUASHY_SKINNING

#include "skinning_loader.hpp"


using namespace vcl;


buffer<joint_geometry> interpolate_skeleton_at_time(float time, const buffer< buffer<joint_geometry_time> >& animation, bool interpolate);
buffer<joint_geometry> local_to_global(const buffer<joint_geometry>& local, const buffer<joint_connectivity>& connectivity);


void scene_model::setup_data(std::map<std::string,GLuint>& shaders, scene_structure& , gui_structure& )
{
    shader_mesh = shaders["mesh"];

    segment_drawer.init();
    segment_drawer.uniform_parameter.color = {0.0f,0.0f,0.0f};
    glEnable(GL_POLYGON_OFFSET_FILL);

    // Init gui parameters
    gui_param.display_mesh      = true;
    gui_param.display_wireframe = false;
    gui_param.display_rest_pose = false;
    gui_param.display_skeleton_bones  = true;
    gui_param.display_skeleton_joints = true;
    gui_param.display_texture = true;
    gui_param.display_type = display_sphere;
    gui_param.dual_quaternion = false;
    gui_param.interpolate = true;


    // Sphere used to display joints
    sphere = mesh_primitive_sphere(0.005f);
    sphere.shader = shader_mesh;

    frame = mesh_primitive_frame();
    frame.uniform.transform.scaling = 0.02f;
    frame.shader = shaders["mesh"];

    // Load initial model
    load_sphere_data(skeleton, skinning, character_visual, timer, shader_mesh);

    const auto skeleton_geometry_local  = interpolate_skeleton_at_time(0, skeleton.anim, gui_param.interpolate);
    skeleton_rest_pose = local_to_global(skeleton.rest_pose, skeleton.connectivity);
    skeleton_current = local_to_global(skeleton_geometry_local, skeleton.connectivity);
    skeleton_speed.resize(skeleton_current.size());
    skeleton_acceleration.resize(skeleton_current.size());
    skeleton_velocity_tracker.resize(skeleton_current.size());

}


buffer<joint_geometry> interpolate_skeleton_at_time(float time, const buffer< buffer<joint_geometry_time> >& animation, bool interpolate)
{
    // Compute skeleton corresponding to a given time from key poses
    // - animation[k] corresponds to the kth animated joint (vector of joint geometry at given time)
    // - animation[k][i] corresponds to the ith pose of the kth joint
    //      - animation[k][i].time         -> corresponding time
    //      - animation[k][i].geometry.p/r -> corresponding position, rotation

    size_t N_joint = animation.size();
    buffer<joint_geometry> skeleton;
    skeleton.resize(N_joint);

    for(size_t k_joint=0; k_joint<N_joint; ++k_joint)
    {
        const buffer<joint_geometry_time>& joint_anim = animation[k_joint];

        // Find the index corresponding to the current time
        size_t k_current = 0;
        assert_vcl_no_msg(joint_anim.size()>k_current+1);
        while( time>joint_anim[k_current+1].time ) {
            ++k_current;
            assert_vcl_no_msg(joint_anim.size()>k_current+1);
        }


        if(interpolate){
            const joint_geometry& g1 = joint_anim[k_current].geometry;
            const joint_geometry& g2 = joint_anim[k_current+1].geometry;
            const float t1 = joint_anim[k_current].time;
            const float t2 = joint_anim[k_current+1].time;

            const float alpha = (time-t1)/(t2-t1);
            const vec3 p = (1.0f-alpha)*g1.p+alpha*g2.p;

            joint_geometry g;
            g.p = p;
            g.r = slerp(g1.r,g2.r,alpha);

            skeleton[k_joint] = g;
        }
        else {
            const joint_geometry current_geometry = joint_anim[k_current].geometry;
            skeleton[k_joint] = current_geometry;
        }

    }

    return skeleton;
}

// COmpute the squashy skinning
void scene_model::squashy_skinning()
{
    const size_t N_vertex = skinning.rest_pose.size();


    for(size_t k=0; k<N_vertex; ++k) {
        const buffer<skinning_influence>& influence = skinning.influence[k];
        const vec3& p_lbs = skinning.deformed.position[k];

        // Transformation matrix for skinning
        //mat4 M = mat4::zero();
        vec3 p = {0,0,0};
        for(size_t kb=0; kb<influence.size(); ++kb) // Loop over influencing joints
        {
            const int idx = influence[kb].joint;
            const float w = influence[kb].weight;

            const vec3 v = skeleton_speed[idx];
            const float vn = norm(v);
            const mat3 S = mat3(std::min(1+vn/10.0f,2.0f),0,0,
                                0,std::max(1-vn/10.0f,0.0f),0,
                                0,0,1);

            const mat3 R = rotation_between_vector_mat3({1,0,0}, normalize(v));
            const vec3 p_c = skeleton_current[idx].p;

            p += w*(  R*S*transpose(R)*  (p_lbs-p_c)+p_c);
        }

        skinning.deformed.position[k] = p;
    }
}

void scene_model::compute_skinning()
{

    const size_t N_vertex = skinning.rest_pose.size();
    for(size_t k=0; k<N_vertex; ++k)
    {
        const buffer<skinning_influence>& influence = skinning.influence[k];

        // Transformation matrix for skinning
        mat4 M = mat4::zero();
        for(size_t kb=0; kb<influence.size(); ++kb) // Loop over influencing joints
        {
            const int idx = influence[kb].joint;
            const float w = influence[kb].weight;

            const quaternion& r = skeleton_current[idx].r;
            const vec3& p = skeleton_current[idx].p;
            const quaternion& r0 = skeleton_rest_pose[idx].r;
            const vec3& p0 = skeleton_rest_pose[idx].p;

            // Convert rotation/translation to matrix
            mat4 T = mat4::from_mat3_vec3(r.matrix(), p);
            mat4 T0_inv = mat4::from_mat3_vec3(conjugate(r0).matrix(), conjugate(r0).apply(-p0)); // inverse

            // Skinning
            M += w*T*T0_inv;
        }

        // Apply skinning transform on vertex
        const vec3& p0 = skinning.rest_pose[k];
        const vec4 p1 = M * vec4(p0.x,p0.y,p0.z,1.0f);
        skinning.deformed.position[k] = {p1.x,p1.y,p1.z};

        const vec3& n0 = skinning.rest_pose_normal[k];
        const vec4 n1 = M * vec4(n0.x, n0.y, n0.z, 0.0f);
        skinning.deformed.normal[k] = {n1.x, n1.y, n1.z};
    }

    squashy_skinning();



}


void scene_model::compute_skinning_dual_quaternion()
{
    const size_t N_vertex = skinning.rest_pose.size();
    for(size_t k=0; k<N_vertex; ++k)
    {
        const buffer<skinning_influence>& influence = skinning.influence[k];

        quaternion_dual d = { {0,0,0,0}, {0,0,0,0} };
        for(size_t kb=0; kb<influence.size(); ++kb) // Loop over influencing joints
        {
            const int idx = influence[kb].joint;
            float w = influence[kb].weight;

            quaternion const& r = skeleton_current[idx].r;
            vec3 const& p = skeleton_current[idx].p;
            quaternion const& r0 = skeleton_rest_pose[idx].r;
            vec3 const& p0 = skeleton_rest_pose[idx].p;

            quaternion const q_deform = r * conjugate(r0);
            vec3 const p_deform = (r*conjugate(r0)).apply(-p0) + p;

            d += w * quaternion_dual(q_deform, p_deform);

        }

        float const n = norm(d.q);
        if(std::abs(n)>1e-5f)
            d = d/n;

        vec3 const& p0 = skinning.rest_pose[k];
        skinning.deformed.position[k] = (d.q).apply(p0) + d.translation();

        vec3 const& n0 = skinning.rest_pose_normal[k];
        skinning.deformed.normal[k] = d.q.apply(n0);

    }

    squashy_skinning();
}



// Convert skeleton from local to global coordinates
buffer<joint_geometry> local_to_global(const buffer<joint_geometry>& local, const buffer<joint_connectivity>& connectivity)
{
    const size_t N = connectivity.size();
    assert(local.size()==connectivity.size());
    std::vector<joint_geometry> global;
    global.resize(N);
    global[0] = local[0];

    // T_global = T_global^parent * T_local (T: 4x4 transformation matrix)
    //   => R_global = R_global^parent * R_local
    //   => P_global = R_global^parent * P_local + P_global^parent
    for(size_t k=1; k<N; ++k)
    {
        const int parent = connectivity[k].parent;
        global[k].r = global[parent].r * local[k].r;
        global[k].p = global[parent].r.apply(local[k].p) + global[parent].p;
    }

    return global;
}


void display_skeleton(const buffer<joint_geometry>& skeleton_geometry,
                      const buffer<joint_connectivity>& skeleton_connectivity,
                      const std::map<std::string,GLuint>& shaders,
                      const scene_structure& scene,
                      segment_drawable_immediate_mode& segment_drawer)
{
    const size_t N = skeleton_geometry.size();
    for(size_t k=1; k<N; ++k)
    {
        int parent = skeleton_connectivity[k].parent;
        const vec3& p1 = skeleton_geometry[parent].p;
        const vec3& p2 = skeleton_geometry[k].p;

        segment_drawer.uniform_parameter.p1 = p1;
        segment_drawer.uniform_parameter.p2 = p2;
        segment_drawer.draw(shaders.at("segment_im"),scene.camera);
    }
}

void display_joints(const buffer<joint_geometry>& skeleton_geometry,
                    const scene_structure& scene,
                    mesh_drawable& sphere)
{
    const size_t N = skeleton_geometry.size();
    for(size_t k=0; k<N; ++k)
    {
        sphere.uniform.transform.translation = skeleton_geometry[k].p;
        draw(sphere, scene.camera);
    }

}

void display_frames(const buffer<joint_geometry>& skeleton_geometry,
                    const scene_structure& scene,
                    mesh_drawable& frame)
{
    const size_t N = skeleton_geometry.size();
    for(size_t k=0; k<N; ++k)
    {
        frame.uniform.transform.rotation = skeleton_geometry[k].r.matrix();
        frame.uniform.transform.translation = skeleton_geometry[k].p;
        draw(frame, scene.camera);
    }
}


void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{

    timer.update();
    set_gui();
    const float t = timer.t;

    const auto skeleton_geometry_local  = interpolate_skeleton_at_time(t, skeleton.anim, gui_param.interpolate);

    if(!is_interactive)
    {
        skeleton_previous = skeleton_current;
        skeleton_current = local_to_global(skeleton_geometry_local, skeleton.connectivity);
    }

    for(int k=0; k<int(skeleton_current.size()); ++k) {
        skeleton_velocity_tracker[k].add( skeleton_current[k].p, t );
        skeleton_speed[k] = skeleton_velocity_tracker[k].speed_avg();
        skeleton_acceleration[k] = skeleton_velocity_tracker[k].acceleration_avg();
    }


    if(gui_param.display_rest_pose)
        skeleton_current = skeleton_rest_pose;


    if(gui_param.dual_quaternion)
        compute_skinning_dual_quaternion();
    else
        compute_skinning();
    character_visual.update_position(skinning.deformed.position);




    character_visual.update_normal(skinning.deformed.normal);

    if(gui_param.display_skeleton_bones)
        display_skeleton(skeleton_current, skeleton.connectivity, shaders, scene, segment_drawer);
    if(gui_param.display_skeleton_joints)
        display_joints(skeleton_current, scene, sphere);
    if(gui_param.display_skeleton_frames)
        display_frames(skeleton_current, scene, frame);


    if(gui_param.display_mesh) {
        glPolygonOffset( 1.0, 1.0 );
        GLuint const texture_id = (gui_param.display_texture? character_visual.texture_id : scene.texture_white);
        draw(character_visual, scene.camera, character_visual.shader, texture_id);
    }
    if(gui_param.display_wireframe) {
        glPolygonOffset( 1.0, 1.0 );
        draw(character_visual, scene.camera, shaders["wireframe_quads"]);
    }
}


void scene_model::set_gui()
{
    ImGui::SliderFloat("Timer",  &timer.t, timer.t_min, timer.t_max, "%.2f s");
    ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 6.0f, "%.2f s");

    ImGui::Text("Display Mesh:");
    ImGui::Checkbox("Mesh", &gui_param.display_mesh); ImGui::SameLine();
    ImGui::Checkbox("Wireframe", &gui_param.display_wireframe); ImGui::SameLine();
    ImGui::Checkbox("Texture", &gui_param.display_texture);

    ImGui::Text("Display Skeleton:");
    ImGui::Checkbox("Bones", &gui_param.display_skeleton_bones);  ImGui::SameLine();
    ImGui::Checkbox("Joints", &gui_param.display_skeleton_joints);  ImGui::SameLine();
    ImGui::Checkbox("Frames", &gui_param.display_skeleton_frames);


    ImGui::Text("Shape type:");
    bool click_sphere = ImGui::RadioButton("Sphere", &gui_param.display_type, display_sphere); ImGui::SameLine();
    bool click_cylinder = ImGui::RadioButton("Cylinder", &gui_param.display_type, display_cylinder); ImGui::SameLine();
    bool click_bar = ImGui::RadioButton("Bar", &gui_param.display_type, display_bar); ImGui::SameLine();
    bool click_character = ImGui::RadioButton("Character", &gui_param.display_type, display_character);

    if(click_sphere)  load_sphere_data(skeleton, skinning, character_visual, timer, shader_mesh);
    if(click_cylinder)  load_cylinder_data(skeleton, skinning, character_visual, timer, shader_mesh);
    if(click_bar)       load_rectangle_data(skeleton, skinning, character_visual, timer, shader_mesh);
    if(click_character) load_character_data(skeleton, skinning, character_visual, timer, shader_mesh);

    if(click_cylinder || click_bar || click_character) {
        const auto skeleton_geometry_local  = interpolate_skeleton_at_time(0, skeleton.anim, gui_param.interpolate);
        skeleton_current = local_to_global(skeleton_geometry_local, skeleton.connectivity);
        skeleton_speed.resize(skeleton_current.size());
        skeleton_acceleration.resize(skeleton_current.size());
        skeleton_velocity_tracker.resize(skeleton_current.size());
        skeleton_rest_pose = local_to_global(skeleton.rest_pose, skeleton.connectivity);
    }

    ImGui::Checkbox("Interactive", &is_interactive);

    ImGui::Checkbox("Rest pose", &gui_param.display_rest_pose);
    ImGui::Checkbox("Dual quaternion", &gui_param.dual_quaternion);
    ImGui::Checkbox("Interpolate skeleton", &gui_param.interpolate);

    // Start and stop animation
    bool const stop  = ImGui::Button("Stop"); ImGui::SameLine();
    bool const start = ImGui::Button("Start"); ImGui::SameLine();

    if(stop)  timer.stop();
    if(start) timer.start();
}


void scene_model::mouse_move(scene_structure& scene, GLFWwindow* window)
{
    const vec2 cursor = glfw_cursor_coordinates_window(window);

    // Check that the mouse is clicked (drag and drop)
    const bool mouse_click_left  = glfw_mouse_pressed_left(window);
    const bool key_shift = glfw_key_shift_pressed(window);



    // Selection
    if(!mouse_click_left && key_shift)
    {
        // Create the 3D ray passing by the selected point on the screen
        const ray r = picking_ray(scene.camera, cursor);

        // Check if this ray intersects a position (represented by a sphere)
        //  Loop over all positions and get the intersected position (the closest one in case of multiple intersection)

        picked_object = -1;
        float distance_min = 0.0f;

        for(int k_joint=0; k_joint<int(skeleton_current.size()); ++k_joint) {
            const vec3& c = skeleton_current[k_joint].p;
            const picking_info info = ray_intersect_sphere(r, c, 0.05f);

            if( info.picking_valid ) // the ray intersects a sphere
            {
                const float distance = norm(info.intersection-r.p); // get the closest intersection
                if( picked_object==-1 || distance<distance_min ){
                    picked_object = k_joint;
                    distance_min = distance;
                }
            }
        }

    }

    // Displacement
    if(mouse_click_left && key_shift && picked_object!=-1)
    {
        // Translate the selected object to the new pointed mouse position within the camera plane
        // ************************************************************************************** //

        // Get vector orthogonal to camera orientation
        //const mat4 M = scene.camera.camera_matrix();
        //const vec3 n = {M(0,2),M(1,2),M(2,2)};

        const vec2 cursor_tr = cursor-cursor_prev;
        const vec3 tr = scene.camera.orientation * vec3(cursor_tr.x, cursor_tr.y, 0.0f);

        vec3& p0 = skeleton_current[picked_object].p;
        p0 += tr;

//        // Compute intersection between current ray and the plane orthogonal to the view direction and passing by the selected object
//        const ray r = picking_ray(scene.camera, cursor);
//        vec3& p0 = skeleton_current[picked_object].p;
//        const picking_info info = ray_intersect_plane(r,n,p0);

        // translate the position
        //p0 = info.intersection;
    }

    cursor_prev = cursor;
}


#endif
