
#include "skinning.hpp"
#ifdef SCENE_SQUASHY_SKINNING

#include "skinning_loader.hpp"
#include "helper.hpp"

using namespace vcl;



/** This function is work in progress ... */
// Compute the flappy example
void scene_model::flappy_skinning()
{
    if(weight_flappy.size()!=skinning.rest_pose.size() || weight_squashy.size()!=skinning.rest_pose.size())
        return ;



    const size_t N_vertex = skinning.rest_pose.size();

    for(size_t k=0; k<N_vertex; ++k) {
        const buffer<skinning_influence>& influence = skinning.influence[k];
        const vec3& p_lbs = skinning.deformed.position[k];

        vec3 const& n = skinning.deformed.normal[k];

        // Transformation matrix for skinning
        //mat4 M = mat4::zero();
        vec3 p = {0,0,0};
        for(size_t kb=0; kb<influence.size(); ++kb) // Loop over influencing joints
        {

            const int idx = influence[kb].joint;
            const float w = influence[kb].weight;

            vec3 v = skeleton_speed[idx];
            vec3 acceleration = skeleton_acceleration[idx]*0.4f;

            float an = norm(acceleration);
            if(an>1.0f)
                an = 1.0f;

            float const lambda = an * flapping_power;

            if(norm(v)>2)
                v = v/norm(v)*2;

            const float a = dot(n,v) * 0.02f;

            //Flappy / Squashy
            const vec3 flappy = - lambda * acceleration / 50.0f;
            p += w*(  p_lbs + weight_flappy[k]*flappy + weight_squashy[k]*squashing_power * a * v );



//            if(k==0)
//                std::cout<<v<<std::endl;







            //            //Squashy
            //            const mat3 S = mat3(1.0f+lambda,0,0,
            //                                0,std::sqrt(1.0f/(1+lambda)),0,
            //                                0,0,std::sqrt(1.0f/(1+lambda)));
            //            const mat3 R = rotation_between_vector_mat3({1,0,0}, normalize(v));
            //            vec3 p_c = skeleton_current[idx].p;



            // Hack cylinder
            //            if(skeleton_current.size()>2) {
            //                if(idx==0)
            //                    p_c = 0.5f*(skeleton_current[0].p+skeleton_current[1].p);
            //                if(idx==1)
            //                    p_c = 0.5f*(skeleton_current[1].p+skeleton_current[2].p);
            //            }

            //p += w*(  R*S*transpose(R)*  (p_lbs-p_c)+p_c);
        }

        skinning.deformed.position[k] = p;
    }



//    // Rotation
//    {
//        const vec3 speed0 = skeleton_speed[0];
//        const vec3 p0 = skeleton_current[0].p;
//        const mat3 R = rotation_between_vector_mat3({1,0,0}, -speed0);
//        for(int k=0; k<skinning.deformed.position.size(); ++k) {
//            const vec3 p = skinning.deformed.position[k];
//            skinning.deformed.position[k] = R * (p-p0)+p0;
//        }
//    }
}

/** This function is work in progress ... */
// Compute the squashy skinning
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

            float const lambda = vn * squashing_power;


            //Squashy
            const mat3 S = mat3(1.0f+lambda,0,0,
                                0,std::sqrt(1.0f/(1+lambda)),0,
                                0,0,std::sqrt(1.0f/(1+lambda)));
            const mat3 R = rotation_between_vector_mat3({1,0,0}, normalize(v));
            vec3 p_c = skeleton_current[idx].p;



            // Hack cylinder
            //            if(skeleton_current.size()>2) {
            //                if(idx==0)
            //                    p_c = 0.5f*(skeleton_current[0].p+skeleton_current[1].p);
            //                if(idx==1)
            //                    p_c = 0.5f*(skeleton_current[1].p+skeleton_current[2].p);
            //            }

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

    if(is_flapping)
        flappy_skinning();
    else
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

    if(is_flapping)
        flappy_skinning();
    else
        squashy_skinning();
}









void scene_model::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{

    timer.update();
    set_gui();
    const float t = timer.t;


    if(!is_interactive)
    {
        skeleton_local_current = interpolate_skeleton_at_time(t, skeleton.anim, gui_param.interpolate);
    }

    skeleton_current = local_to_global(skeleton_local_current, skeleton.connectivity);

    for(int k=0; k<int(skeleton_current.size()); ++k) {
        skeleton_velocity_tracker[k].add( skeleton_current[k].p, t );
        //skeleton_angular_velocity_tracker[k].add( skeleton_current[k].r, t );

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



//    sphere.uniform.transform.translation = vec3(0,0.05,0);
//    sphere.uniform.transform.scaling = 0.25f/0.005f;
//    draw(sphere, scene.camera);
//    sphere.uniform.transform.scaling = 1.0f;


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

    ImGui::SliderFloat("Flapping power", &flapping_power, 0.0f, 20.0f, "%.2f s", 2.0f);
    ImGui::SliderFloat("Squashing power", &squashing_power, 0.0f, 20.0f, "%.2f s", 2.0f);

    ImGui::Text("Display Mesh:");
    ImGui::Checkbox("Mesh", &gui_param.display_mesh); ImGui::SameLine();
    ImGui::Checkbox("Wireframe", &gui_param.display_wireframe); ImGui::SameLine();
    ImGui::Checkbox("Texture", &gui_param.display_texture);

    ImGui::Text("Display Skeleton:");
    ImGui::Checkbox("Bones", &gui_param.display_skeleton_bones);  ImGui::SameLine();
    ImGui::Checkbox("Joints", &gui_param.display_skeleton_joints);  ImGui::SameLine();
    ImGui::Checkbox("Frames", &gui_param.display_skeleton_frames);


    ImGui::Text("Shape type:");
    bool click_sphere    = ImGui::RadioButton("Sphere", &gui_param.display_type, display_sphere); ImGui::SameLine();
    bool click_cylinder_bending   = ImGui::RadioButton("Cylinder Bending", &gui_param.display_type, display_cylinder_bending); ImGui::SameLine();
    bool click_cylinder_translate = ImGui::RadioButton("Cylinder Translate", &gui_param.display_type, display_cylinder_translate); ImGui::SameLine();
    bool click_rondinella = ImGui::RadioButton("Rondinella", &gui_param.display_type, display_rondinella);

    bool click_bar       = ImGui::RadioButton("Bar", &gui_param.display_type, display_bar); ImGui::SameLine();
    bool click_character = ImGui::RadioButton("Character", &gui_param.display_type, display_character);


    if(click_sphere)    load_sphere_data(skeleton, skinning, weight_flappy, character_visual, timer, shader_mesh);
    if(click_cylinder_bending)  load_bending_cylinder_data(skeleton, skinning, character_visual, timer, shader_mesh);
    if(click_cylinder_translate)  load_diagonal_translate_cylinder_data(skeleton, skinning, weight_flappy, character_visual, timer, shader_mesh);
    if(click_rondinella)  load_rondinella_data(skeleton, skinning, weight_flappy, weight_squashy, character_visual, timer, shader_mesh);

    if(click_bar)       load_rectangle_data(skeleton, skinning, character_visual, timer, shader_mesh);
    if(click_character) load_character_data(skeleton, skinning, weight_flappy, character_visual, timer, shader_mesh);

    if(click_sphere || click_cylinder_bending || click_cylinder_translate || click_rondinella || click_bar || click_character) {
        resize_structure();
    }

    ImGui::Checkbox("Interactive", &is_interactive);

    ImGui::Checkbox("Rest pose", &gui_param.display_rest_pose);
    ImGui::Checkbox("Dual quaternion", &gui_param.dual_quaternion);
    ImGui::Checkbox("Interpolate skeleton", &gui_param.interpolate);

    ImGui::Checkbox("Flapping", &is_flapping);
//    ImGui::Checkbox("speed", &is_speed);
//    ImGui::Checkbox("acceleration", &is_acceleration);


    if( ImGui::Button("Color white") ) {
        for(int k=0; k<skinning.deformed.position.size(); ++k) {
            skinning.deformed.color[k] = {1.0f, 1.0f, 1.0f, 0};
            character_visual.update_color(skinning.deformed.color);
        }
    }
    ImGui::SameLine();
    if( ImGui::Button("Color flappy") ) {
        for(int k=0; k<skinning.deformed.position.size(); ++k) {
            skinning.deformed.color[k] = {weight_flappy[k],0,0,0};
            character_visual.update_color(skinning.deformed.color);
        }
    }
    ImGui::SameLine();
    if( ImGui::Button("Color squashy") ) {
        for(int k=0; k<skinning.deformed.position.size(); ++k) {
            skinning.deformed.color[k] = {0,weight_squashy[k],0,0};
            character_visual.update_color(skinning.deformed.color);
        }
    }

    // Start and stop animation
    bool const stop  = ImGui::Button("Stop"); ImGui::SameLine();
    bool const start = ImGui::Button("Start"); ImGui::SameLine();

    if(stop)  timer.stop();
    if(start) timer.start();
}


void scene_model::resize_structure()
{
    skeleton_local_current  = interpolate_skeleton_at_time(0, skeleton.anim, gui_param.interpolate);
    skeleton_current = local_to_global(skeleton_local_current, skeleton.connectivity);
    skeleton_speed.resize(skeleton_current.size());
    skeleton_acceleration.resize(skeleton_current.size());
    skeleton_velocity_tracker.resize(skeleton_current.size());
    skeleton_rest_pose = local_to_global(skeleton.rest_pose, skeleton.connectivity);
    skeleton_angular_velocity_tracker.resize(skeleton_current.size());

    if(weight_squashy.size()!=skinning.rest_pose.size()){
        weight_squashy.resize(skinning.rest_pose.size());
        weight_squashy.fill(1.0f);
    }
    if(weight_flappy.size()!=skinning.rest_pose.size()){
        weight_flappy.resize(skinning.rest_pose.size());
        weight_flappy.fill(0.0f);
    }

}

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
    load_sphere_data(skeleton, skinning, weight_flappy, character_visual, timer, shader_mesh);

    resize_structure();

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

        vec3& p0 = skeleton_local_current[picked_object].p;
        p0 += tr;

    }

    cursor_prev = cursor;
}


#endif
