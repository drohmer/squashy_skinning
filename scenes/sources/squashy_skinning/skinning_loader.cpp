#include "skinning_loader.hpp"
#ifdef SCENE_SQUASHY_SKINNING

#include <fstream>
#include <sstream>

using namespace vcl;


void load_sphere_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, vcl::mesh_drawable& shape_visual, vcl::timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    skeleton.connectivity = { {-1,"joint_0"} };

    quaternion q0 = quaternion::axis_angle(normalize(vec3{0,0,1}),0.0f);      // no rotation

    joint_geometry g0   = {{0.0f,0.0f,0.0f},q0};
    joint_geometry g1   = {{0.0f,0.0f,-1.0f},q0};
    joint_geometry g2   = {{0.0f,0.0f,0.0f},q0};

    // Skeleton at rest shape
    skeleton.rest_pose = { g0 };

    std::vector<joint_geometry_time> anim_g0 = {{0,g0},{1,g1},{2,g2}};

    skeleton.anim = { anim_g0 };


    // Cylinder shape
    mesh sphere;
    const size_t N=50;
    const float r = 0.1f;
    skinning.influence.clear();
    for(size_t ku=0; ku<N; ++ku)
    {
        for(size_t kv=0; kv<N; ++kv)
        {
            const float u = ku/float(N-1.0f);
            const float v = kv/float(N);

            const float theta = float(M_PI)* v;
            const float phi = 2*float(M_PI)* u;

            const float x = r * std::sin(theta) * std::cos(phi);
            const float y = r * std::sin(theta) * std::sin(phi);
            const float z = r * std::cos(theta);
            const vec3 p = {x,y,z};

            const vec3 n = p / norm(p);

            sphere.position.push_back(p);
            sphere.normal.push_back(n);
        }
    }
    sphere.connectivity = connectivity_grid(N,N,false,true);


    // Skinning weights
    for(size_t ku=0; ku<N; ++ku)
    {
        //const float u = ku/float(N-1.0f);
        for(size_t kv=0; kv<N; ++kv)
        {
            skinning.influence.push_back( {{0, 1.0f}} );
        }
    }

    weight_flappy.resize(sphere.position.size());
    weight_flappy.fill(0.0f);

    skinning.rest_pose = sphere.position;
    skinning.rest_pose_normal = sphere.normal;
    skinning.deformed  = sphere;


    shape_visual.clear();
    shape_visual = mesh_drawable(sphere);
    shape_visual.shader = shader;

    timer = timer_interval();
    timer.t_max = 2.0f;
}

void load_rondinella_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, vcl::buffer<float>& weight_squashy,vcl::mesh_drawable& shape_visual, vcl::timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    skeleton.connectivity = { {-1,"joint_0"} };

    quaternion q0 = quaternion::axis_angle(normalize(vec3{0,0,1}),0.0f);      // no rotation

    joint_geometry g0   = {{0.0f,0.0f,0.0f},q0};
    joint_geometry g1   = {{0.0f,0.0f,-1.0f},q0};
    joint_geometry g2   = {{0.0f,0.0f,0.0f},q0};

    // Skeleton at rest shape
    skeleton.rest_pose = { g0 };

    std::vector<joint_geometry_time> anim_g0 = {{0,g0},{1,g1},{2,g2}};

    skeleton.anim = { anim_g0 };


    // Cylinder shape
    mesh sphere;
    sphere = mesh_load_file_obj("rondinella.obj");

    for(int k=0; k<sphere.position.size(); ++k)
        sphere.position[k] /= 5.0f;

    mat3 R = rotation_from_axis_angle_mat3({1,0,0}, -M_PI/2.0f);
    for(int k=0; k<sphere.position.size(); ++k)
        sphere.position[k] = R*sphere.position[k];

    sphere.fill_empty_fields();

//    const size_t N=50;
//    const float r = 0.1f;
//    skinning.influence.clear();
//    for(size_t ku=0; ku<N; ++ku)
//    {
//        for(size_t kv=0; kv<N; ++kv)
//        {
//            const float u = ku/float(N-1.0f);
//            const float v = kv/float(N);

//            const float theta = float(M_PI)* v;
//            const float phi = 2*float(M_PI)* u;

//            const float x = r * std::sin(theta) * std::cos(phi);
//            const float y = r * std::sin(theta) * std::sin(phi);
//            const float z = r * std::cos(theta);
//            const vec3 p = {x,y,z};

//            const vec3 n = p / norm(p);

//            sphere.position.push_back(p);
//            sphere.normal.push_back(n);
//        }
//    }
//    sphere.connectivity = connectivity_grid(N,N,false,true);


    // Skinning weights
    int const N = sphere.position.size();
    for(size_t k=0; k<N; ++k)
    {
        skinning.influence.push_back( {{0, 1.0f}} );
    }

    weight_flappy.resize(0);
    weight_squashy.resize(0);
    for(size_t k=0; k<N; ++k)
    {
        const vec3& p = sphere.position[k];

        const vec3& pc = p-vec3(0,0.05f,0);


        float distance = norm(pc);



        float w_flappy = 0.0f;
        float w_squashy = 0.0f;

        float threshold = 0.25f;
        if(distance<threshold) {
            w_flappy  = 0.0f;
            w_squashy = 1.0f;
        }
        else{
            float s = 1.0f/(1 + 10*(distance-threshold));
            w_squashy = s;
            w_flappy = 1-s;
/*            w_flappy = (distance-thet)
                    w_squashy
            w = 5*(n-threshold);
            w_squashy = 0.0f;*///1.0f/(n-threshold+1);
        }
        weight_flappy.push_back(w_flappy);
        weight_squashy.push_back(w_squashy);
    }

    skinning.rest_pose = sphere.position;
    skinning.rest_pose_normal = sphere.normal;
    skinning.deformed  = sphere;


    shape_visual.clear();
    shape_visual = mesh_drawable(sphere);
    shape_visual.shader = shader;

    timer = timer_interval();
    timer.t_max = 2.0f;
}

void load_diagonal_translate_cylinder_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, mesh_drawable& shape_visual, timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    skeleton.connectivity = { {-1,"joint_0"},
                              { 0,"joint_1"},
                              { 1,"joint_2"} };

    quaternion q0 = quaternion::axis_angle(normalize(vec3{0,0,1}),0.0f);       // no rotation

    joint_geometry g0_0   = {{ 0.0f,0.0f,0.0f},q0}; // First joint
    joint_geometry g0_1   = {{ 0.25f,0.25f,0.0f},q0};
    joint_geometry g1     = {{0.5f,0.0f,0.0f},q0}; // Second joint, frame 0
    joint_geometry g2     = {{0.5f,0.0f,0.0f},q0}; // Third joint (only used to display bone)

    // Skeleton at rest shape
    skeleton.rest_pose = { g0_0, g1, g2 };

    // Anim of cylinder skeleton
    //  Pose 0: straight
    //  Pose 1: First joint is rotated of pi/2 around z axis
    //  Pose 2: straight, similar to pose 0
    std::vector<joint_geometry_time> anim_g0 = {{0,g0_0},{1,g0_1},{2,g0_0}};
    std::vector<joint_geometry_time> anim_g1 = {{0,g1},{1,g1},{2,g1}};
    std::vector<joint_geometry_time> anim_g2 = {{0,g2},{1,g2},{2,g2}};
    skeleton.anim = {anim_g0, anim_g1, anim_g2};

    // Cylinder shape
    mesh cylinder;
    const size_t N=50;
    const float r = 0.1f;
    skinning.influence.clear();
    for(size_t ku=0; ku<N; ++ku)
    {
        for(size_t kv=0; kv<N; ++kv)
        {
            const float u = ku/float(N-1.0f);
            const float v = kv/float(N);

            const float theta = 2*float(M_PI)* v;

            const vec3 p = {u, r*std::cos(theta), r*std::sin(theta)};
            const vec3 n = {0, std::cos(theta), std::sin(theta)};

            cylinder.position.push_back(p);
            cylinder.normal.push_back(n);
        }
    }
    cylinder.connectivity = connectivity_grid(N,N,false,true);


    weight_flappy.resize(cylinder.position.size());
    weight_flappy.fill(0.0f);

    // Skinning weights
    for(size_t ku=0; ku<N; ++ku)
    {
        const float u = ku/float(N-1.0f);
        for(size_t kv=0; kv<N; ++kv)
        {
            float w0, w1;
            const float alpha = 3.0f; // power for skinning weights evolution
            if(u<0.5f) {
                w1 = 0.5f*std::pow(u/0.5f, alpha);
                w0 = 1-w1;
            }
            else {
                w0 = 0.5f*std::pow(1-(u-0.5f)/0.5f, alpha);
                w1 = 1-w0;
            }

            skinning_influence influence_bone_0 = {0, w0};
            skinning_influence influence_bone_1 = {1, w1};
            skinning.influence.push_back( {influence_bone_0, influence_bone_1} );
        }
    }

    skinning.rest_pose = cylinder.position;
    skinning.rest_pose_normal = cylinder.normal;
    skinning.deformed  = cylinder;


    shape_visual.clear();
    shape_visual = mesh_drawable(cylinder);
    shape_visual.shader = shader;

    timer = timer_interval();
    timer.t_max = 2.0f;
}

void load_bending_cylinder_data(skeleton_structure& skeleton, skinning_structure& skinning, mesh_drawable& shape_visual, timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    skeleton.connectivity = { {-1,"joint_0"},
                              { 0,"joint_1"},
                              { 1,"joint_2"} };

    quaternion q0 = quaternion::axis_angle(normalize(vec3{0,0,1}),0.0f);       // no rotation
    quaternion q1 = quaternion::axis_angle(normalize(vec3{0,0,1}),3.14f/2.0f*0.8);
    quaternion q2 = quaternion::axis_angle(normalize(vec3{0,0,1}),3.14f/2.0f); // rotation of pi/2 around z-axis


    joint_geometry g0   = {{0.0f,0.0f,0.0f},q0}; // First joint
    joint_geometry g1_0 = {{0.5f,0.0f,0.0f},q0}; // Second joint, frame 0
    joint_geometry g1_1 = {{0.5f,0.0f,0.0f},q1}; // Second joint, frame 1
    joint_geometry g1_2 = {{0.5f,0.0f,0.0f},q2}; // Second joint, frame 2
    joint_geometry g2   = {{0.5f,0.0f,0.0f},q0}; // Third joint (only used to display bone)

    // Skeleton at rest shape
    skeleton.rest_pose = { g0, g1_0, g2 };

    // Anim of cylinder skeleton
    //  Pose 0: straight
    //  Pose 1: First joint is rotated of pi/2 around z axis
    //  Pose 2: straight, similar to pose 0
    std::vector<joint_geometry_time> anim_g0 = {{0,g0},{1,g0},{2,g0},{3,g0}};
    std::vector<joint_geometry_time> anim_g1 = {{0,g1_0},{1,g1_1},{2,g1_2},{3,g1_0}};
    std::vector<joint_geometry_time> anim_g2 = {{0,g2},{1,g2},{2,g2},{3,g2}};
    skeleton.anim = {anim_g0, anim_g1, anim_g2};

    // Cylinder shape
    mesh cylinder;
    const size_t N=50;
    const float r = 0.1f;
    skinning.influence.clear();
    for(size_t ku=0; ku<N; ++ku)
    {
        for(size_t kv=0; kv<N; ++kv)
        {
            const float u = ku/float(N-1.0f);
            const float v = kv/float(N);

            const float theta = 2*float(M_PI)* v;

            const vec3 p = {u, r*std::cos(theta), r*std::sin(theta)};
            const vec3 n = {0, std::cos(theta), std::sin(theta)};

            cylinder.position.push_back(p);
            cylinder.normal.push_back(n);
        }
    }
    cylinder.connectivity = connectivity_grid(N,N,false,true);


    // Skinning weights
    for(size_t ku=0; ku<N; ++ku)
    {
        const float u = ku/float(N-1.0f);
        for(size_t kv=0; kv<N; ++kv)
        {
            float w0, w1;
            const float alpha = 3.0f; // power for skinning weights evolution
            if(u<0.5f) {
                w1 = 0.5f*std::pow(u/0.5f, alpha);
                w0 = 1-w1;
            }
            else {
                w0 = 0.5f*std::pow(1-(u-0.5f)/0.5f, alpha);
                w1 = 1-w0;
            }

            skinning_influence influence_bone_0 = {0, w0};
            skinning_influence influence_bone_1 = {1, w1};
            skinning.influence.push_back( {influence_bone_0, influence_bone_1} );
        }
    }

    skinning.rest_pose = cylinder.position;
    skinning.rest_pose_normal = cylinder.normal;
    skinning.deformed  = cylinder;


    shape_visual.clear();
    shape_visual = mesh_drawable(cylinder);
    shape_visual.shader = shader;

    timer = timer_interval();
    timer.t_max = 3.0f;
}

void load_cylinder_data(skeleton_structure& skeleton, skinning_structure& skinning, mesh_drawable& shape_visual, timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    skeleton.connectivity = { {-1,"joint_0"},
                              { 0,"joint_1"},
                              { 1,"joint_2"} };

    quaternion q0 = quaternion::axis_angle(normalize(vec3{0,0,1}),0.0f);       // no rotation
    quaternion q1 = quaternion::axis_angle(normalize(vec3{0,0,1}),3.14f/2.0f); // rotation of pi/2 around z-axis


    joint_geometry g0   = {{0.0f,0.0f,0.0f},q0}; // First joint
    joint_geometry g1_0 = {{0.5f,0.0f,0.0f},q0}; // Second joint, frame 0
    joint_geometry g1_1 = {{0.5f,0.0f,0.0f},q1}; // Second joint, frame 1
    joint_geometry g2   = {{0.5f,0.0f,0.0f},q0}; // Third joint (only used to display bone)

    // Skeleton at rest shape
    skeleton.rest_pose = { g0, g1_0, g2 };

    // Anim of cylinder skeleton
    //  Pose 0: straight
    //  Pose 1: First joint is rotated of pi/2 around z axis
    //  Pose 2: straight, similar to pose 0
    std::vector<joint_geometry_time> anim_g0 = {{0,g0},{1,g0},{2,g0}};
    std::vector<joint_geometry_time> anim_g1 = {{0,g1_0},{1,g1_1},{2,g1_0}};
    std::vector<joint_geometry_time> anim_g2 = {{0,g2},{1,g2},{2,g2}};
    skeleton.anim = {anim_g0,anim_g1,anim_g2};

    // Cylinder shape
    mesh cylinder;
    const size_t N=50;
    const float r = 0.1f;
    skinning.influence.clear();
    for(size_t ku=0; ku<N; ++ku)
    {
        for(size_t kv=0; kv<N; ++kv)
        {
            const float u = ku/float(N-1.0f);
            const float v = kv/float(N);

            const float theta = 2*float(M_PI)* v;

            const vec3 p = {u, r*std::cos(theta), r*std::sin(theta)};
            const vec3 n = {0, std::cos(theta), std::sin(theta)};

            cylinder.position.push_back(p);
            cylinder.normal.push_back(n);
        }
    }
    cylinder.connectivity = connectivity_grid(N,N,false,true);


    // Skinning weights
    for(size_t ku=0; ku<N; ++ku)
    {
        const float u = ku/float(N-1.0f);
        for(size_t kv=0; kv<N; ++kv)
        {
            float w0, w1;
            const float alpha = 3.0f; // power for skinning weights evolution
            if(u<0.5f) {
                w1 = 0.5f*std::pow(u/0.5f, alpha);
                w0 = 1-w1;
            }
            else {
                w0 = 0.5f*std::pow(1-(u-0.5f)/0.5f, alpha);
                w1 = 1-w0;
            }

            skinning_influence influence_bone_0 = {0, w0};
            skinning_influence influence_bone_1 = {1, w1};
            skinning.influence.push_back( {influence_bone_0, influence_bone_1} );
        }
    }

    skinning.rest_pose = cylinder.position;
    skinning.rest_pose_normal = cylinder.normal;
    skinning.deformed  = cylinder;


    shape_visual.clear();
    shape_visual = mesh_drawable(cylinder);
    shape_visual.shader = shader;

    timer = timer_interval();
    timer.t_max = 2.0f;

}


void load_rectangle_data(skeleton_structure& skeleton, skinning_structure& skinning, mesh_drawable& shape_visual, timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    skeleton.connectivity = { {-1,"joint_0"},
                              { 0,"joint_1"},
                              { 1,"joint_2"} };

    quaternion q0 = quaternion::axis_angle(normalize(vec3{0,0,1}),0.0f);      // no rotation
    quaternion q1 = quaternion::axis_angle(normalize(vec3{1,0,0}),3.14f); // rotation of pi around x-axis

    joint_geometry g0   = {{0.0f,0.0f,0.0f},q0}; // First joint
    joint_geometry g1_0 = {{0.5f,0.0f,0.0f},q0}; // Second joint, frame 0
    joint_geometry g1_1 = {{0.5f,0.0f,0.0f},q1}; // Second joint, frame 1
    joint_geometry g2   = {{0.5f,0.0f,0.0f},q0}; // Third joint (only used to display bone)

    // Skeleton at rest shape
    skeleton.rest_pose = { g0, g1_0, g2 };

    // Anim of cylinder skeleton
    std::vector<joint_geometry_time> anim_g0 = {{0,g0},{1,g0},{2,g0}};
    std::vector<joint_geometry_time> anim_g1 = {{0,g1_0},{1,g1_1},{2,g1_0}};
    std::vector<joint_geometry_time> anim_g2 = {{0,g2},{1,g2},{2,g2}};
    skeleton.anim = {anim_g0,anim_g1,anim_g2};

    // Cylinder shape
    float const r = 0.1f;
    mesh shape;
    shape = mesh_primitive_bar_grid(50,10, 10,{0,-r,-r},{1,0,0},{0,2*r,0},{0,0,2*r});
    skinning.influence.clear();

    // Skinning weights
    size_t const N = shape.position.size();
    for(size_t k=0; k<N; ++k) {
        float const u = shape.position[k].x;

        float w0, w1;
        const float alpha = 3.0f; // power for skinning weights evolution
        if(u<0.5f) {
            w1 = 0.5f*std::pow(u/0.5f, alpha);
            w0 = 1-w1;
        }
        else {
            w0 = 0.5f*std::pow(1-(u-0.5f)/0.5f, alpha);
            w1 = 1-w0;
        }

        skinning_influence influence_bone_0 = {0, w0};
        skinning_influence influence_bone_1 = {1, w1};
        skinning.influence.push_back( {influence_bone_0, influence_bone_1} );
    }

    skinning.rest_pose = shape.position;
    skinning.rest_pose_normal = shape.normal;
    skinning.deformed  = shape;

    shape_visual.clear();
    shape_visual = mesh_drawable(shape);
    shape_visual.shader = shader;

    timer = timer_interval();
    timer.t_max = 2.0f;
}



void load_character_data(skeleton_structure& skeleton, skinning_structure& skinning, vcl::buffer<float>& weight_flappy, mesh_drawable& shape_visual, timer_interval& timer, GLuint shader)
{
    skinning.influence = {};
    timer = timer_interval();

    const float scaling = 0.005f;

    // Load mesh
    buffer<buffer<int> > vertex_correspondance;
    mesh character = mesh_load_file_obj("scenes/sources/skinning/assets/marine/mesh.obj", vertex_correspondance);

    skeleton.connectivity   = read_skeleton_connectivity("scenes/sources/skinning/assets/marine/skeleton_connectivity");
    skeleton.rest_pose      = read_skeleton_geometry("scenes/sources/skinning/assets/marine/skeleton_geometry_local", scaling);
    buffer<buffer<skinning_influence>> influence = read_skinning_influence("scenes/sources/skinning/assets/marine/skinning_data");
    skinning.influence = map_correspondance(influence, vertex_correspondance);

    // 3 possibles animations:
    // skeleton.anim           = read_skeleton_animation("scenes/sources/skinning/assets/marine/skeleton_animation_idle", scaling); // t_max = 4.0f
    // skeleton.anim           = read_skeleton_animation("scenes/sources/skinning/assets/marine/skeleton_animation_walk", scaling); // t_max = 1.0f;
    skeleton.anim           = read_skeleton_animation("scenes/sources/skinning/assets/marine/skeleton_animation_run", scaling);  // t_max = 0.733f;
    timer.t_max = 0.733f;

    GLuint texture_id = create_texture_gpu(image_load_png("scenes/sources/skinning/assets/marine/texture.png"));

    for(vec3& p : character.position) p *= scaling; // scale vertices of the mesh
    shape_visual.clear();
    shape_visual = mesh_drawable(character);
    shape_visual.shader = shader;
    shape_visual.uniform.shading.specular = 0.1f;
    shape_visual.uniform.shading.specular_exponent = 8;

    character.fill_empty_fields();
    skinning.rest_pose = character.position;
    skinning.rest_pose_normal = character.normal;
    skinning.deformed  = character;


    weight_flappy.resize(character.position.size());
    weight_flappy.fill(1.0f);

    shape_visual.texture_id = texture_id;

}





buffer<buffer<skinning_influence> > read_skinning_influence(const std::string& filename)
{
    buffer<buffer<skinning_influence> > influence;

    std::ifstream fid(filename);

    // first line = number of influence per pertex (fixed for all vertices)
    size_t N_bone_influence=0;
    fid >> N_bone_influence;

    assert_vcl(fid.good(), "Cannot read file "+filename);
    assert_vcl_no_msg(N_bone_influence>0 && N_bone_influence<=6);

    // Read influence associated to each vertex
    std::vector<skinning_influence> skinning_vertex;
    skinning_vertex.resize(N_bone_influence);

    while(fid.good())
    {
        // read list of [bone index] [skinning weights]
        for(size_t k=0; k<N_bone_influence && fid.good(); ++k)
            fid >> skinning_vertex[k].joint >> skinning_vertex[k].weight;

        if(fid.good())
            influence.push_back(skinning_vertex);
    }

    fid.close();


    // normalize skinning weights (sum up to one)
    for(size_t kv=0; kv<influence.size(); ++kv)
    {
        float w = 0.0f;
        // compute weight sum
        for(size_t k=0; k<N_bone_influence; ++k)
            w += influence[kv][k].weight;
        // normalize
        if(w>1e-5f)
            for(size_t k=0; k<N_bone_influence; ++k)
                influence[kv][k].weight /= w;

    }

    return influence;
}

buffer<buffer<joint_geometry_time> > read_skeleton_animation(const std::string& filename, float scaling)
{
    buffer<buffer<joint_geometry_time> > skeleton;

    std::ifstream fid(filename);
    while(fid.good())
    {
        size_t N_key=0;
        fid >> N_key;

        if(fid.good())
        {
            buffer<joint_geometry_time> animated_joint;
            animated_joint.resize(N_key);

            for(size_t k_key=0; k_key<N_key; ++k_key)
            {
                float key_time;
                vec3 p;
                vec4 q;

                fid >> key_time;
                fid >> p.x >> p.y >> p.z;
                fid >> q.x >> q.y >> q.z >> q.w;

                q = normalize(q);

                animated_joint[k_key] = {key_time, {p*scaling,q} };
            }

            skeleton.push_back(animated_joint);
        }
    }

    fid.close();

    return skeleton;
}

buffer<joint_connectivity> read_skeleton_connectivity(const std::string& filename)
{
    buffer<joint_connectivity> skeleton;

    std::ifstream fid(filename);

    while(fid.good())
    {
        std::string line;
        std::getline(fid, line);

        if(fid.good())
        {
            std::stringstream sstream(line);

            int k;
            int parent;
            std::string name;

            sstream >> k >> parent >> name;

            skeleton.push_back({parent,name});
        }
    }

    fid.close();

    return skeleton;
}

buffer<joint_geometry> read_skeleton_geometry(const std::string& filename, float scaling)
{
    buffer<joint_geometry> skeleton;

    std::ifstream fid(filename);
    while(fid.good())
    {
        std::string line;
        std::getline(fid, line);

        if(fid.good())
        {
            std::stringstream sstream(line);

            vec3 p;
            quaternion q;

            sstream >> p.x >> p.y >> p.z;
            sstream >> q.x >> q.y >> q.z >> q.w;

            //q = normalize(q);

            skeleton.push_back({p*scaling,q});
        }
    }

    fid.close();

    return skeleton;
}


#endif
