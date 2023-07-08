#include "src/math/q3Vec3.h"
#include <cmath>
#include <math.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include "gl.h"
#include "../imgui/imgui.h"
#include "../imgui/backends/imgui_impl_glfw.h"
#include "../imgui/backends/imgui_impl_opengl3.h"

#include "../src/q3.h"
#include "../src/zig_style/debug.cpp"

float dt = 1 / 60.f;

q3Scene scene(dt);

static const q3Vec3 camera_position = {0.0f, 5.0f, 20.0f};
static const q3Vec3 camera_target = {0.0f, 0.0f, 0.0f};

static const float light_ambient[4] = {1.0f, 1.0f, 1.0f, 0.5f};
static const float light_diffuse[4] = {0.2f, 0.4f, 0.7f, 1.0f};
static const float light_specular[4] = {1.0f, 1.0f, 1.0f, 1.0f};

static const char* vertex_shader_src = "#version 330 core\n"
                                       "uniform mat4 proj_mat;\n"
                                       "uniform mat4 view_mat;\n"
                                       "layout (location = 0) in vec3 pos;\n"
                                       "void main() {;\n"
                                       "    gl_Position = proj_mat * view_mat * vec4(pos, 1);\n"
                                       "};\n";
static const char* fragment_shader_src = "#version 330 core\n"
                                         "uniform vec3 color;\n"
                                         "uniform vec3 light_dir;\n"
                                         "uniform vec3 normal;\n"
                                         "out vec4 FragColor;\n"
                                         "void main() {;\n"
                                         "    vec3 sun_dir = normalize(light_dir);\n"
                                         "    float ambient = 0.5;\n"
                                         "    float light = max(ambient, dot(-normal, sun_dir));\n"
                                         "    gl_FragColor = vec4(color * light, 1);\n"
                                         "};\n";

struct Mesh {
    uint32_t vao, vbo, ebo;
    uint16_t n_indices;

    static Mesh init(
        const float vert_data[], size_t vert_data_len, const uint32_t indices[], size_t indices_len,
        size_t n_elems
    ) {
        Mesh mesh = {.n_indices = (uint16_t)indices_len};

        glGenVertexArrays(1, &mesh.vao);
        glBindVertexArray(mesh.vao);

        glGenBuffers(1, &mesh.vbo);
        glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);

        glBufferData(GL_ARRAY_BUFFER, vert_data_len * sizeof(float), vert_data, GL_STATIC_DRAW);
        glVertexAttribPointer(0, n_elems, GL_FLOAT, GL_FALSE, 0, NULL);
        glEnableVertexAttribArray(0);

        glGenBuffers(1, &mesh.ebo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ebo);
        glBufferData(
            GL_ELEMENT_ARRAY_BUFFER, indices_len * sizeof(uint32_t), indices, GL_STATIC_DRAW
        );

        return mesh;
    }

    void deinit() {
        glDeleteVertexArrays(1, &this->vao);
        glDeleteBuffers(1, &this->vbo);
        glDeleteBuffers(1, &this->ebo);
    }

    void draw(uint32_t primitive_type) {
        glBindVertexArray(this->vao);
        glDrawElements(primitive_type, this->n_indices, GL_UNSIGNED_INT, NULL);
    }
};

struct Renderer : public q3Render {
    float triangle_normal[3];
    uint32_t shader_id;
    float pen_color[3];
    float pen_pos[3];

    int32_t CompileShaderSource(int32_t shader_type, const char* src) {
        auto id = glCreateShader(shader_type);
        glShaderSource(id, 1, &src, NULL);
        glCompileShader(id);
        int32_t success = 0;
        glGetShaderiv(id, GL_COMPILE_STATUS, &success);
        if (success == GL_FALSE) {
            char msg[1000] = {};
            glGetShaderInfoLog(id, 1000, NULL, msg);
            debug::panic("gl shader iv: %s\n", msg);
        }
        return id;
    }

    void Build() {
        // compile/link shader source
        auto v_id = CompileShaderSource(GL_VERTEX_SHADER, vertex_shader_src);
        auto f_id = CompileShaderSource(GL_FRAGMENT_SHADER, fragment_shader_src);
        auto prog_id = glCreateProgram();
        glAttachShader(prog_id, v_id);
        glAttachShader(prog_id, f_id);
        glLinkProgram(prog_id);

        this->shader_id = prog_id;
    }

    void SetShaderMatrices() {
        glUseProgram(this->shader_id);

        const auto f = q3Normalize(camera_target - camera_position);
        const auto s = q3Normalize(q3Cross(f, q3Vec3(0, 1, 0)));
        const auto u = q3Cross(s, f);
        const auto pos = camera_position;
        const float view_mat[16] = {
            s[0], u[0], -f[0], 0, s[1],           u[1],           -f[1],         0,
            s[2], u[2], -f[2], 0, -q3Dot(s, pos), -q3Dot(u, pos), q3Dot(f, pos), 1};
        auto loc = glGetUniformLocation(this->shader_id, "view_mat");
        glUniformMatrix4fv(loc, 1, GL_FALSE, view_mat);

        const float ratio = 10.0 / 6.0;
        const float near = 0.1;
        const float far = 1000;
        const float fov = 40;
        const float half_tan_fov = tanf(fov * M_PI / 360.0);
        float proj_mat[16] = {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        };
        proj_mat[0 * 4 + 0] = 1 / (ratio * half_tan_fov);
        proj_mat[1 * 4 + 1] = 1 / half_tan_fov;
        proj_mat[2 * 4 + 2] = (near + far) / (near - far);
        proj_mat[2 * 4 + 3] = -1;
        proj_mat[3 * 4 + 2] = (2 * near * far) / (near - far);
        loc = glGetUniformLocation(this->shader_id, "proj_mat");
        glUniformMatrix4fv(loc, 1, GL_FALSE, proj_mat);

        auto light_dir = camera_target - camera_position;
        loc = glGetUniformLocation(this->shader_id, "light_dir");
        glUniform3fv(loc, 1, light_dir.v);
    }

    void SetPenColor(f32 r, f32 g, f32 b, f32 a = 1.0f) override {
        this->pen_color[0] = r;
        this->pen_color[1] = g;
        this->pen_color[2] = b;
    }

    void SetPenPosition(f32 x, f32 y, f32 z) override {
        pen_pos[0] = x;
        pen_pos[1] = y;
        pen_pos[2] = z;
    }

    void SetScale(f32 sx, f32 sy, f32 sz) override { glPointSize(sx); }

    void Line(f32 x, f32 y, f32 z) override {
        const float verts[6] = {pen_pos[0], pen_pos[1], pen_pos[2], x, y, z};
        const uint32_t indices[2] = {0, 1};
        auto mesh = Mesh::init(verts, 6, indices, 2, 3);
        SetShaderMatrices();
        auto loc = glGetUniformLocation(this->shader_id, "color");
        glUniform3fv(loc, 1, pen_color);
        mesh.draw(GL_LINES);
        mesh.deinit();
    }

    void Triangle(f32 x1, f32 y1, f32 z1, f32 x2, f32 y2, f32 z2, f32 x3, f32 y3, f32 z3) override {
        const float verts[9] = {x1, y1, z1, x2, y2, z2, x3, y3, z3};
        const uint32_t indices[3] = {0, 1, 2};
        auto mesh = Mesh::init(verts, 9, indices, 3, 3);

        SetShaderMatrices();

        const float color[3] = {.2, .4, .7};
        auto loc = glGetUniformLocation(this->shader_id, "color");
        glUniform3fv(loc, 1, color);

        loc = glGetUniformLocation(this->shader_id, "normal");
        glUniform3fv(loc, 1, triangle_normal);

        mesh.draw(GL_TRIANGLES);

        // draw wireframe
        const float black[3] = {0, 0, 0};
        loc = glGetUniformLocation(this->shader_id, "color");
        glUniform3fv(loc, 1, black);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        mesh.draw(GL_TRIANGLES);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        mesh.deinit();
    }

    void SetTriNormal(f32 x, f32 y, f32 z) override {
        triangle_normal[0] = x;
        triangle_normal[1] = y;
        triangle_normal[2] = z;
    }

    void Point() override {
        const uint32_t indices[1] = {0};
        auto mesh = Mesh::init(pen_pos, 3, indices, 1, 3);
        SetShaderMatrices();
        auto loc = glGetUniformLocation(this->shader_id, "color");
        glUniform3fv(loc, 1, pen_color);
        mesh.draw(GL_POINTS);
        mesh.deinit();
    };
};

Renderer renderer;

void gl_error_callback(
    uint32_t source, uint32_t error_type, uint32_t id, uint32_t severity, int32_t len,
    const char* msg, void* user_param
) {
    if (severity == GL_DEBUG_SEVERITY_NOTIFICATION) return;

    debug::print("OpenGL: (");
    switch (source) {
        case 0x824B: debug::print("SOURCE_OTHER"); break;
        case 0x824A: debug::print("SOURCE_APPLICATION"); break;
        case 0x8249: debug::print("SOURCE_THIRD_PARTY"); break;
        case 0x8248: debug::print("SOURCE_SHADER_COMPILER"); break;
        case 0x8247: debug::print("SOURCE_WINDOW_SYSTEM"); break;
        case 0x8246: debug::print("SOURCE_API"); break;
    }
    debug::print(", ");
    switch (severity) {
        case 0x826B: debug::print("SEVERITY_NOTIFICATION"); break;
        case 0x9148: debug::print("SEVERITY_LOW"); break;
        case 0x9147: debug::print("SEVERITY_MEDIUM"); break;
        case 0x9146: debug::print("SEVERITY_HIGH"); break;
    }
    debug::print(", ");
    switch (error_type) {
        case 0x826A: debug::print("TYPE_POP_GROUP"); break;
        case 0x8269: debug::print("TYPE_PUSH_GROUP"); break;
        case 0x8268: debug::print("TYPE_MARKER"); break;
        case 0x8251: debug::print("TYPE_OTHER"); break;
        case 0x8250: debug::print("TYPE_PERFORMANCE"); break;
        case 0x824F: debug::print("TYPE_PORTABILITY"); break;
        case 0x824E: debug::print("TYPE_UNDEFINED_BEHAVIOR"); break;
        case 0x824D: debug::print("TYPE_DEPRECATED_BEHAVIOR"); break;
        case 0x824C: debug::print("TYPE_ERROR"); break;
    }
    debug::print(", id=%u) %s\n", id, msg);
}

struct Demo {
    virtual ~Demo() {}

    virtual void Init(){};
    virtual void Update(){};
    virtual void Shutdown(){};

    virtual void Render(q3Render* debugDrawer) { (void)debugDrawer; }
};

struct DropBoxes : public Demo {
    float acc;

    virtual void Init() {
        acc = 0;

        // Create the floor
        q3Body* body = scene.CreateBody({});

        q3BoxDef boxDef;
        boxDef.m_restitution = 0;
        q3Transform tx;
        q3Identity(tx);
        boxDef.Set(tx, q3Vec3(50.0f, 1.0f, 50.0f));
        body->SetBox(boxDef);
    }

    virtual void Update() {
        acc += dt;

        if (acc > 1.0f) {
            acc = 0;

            q3Body* body = scene.CreateBody({
                .axis = q3Vec3(q3RandomFloat(-1, 1), q3RandomFloat(-1, 1), q3RandomFloat(-1, 1)),
                .angle = q3PI * q3RandomFloat(-1, 1),
                .position = q3Vec3(0, 3, 0),
                .linearVelocity =
                    q3Vec3(q3RandomFloat(1, 3), q3RandomFloat(1, 3), q3RandomFloat(1, 3)) *
                    q3Sign(q3RandomFloat(-1, 1)),
                .angularVelocity =
                    q3Vec3(q3RandomFloat(1, 3), q3RandomFloat(1, 3), q3RandomFloat(1, 3)) *
                    q3Sign(q3RandomFloat(-1, 1)),
                .bodyType = eDynamicBody,
            });

            q3Transform tx;
            q3Identity(tx);
            q3BoxDef boxDef;
            boxDef.Set(tx, q3Vec3(1.0f, 1.0f, 1.0f));
            body->SetBox(boxDef);
        }
    }

    virtual void Shutdown() { scene.RemoveAllBodies(); }
};

class Raycast : public q3QueryCallback {
public:
    q3RaycastData data;
    r32 tfinal;
    q3Vec3 nfinal;
    q3Body* impactBody;

    bool ReportShape(q3Box* shape) {
        if (data.toi < tfinal) {
            tfinal = data.toi;
            nfinal = data.normal;
            impactBody = shape->body;
        }

        data.toi = tfinal;
        return true;
    }

    void Init(const q3Vec3& spot, const q3Vec3& dir) {
        data.start = spot;
        data.dir = q3Normalize(dir);
        data.t = r32(10000.0);
        tfinal = FLT_MAX;
        data.toi = data.t;
        impactBody = NULL;
    }
};

struct RayPush : Demo {
    float acc;
    Raycast rayCast;

    void Init() {
        acc = 0;

        // Create the floor
        q3Body* body = scene.CreateBody({});

        q3BoxDef boxDef;
        boxDef.m_restitution = 0;
        q3Transform tx;
        q3Identity(tx);
        boxDef.Set(tx, q3Vec3(50.0f, 1.0f, 50.0f));
        body->SetBox(boxDef);
    }

    void Update() {
        acc += dt;

        if (acc > 1.0f) {
            acc = 0;

            q3Body* body = scene.CreateBody({
                .axis = q3Vec3(q3RandomFloat(-1, 1), q3RandomFloat(-1, 1), q3RandomFloat(-1, 1)),
                .angle = q3PI * q3RandomFloat(-1, 1),
                .position = q3Vec3(0, 3, 0),
                .linearVelocity =
                    q3Vec3(q3RandomFloat(1, 3), q3RandomFloat(1, 3), q3RandomFloat(1, 3)) *
                    q3Sign(q3RandomFloat(-1, 1)),
                .angularVelocity =
                    q3Vec3(q3RandomFloat(1, 3), q3RandomFloat(1, 3), q3RandomFloat(1, 3)) *
                    q3Sign(q3RandomFloat(-1, 1)),
                .bodyType = eDynamicBody,
            });

            q3Transform tx;
            q3Identity(tx);
            q3BoxDef boxDef;
            boxDef.Set(tx, q3Vec3(1.0f, 1.0f, 1.0f));
            body->SetBox(boxDef);
        }

        rayCast.Init(q3Vec3(3.0f, 5.0f, 3.0f), q3Vec3(-1.0f, -1.0f, -1.0f));
        scene.RayCast(&rayCast, rayCast.data);

        if (rayCast.impactBody) {
            rayCast.impactBody->ApplyForceAtWorldPoint(
                rayCast.data.dir * 20.0f, rayCast.data.GetImpactPoint()
            );
        }
    }

    void Shutdown() { scene.RemoveAllBodies(); }

    void Render(q3Render* render) {
        render->SetScale(1.0f, 1.0f, 1.0f);
        render->SetPenColor(0.2f, 0.5f, 1.0f);
        render->SetPenPosition(rayCast.data.start.x, rayCast.data.start.y, rayCast.data.start.z);
        q3Vec3 impact = rayCast.data.GetImpactPoint();
        render->Line(impact.x, impact.y, impact.z);

        render->SetPenPosition(impact.x, impact.y, impact.z);
        render->SetPenColor(1.0f, 0.5f, 0.5f);
        render->SetScale(10.0f, 10.0f, 10.0f);
        render->Point();

        render->SetPenColor(1.0f, 0.5f, 0.2f);
        render->SetScale(1.0f, 1.0f, 1.0f);
        impact += rayCast.nfinal * 2.0f;
        render->Line(impact.x, impact.y, impact.z);
    }
};

struct BoxStack : Demo {
    virtual void Init() {
        // Create the floor
        q3Body* body = scene.CreateBody({});

        q3BoxDef boxDef;
        boxDef.m_restitution = 0;
        q3Transform tx;
        q3Identity(tx);
        boxDef.Set(tx, q3Vec3(50.0f, 1.0f, 50.0f));
        body->SetBox(boxDef);

        boxDef.Set(tx, q3Vec3(1.0f, 1.0f, 1.0f));

        for (int32_t i = 0; i < 8; ++i) {
            for (int32_t j = 0; j < 8; ++j) {
                for (int32_t k = 0; k < 10; ++k) {
                    body = scene.CreateBody({
                        .position = q3Vec3(-16.0f + 1.0f * j, 1.0f * i + 5.0f, -16.0f + 1.0f * k),
                        .bodyType = eDynamicBody,
                    });
                    body->SetBox(boxDef);
                }
            }
        }
    }

    virtual void Shutdown() { scene.RemoveAllBodies(); }
};

int main() {
    // setup GLFW window
    if (!glfwInit()) return 1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    // glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    GLFWwindow* window = glfwCreateWindow(1000, 600, "", NULL, NULL);
    if (!window) return 1;
    glfwMakeContextCurrent(window);

    gladLoadGL((GLADloadfunc)glfwGetProcAddress);
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback((GLDEBUGPROC)gl_error_callback, NULL);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls
    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init();

    // opengl state stuff
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);

    renderer.Build();

    const char* const demo_choices[] = {"Drop Boxes", "Ray Push", "Box Stack"};
    Demo* demos[3] = {new DropBoxes(), new RayPush(), new BoxStack()};
    constexpr int demo_count = 3;
    int32_t current_demo = 1;
    bool paused = false;
    bool do_single_step = false;
    bool enable_friction = true;
    int32_t iterations = 10;

    demos[current_demo]->Init();

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);

        ImGui::Begin("q3Scene Settings");
        {
            const auto saved_demo = current_demo;
            ImGui::Combo("Demo", &current_demo, demo_choices, demo_count);
            if (saved_demo != current_demo) {
                // demos[saved_demo]->Shutdown();
                // demos[current_demo]->Init();
            }
            ImGui::Checkbox("Pause", &paused);
            if (paused && ImGui::Button("Single Step")) do_single_step = true;
            ImGui::Checkbox("Friction", &enable_friction);
            ImGui::SliderInt("Iterations", &iterations, 1, 50);
        }
        ImGui::End();

        auto active_demo = demos[current_demo];
        scene.enable_friction = enable_friction;
        scene.iterations = iterations;

        if (!paused || do_single_step) {
            scene.Step();
            active_demo->Update();
            if (do_single_step) do_single_step = false;
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        scene.Render(&renderer);
        active_demo->Render(&renderer);

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // imgui shutdown
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    // glfw shutdown
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}