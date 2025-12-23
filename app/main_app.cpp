#include <SDL3/SDL.h>
#include <GL/glew.h>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cmath>
#include <stdexcept>

// Dear ImGui
#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_opengl3.h"

// Project headers (Lab 3)
#include "Matrix3x3.hpp"
#include "Matrix4x4.hpp"
#include "Quat.hpp"

static void Check(bool ok, const char* msg) {
    if (!ok) {
        std::fprintf(stderr, "%s: %s\n", msg, SDL_GetError());
        std::fflush(stderr);
        std::exit(1);
    }
}

// ---------------- Helpers numèrics ----------------
static constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;

// ---------------- UI State ----------------
static bool show_pan_inputs = true;
static bool show_pan_ops = true;
static bool show_pan_out = true;

// Matriu principal 4x4 que l’alumne manipula
static Matrix4x4 a_user = Matrix4x4::Identity();

// TRS d’entrada
static Vec3   t_user = { 0.0, 0.0, 0.0 };
static double yaw_deg = 0.0;   // Z
static double pitch_deg = 0.0;   // Y
static double roll_deg = 0.0;   // X
static Vec3   s_user = { 1.0, 1.0, 1.0 };

// Punt i vector per TransformPoint / TransformVector
static Vec3 p_user = { 1.0, 2.0, 3.0 };
static Vec3 v_user = { 1.0, 0.0, 0.0 };

// Operacions disponibles
// 0) GetTranslation(a_user)
// 1) GetRotation(a_user)
// 2) GetRotationQuat(a_user)
// 3) GetScale(a_user)
// 4) GetRotationScale(a_user)
// 5) IsAffine(a_user)
// 6) TransformPoint(a_user, p_user)
// 7) TransformVector(a_user, v_user)
// 8) Identity()
// 9) Translate(t_user)
// 10) Scale(s_user)
// 11) Rotate(FromEulerZYX -> R)
// 12) Rotate(FromEulerZYX -> q)
// 13) FromTRS(t_user, R_euler, s_user)
// 14) FromTRS(t_user, q_euler, s_user)
static int op_selected = 0;

// Resultats getters
static bool      is_affine_out = false;
static Vec3      t_get = { 0.0, 0.0, 0.0 };
static Matrix3x3 R_get = Matrix3x3::Identity();
static Quat      q_get = { 1.0, 0.0, 0.0, 0.0 };
static Vec3      s_get = { 1.0, 1.0, 1.0 };
static Matrix3x3 RS_get = Matrix3x3::Identity();

// Resultats transformacions
static Vec3 p_out = { 0.0, 0.0, 0.0 };
static Vec3 v_out = { 0.0, 0.0, 0.0 };

// Resultats mètodes estàtics
static Matrix4x4 mat_static_out = Matrix4x4::Identity();
static char      mat_static_label[64] = "Cap metode static executat encara";

// Estat feedback
enum class Status { None, Ok, Error };
static Status last_status = Status::None;
static char   last_msg[256] = "";
// Quin ha estat l'ultim botó/operacio executada?
// -100 = cap
// -3 = SetTranslation
// -2 = SetRotation
// -1 = SetScale
// 0..14 = una de les operacions del combo
static int last_op_id = -100;

static void SetOk(const char* msg) {
    last_status = Status::Ok;
    std::snprintf(last_msg, sizeof(last_msg), "%s", msg);
}

static void SetErr(const char* msg) {
    last_status = Status::Error;
    std::snprintf(last_msg, sizeof(last_msg), "%s", msg);
}

// ---------------- Dibuixadors simples ----------------
static void DrawVec3Edit(const char* label, Vec3& v) {
    ImGui::SeparatorText(label);
    ImGui::PushID(label);
    ImGui::PushItemWidth(90);
    ImGui::InputDouble("x", &v.x, 0, 0, "%.6f"); ImGui::SameLine();
    ImGui::InputDouble("y", &v.y, 0, 0, "%.6f"); ImGui::SameLine();
    ImGui::InputDouble("z", &v.z, 0, 0, "%.6f");
    ImGui::PopItemWidth();
    ImGui::PopID();
}

static void DrawMat3Edit(const char* label, Matrix3x3& M, bool editable = true) {
    ImGui::SeparatorText(label);
    if (ImGui::BeginTable(label, 3, ImGuiTableFlags_Borders)) {
        for (int i = 0; i < 3; ++i) {
            ImGui::TableNextRow();
            for (int j = 0; j < 3; ++j) {
                ImGui::TableSetColumnIndex(j);
                double tmp = M.At(i, j);
                if (editable) {
                    ImGui::SetNextItemWidth(70);
                    std::string cell_id = std::string(label) + "[" +
                        std::to_string(i) + "," + std::to_string(j) + "]";
                    if (ImGui::InputDouble(cell_id.c_str(), &tmp, 0, 0, "%.6f")) {
                        M.At(i, j) = tmp;
                    }
                }
                else {
                    ImGui::Text("%.6f", tmp);
                }
            }
        }
        ImGui::EndTable();
    }
}

static void DrawQuatView(const char* label, const Quat& q) {
    ImGui::SeparatorText(label);
    ImGui::Text("s = %.6f, x = %.6f, y = %.6f, z = %.6f",
        q.s, q.x, q.y, q.z);
}

static void DrawMat4Edit(const char* label, Matrix4x4& M, bool editable = true) {
    ImGui::SeparatorText(label);
    if (ImGui::BeginTable(label, 4, ImGuiTableFlags_Borders)) {
        for (int i = 0; i < 4; ++i) {
            ImGui::TableNextRow();
            for (int j = 0; j < 4; ++j) {
                ImGui::TableSetColumnIndex(j);
                double tmp = M.At(i, j);
                if (editable) {
                    ImGui::SetNextItemWidth(70);
                    std::string cell_id = "[" +
                        std::to_string(i) + "," + std::to_string(j) + "]";
                    if (ImGui::InputDouble(cell_id.c_str(), &tmp, 0, 0, "%.6f")) {
                        M.At(i, j) = tmp;
                    }
                }
                else {
                    ImGui::Text("%.6f", tmp);
                }
            }
        }
        ImGui::EndTable();
    }
}

int main()
{
    // Logs SDL
    SDL_SetLogPriority(SDL_LOG_CATEGORY_APPLICATION, SDL_LOG_PRIORITY_VERBOSE);

    // SDL init
    Check(SDL_Init(SDL_INIT_VIDEO), "SDL_Init failed");

    // GL attributes & window
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    SDL_Window* window = SDL_CreateWindow(
        "Laboratori 3: Transformacions Afins",
        1280, 720,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE
    );
    Check(window != nullptr, "SDL_CreateWindow failed");

    SDL_GLContext gl_ctx = SDL_GL_CreateContext(window);
    Check(gl_ctx != nullptr, "SDL_GL_CreateContext failed");
    SDL_GL_MakeCurrent(window, gl_ctx);
    SDL_GL_SetSwapInterval(1); // vsync

    // GLEW
    glewExperimental = GL_TRUE;
    GLenum glew_err = glewInit();
    if (glew_err != GLEW_OK) {
        std::fprintf(stderr, "glewInit failed\n");
        return 1;
    }

    // Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplSDL3_InitForOpenGL(window, gl_ctx);
    const char* glsl_version = "#version 330";
    ImGui_ImplOpenGL3_Init(glsl_version);

    bool running = true;
    while (running)
    {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            ImGui_ImplSDL3_ProcessEvent(&e);
            if (e.type == SDL_EVENT_QUIT) running = false;
            if (e.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED &&
                e.window.windowID == SDL_GetWindowID(window))
                running = false;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        // ------------- PANELL: ENTRADES -------------
        if (show_pan_inputs && ImGui::Begin("Entrades", &show_pan_inputs)) {
            ImGui::TextWrapped(
                "Lab 3: Afinitats\n"
                "1) Introdueix translacio, rotacio (Euler ZYX en graus) i escala,\n"
                "   i aplica-les a la matriu A amb els botons.\n"
                "2) Fes servir el llistat d'operacions per provar getters, metodes\n"
                "   estatics i TransformPoint/TransformVector."
            );
            ImGui::Separator();

            // Matriu principal a_user (editable)
            DrawMat4Edit("Matriu A (4x4)", a_user, true);

            ImGui::Separator();
            // TRS d'entrada
            DrawVec3Edit("Translacio T", t_user);

            ImGui::SeparatorText("Rotacio Euler (ZYX) en graus");
            ImGui::InputDouble("Yaw (Z)", &yaw_deg, 0, 0, "%.6f");
            ImGui::InputDouble("Pitch (Y)", &pitch_deg, 0, 0, "%.6f");
            ImGui::InputDouble("Roll (X)", &roll_deg, 0, 0, "%.6f");
            if (ImGui::Button("Gimbal lock +90 deg (pitch)")) pitch_deg = 90.0;
            ImGui::SameLine();
            if (ImGui::Button("Gimbal lock -90 deg (pitch)")) pitch_deg = -90.0;

            DrawVec3Edit("Escala S", s_user);

            ImGui::SeparatorText("Aplica components a A");
            if (ImGui::Button("Aplica TRANSLACIO a A")) {
                try {
                    a_user.SetTranslation(t_user);
                    last_op_id = -3;
                    SetOk("SetTranslation aplicat a A.");
                }
                catch (const std::exception& ex) {
                    SetErr(ex.what());
                }
            }

            if (ImGui::Button("Aplica ROTACIO (Euler) a A")) {
                try {
                    double yaw = yaw_deg * DEG2RAD;
                    double pitch = pitch_deg * DEG2RAD;
                    double roll = roll_deg * DEG2RAD;
                    Matrix3x3 R_euler = Matrix3x3::FromEulerZYX(yaw, pitch, roll);
                    a_user.SetRotation(R_euler);
                    last_op_id = -2;
                    SetOk("SetRotation (a partir d'Euler) aplicat a A.");
                }
                catch (const std::exception& ex) {
                    SetErr(ex.what());
                }
            }

            if (ImGui::Button("Aplica ESCALA a A")) {
                try {
                    a_user.SetScale(s_user);
                    last_op_id = -1;
                    SetOk("SetScale aplicat a A.");
                }
                catch (const std::exception& ex) {
                    SetErr(ex.what());
                }
            }

            ImGui::Separator();
            DrawVec3Edit("Punt P (per Transform Point)", p_user);
            DrawVec3Edit("Vector V (per Transform Vector)", v_user);
        }
        if (show_pan_inputs) ImGui::End();

        // ------------- PANELL: OPERACIONS -------------
        if (show_pan_ops && ImGui::Begin("Operacions", &show_pan_ops)) {
            static const char* ops[] = {
                "0) Get Translation (A)",
                "1) Get Rotation (A)",
                "2) Get Rotation Quat (A)",
                "3) Get Scale (A)",
                "4) Get Rotation-Scale (A)",
                "5) Is Affine? (A)",
                "6) Transform Point (A, P)",
                "7) Transform Vector (A, V)",
                "8) Get Matrix Translate (T)",
                "9) Get Matrix Scale (S)",
                "10) Get Matrix Rotate(ZYX -> R)",
                "11) Get Matrix Rotate(ZYX -> q)",
                "12) Get Matrix TRS(T, ZYX -> R, S)",
                "13) Get Matrix TRS(T, ZYX -> q, S)",
				"14) Get Inverse TR (A)",
				"15) Get Inverse TRS (A)",
            };

            ImGui::Combo("Operacio", &op_selected, ops, IM_ARRAYSIZE(ops));

            if (ImGui::Button("Run")) {
                last_status = Status::None;
                last_msg[0] = '\0';
                last_op_id = op_selected;

                try {
                    // Conversions comuns (Euler -> R, q)
                    double yaw = yaw_deg * DEG2RAD;
                    double pitch = pitch_deg * DEG2RAD;
                    double roll = roll_deg * DEG2RAD;
                    Matrix3x3 R_euler = Matrix3x3::FromEulerZYX(yaw, pitch, roll);
                    Quat      q_euler = Quat::FromEulerZYX(yaw, pitch, roll);

                    switch (op_selected) {
                    case 0: // GetTranslation
                        t_get = a_user.GetTranslation();
                        SetOk("Get Translation (A) executat.");
                        break;

                    case 1: // GetRotation
                        R_get = a_user.GetRotation();
                        SetOk("Get Rotation (A) executat.");
                        break;

                    case 2: // GetRotationQuat
                        q_get = a_user.GetRotationQuat();
                        SetOk("Get Rotation Quat (A) executat.");
                        break;

                    case 3: // GetScale
                        s_get = a_user.GetScale();
                        SetOk("Get Scale (A) executat.");
                        break;

                    case 4: // GetRotationScale
                        RS_get = a_user.GetRotationScale();
                        SetOk("Get Rotation-Scale (A) executat.");
                        break;

                    case 5: // IsAffine
                        is_affine_out = a_user.IsAffine();
                        SetOk("Is Affine? (A) executat.");
                        break;

                    case 6: // TransformPoint
                        p_out = a_user.TransformPoint(p_user);
                        SetOk("Transform Point (A, P) executat.");
                        break;

                    case 7: // TransformVector
                        v_out = a_user.TransformVector(v_user);
                        SetOk("Transform Vector (A, V) executat.");
                        break;

                    case 8: // Translate
                        mat_static_out = Matrix4x4::Translate(t_user);
                        std::snprintf(mat_static_label, sizeof(mat_static_label),
                            "Translate(T)");
                        SetOk("Get Matrix Translate (T) executat.");
                        break;

                    case 9: // Scale
                        mat_static_out = Matrix4x4::Scale(s_user);
                        std::snprintf(mat_static_label, sizeof(mat_static_label),
                            "Scale(S)");
                        SetOk("Get Matrix Scale (S) executat.");
                        break;

                    case 10: // Rotate (R)
                        mat_static_out = Matrix4x4::Rotate(R_euler);
                        std::snprintf(mat_static_label, sizeof(mat_static_label),
                            "Rotate(R)");
                        SetOk("Get Matrix Rotate(ZYX -> R) executat.");
                        break;

                    case 11: // Rotate (q)
                        mat_static_out = Matrix4x4::Rotate(q_euler);
                        std::snprintf(mat_static_label, sizeof(mat_static_label),
                            "Rotate(q)");
                        SetOk("Get Matrix Rotate(ZYX -> q) executat.");
                        break;

                    case 12: // FromTRS(t, R, s)
                        mat_static_out = Matrix4x4::FromTRS(t_user, R_euler, s_user);
                        std::snprintf(mat_static_label, sizeof(mat_static_label),
                            "FromTRS(T, R, S)");
                        SetOk("Get Matrix TRS(T, ZYX -> R, S) executat.");
                        break;

                    case 13: // FromTRS(t, q, s)
                        mat_static_out = Matrix4x4::FromTRS(t_user, q_euler, s_user);
                        std::snprintf(mat_static_label, sizeof(mat_static_label),
                            "FromTRS(T, q, S)");
                        SetOk("Get Matrix TRS(T, ZYX -> q, S) executat.");
                        break;
					case 14: // InverseTR
						mat_static_out = a_user.InverseTR();
						std::snprintf(mat_static_label, sizeof(mat_static_label),
							"InverseTR(A)");
						SetOk("Get Inverse TR (A) executat.");
						break;
					case 15: // InverseTRS
						mat_static_out = a_user.InverseTRS();
						std::snprintf(mat_static_label, sizeof(mat_static_label),
							"InverseTRS(A)");
						SetOk("Get Inverse TRS (A) executat.");
						break;

                    default:
                        SetErr("Operacio desconeguda.");
                        break;
                    }
                }
                catch (const std::exception& ex) {
                    SetErr(ex.what());
                }
            }
        }
        if (show_pan_ops) ImGui::End();

        // ------------- PANELL: RESULTATS -------------
        if (show_pan_out && ImGui::Begin("Resultats", &show_pan_out)) {
            ImGui::SeparatorText("Estat execucio");
            if (last_status == Status::Ok) {
                ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.2f, 1.0f),
                    "OK: %s", last_msg);
            }
            else if (last_status == Status::Error) {
                ImGui::TextColored(ImVec4(0.95f, 0.25f, 0.2f, 1.0f),
                    "Error: %s", last_msg);
            }
            else {
                ImGui::TextDisabled("Encara no s'ha executat cap operacio o boto.");
            }

            ImGui::SeparatorText("Resultat de l'ultima operacio");

            if (last_op_id == -100) {
                ImGui::TextDisabled("Cap operacio executada encara.");
            }
            else if (last_op_id == -3) {        // SetTranslation
                ImGui::Text("SetTranslation aplicat. Matriu A actual:");
                Matrix4x4 a_view = a_user;
                DrawMat4Edit("A despres de SetTranslation", a_view, false);
            }
            else if (last_op_id == -2) {        // SetRotation
                ImGui::Text("SetRotation (Euler) aplicat. Matriu A actual:");
                Matrix4x4 a_view = a_user;
                DrawMat4Edit("A despres de SetRotation", a_view, false);
            }
            else if (last_op_id == -1) {        // SetScale
                ImGui::Text("SetScale aplicat. Matriu A actual:");
                Matrix4x4 a_view = a_user;
                DrawMat4Edit("A despres de Set Scale", a_view, false);
            }
            else {
                // 0..14 venen del combo d'operacions
                switch (last_op_id) {
                case 0: // GetTranslation
                    ImGui::Text("GetTranslation(A) = (%.6f, %.6f, %.6f)",
                        t_get.x, t_get.y, t_get.z);
                    break;

                case 1: // GetRotation
                    ImGui::Text("GetRotation(A) (3x3):");
                    DrawMat3Edit("R_get", R_get, false);
                    break;

                case 2: // GetRotationQuat
                    DrawQuatView("GetRotationQuat(A)", q_get);
                    break;

                case 3: // GetScale
                    ImGui::Text("GetScale(A) = (%.6f, %.6f, %.6f)",
                        s_get.x, s_get.y, s_get.z);
                    break;

                case 4: // GetRotationScale
                    ImGui::Text("GetRotationScale(A) (3x3):");
                    DrawMat3Edit("RS_get", RS_get, false);
                    break;

                case 5: // IsAffine
                    ImGui::Text("IsAffine(A) = %s",
                        is_affine_out ? "true" : "false");
                    break;

                case 6: // TransformPoint
                    ImGui::Text("P = (%.6f, %.6f, %.6f)",
                        p_user.x, p_user.y, p_user.z);
                    ImGui::Text("P' = (%.6f, %.6f, %.6f)",
                        p_out.x, p_out.y, p_out.z);
                    break;

                case 7: // TransformVector
                    ImGui::Text("V = (%.6f, %.6f, %.6f)",
                        v_user.x, v_user.y, v_user.z);
                    ImGui::Text("V' = (%.6f, %.6f, %.6f)",
                        v_out.x, v_out.y, v_out.z);
                    break;

                case 8:  
                case 9:  
                case 10: 
                case 11: 
                case 12: 
                case 13:
                case 14:
                case 15:
                    ImGui::Text("Operacio: %s", mat_static_label);
                    DrawMat4Edit("Matriu resultat", mat_static_out, false);
                    break;
                default:
                    ImGui::TextDisabled("Operacio desconeguda o no suportada.");
                    break;
                }
            }
        }
        if (show_pan_out) ImGui::End();


        // ------------- Render -------------
        ImGui::Render();
        glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    // Shutdown ImGui + SDL
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DestroyContext(gl_ctx);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
