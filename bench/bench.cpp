#include <iostream>
#include <iomanip>
#include <random>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>

// ---------------------------------------------------------
// CORRECCIÓN: Solo incluimos la matriz principal y Quat.
// Asumimos que Matrix4x4.hpp ya hace include de Vec3 y Matrix3x3.
// ---------------------------------------------------------
#include "Matrix4x4.hpp"
#include "Quat.hpp"

// -------------------- Colors ANSI ----------------------
static constexpr const char* GREEN = "\x1b[32m";
static constexpr const char* RED = "\x1b[31m";
static constexpr const char* YELL = "\x1b[33m";
static constexpr const char* CYAN = "\x1b[36m";
static constexpr const char* BOLD = "\x1b[1m";
static constexpr const char* RESET = "\x1b[0m";

static constexpr double PI = 3.14159265358979323846;
static constexpr double TOL = 1e-4;
// Asumimos que Matrix4x4 tiene Identity()
static const Matrix4x4 M4_IDENTITY = Matrix4x4::Identity();

// -------------------- Helpers Auxiliares -----------------

static bool Nearly(double a, double b, double eps = TOL) { return std::fabs(a - b) <= eps; }

static bool VecEq(const Vec3& a, const Vec3& b, double eps = TOL) {
    return Nearly(a.x, b.x, eps) && Nearly(a.y, b.y, eps) && Nearly(a.z, b.z, eps);
}

static bool Mat3Eq(const Matrix3x3& A, const Matrix3x3& B, double eps = TOL) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (!Nearly(A.At(i, j), B.At(i, j), eps)) return false;
    return true;
}

static bool Mat4Eq(const Matrix4x4& A, const Matrix4x4& B, double eps = TOL) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            if (!Nearly(A.At(i, j), B.At(i, j), eps)) return false;
    return true;
}

// Helpers externos para no modificar tus archivos .hpp

static Vec3 GetTraFromMat(const Matrix4x4& M) {
    return Vec3(M.At(0, 3), M.At(1, 3), M.At(2, 3));
}

static Vec3 GetSclFromMat(const Matrix4x4& M) {
    // Calculamos la magnitud de las columnas 0, 1 y 2
    double sx = std::sqrt(M.At(0, 0) * M.At(0, 0) + M.At(1, 0) * M.At(1, 0) + M.At(2, 0) * M.At(2, 0));
    double sy = std::sqrt(M.At(0, 1) * M.At(0, 1) + M.At(1, 1) * M.At(1, 1) + M.At(2, 1) * M.At(2, 1));
    double sz = std::sqrt(M.At(0, 2) * M.At(0, 2) + M.At(1, 2) * M.At(1, 2) + M.At(2, 2) * M.At(2, 2));
    return Vec3(sx, sy, sz);
}

static Matrix3x3 GetRotFromMat(const Matrix4x4& M) {
    Vec3 s = GetSclFromMat(M);
    Matrix3x3 R;
    // Normalizamos las columnas para obtener la rotación
    // Evitamos división por cero si la escala es 0
    double idx = (s.x > 1e-9) ? 1.0 / s.x : 0.0;
    double idy = (s.y > 1e-9) ? 1.0 / s.y : 0.0;
    double idz = (s.z > 1e-9) ? 1.0 / s.z : 0.0;

    R.At(0, 0) = M.At(0, 0) * idx; R.At(0, 1) = M.At(0, 1) * idy; R.At(0, 2) = M.At(0, 2) * idz;
    R.At(1, 0) = M.At(1, 0) * idx; R.At(1, 1) = M.At(1, 1) * idy; R.At(1, 2) = M.At(1, 2) * idz;
    R.At(2, 0) = M.At(2, 0) * idx; R.At(2, 1) = M.At(2, 1) * idy; R.At(2, 2) = M.At(2, 2) * idz;
    return R;
}

// -------------------- Generadores Aleatorios -------------------
static Vec3 RandUnit(std::mt19937& g) {
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    Vec3 v{ U(g),U(g),U(g) };
    double n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (n < 1e-9) return Vec3(1, 0, 0);
    double inv = 1.0 / n;
    return Vec3(v.x * inv, v.y * inv, v.z * inv);
}
static Vec3 RandVec(std::mt19937& g) {
    std::uniform_real_distribution<double> U(-10.0, 10.0);
    return { U(g),U(g),U(g) };
}

// -------------------- Sistema de Suites -------------------
struct CaseResult { bool ok; std::string name; std::string detail; };
struct Suite {
    std::string title; std::vector<CaseResult> cases; int passed = 0; int total = 0;
    explicit Suite(std::string t) : title(std::move(t)) {}
    void add(bool ok, const std::string& name, const std::string& detail = "") {
        cases.push_back({ ok,name,detail }); total++; if (ok) passed++;
    }
    bool print() const {
        std::cout << CYAN << "\n== " << title << " ==" << RESET << "\n";
        for (auto& c : cases) {
            std::cout << (c.ok ? GREEN : RED) << (c.ok ? "OK" : "KO") << RESET << "  " << c.name;
            if (!c.detail.empty()) std::cout << "  " << (c.ok ? CYAN : YELL) << c.detail << RESET;
            std::cout << "\n";
        }
        std::cout << BOLD << ((passed == total) ? GREEN : (passed ? YELL : RED))
            << "-- " << passed << "/" << total << " subtests" << RESET << "\n";
        return passed == total;
    }
};

// =========================================================================
//                              TESTS LAB 3
// =========================================================================

static void EX1_Test_IsAffine(Suite& S) {
    S.add(M4_IDENTITY.IsAffine(), "Identity.IsAffine()", "Debe ser true");

    Matrix4x4 T = Matrix4x4::Identity();
    T.At(0, 3) = 10.0;
    S.add(T.IsAffine(), "Translation.IsAffine()", "Fila inf [0 0 0 1]");

    Matrix4x4 P = Matrix4x4::Identity();
    P.At(3, 2) = -1.0;
    P.At(3, 3) = 0.0;
    S.add(!P.IsAffine(), "Projection.IsAffine()", "Fila inf modificada -> false");
}

static void EX1_Test_PointVsVector(Suite& S) {
    Vec3 t{ 5.0, -2.0, 3.0 };
    Matrix4x4 M = Matrix4x4::Translate(t);

    Vec3 p{ 1, 1, 1 };
    Vec3 res_p = M.TransformPoint(p);
    S.add(VecEq(res_p, { 6.0, -1.0, 4.0 }), "TransformPoint (Translacio)", "p + t");

    Vec3 v{ 1, 0, 0 };
    Vec3 res_v = M.TransformVector(v);
    S.add(VecEq(res_v, v), "TransformVector (Translacio)", "Invariant");

    Matrix4x4 Ms = Matrix4x4::Scale({ 2.0, 2.0, 2.0 });
    S.add(VecEq(Ms.TransformPoint({ 1,1,1 }), { 2,2,2 }), "TransformPoint (Escala)", "x2");
}

static void EX1_Test_Constructors(Suite& S) {
    // Si Matrix4x4 no incluye Matrix3x3, esto fallará. Asumimos que sí lo hace.
    Matrix3x3 Rz = Matrix3x3::RotationAxisAngle({ 0,0,1 }, PI / 2);
    Matrix4x4 M_Rz = Matrix4x4::Rotate(Rz);
    S.add(VecEq(M_Rz.TransformPoint({ 1,0,0 }), { 0,1,0 }), "Rotate(Matrix3x3)", "Z-90");

    Vec3 t{ 10, 0, 0 };
    Matrix3x3 r = Matrix3x3::Identity();
    Vec3 s{ 2, 2, 2 };
    Matrix4x4 TRS = Matrix4x4::FromTRS(t, r, s);
    S.add(VecEq(TRS.TransformPoint({ 1,0,0 }), { 12, 0, 0 }), "FromTRS", "T * R * S");

    Quat q = Quat::FromAxisAngle({ 0,1,0 }, PI / 2);
    Matrix4x4 M_Q = Matrix4x4::Rotate(q);
    S.add(VecEq(M_Q.TransformPoint({ 0,0,1 }), { 1,0,0 }), "Rotate(Quat)", "Y-90");
}

static void EX2_Test_Inverses(Suite& S) {
    std::mt19937 g(42);
    int N = 20;
    bool all_tr = true;

    for (int i = 0; i < N; ++i) {
        Vec3 t = RandVec(g);
        Matrix3x3 R = Matrix3x3::RotationAxisAngle(RandUnit(g), 1.0);

        Matrix4x4 M = Matrix4x4::FromTRS(t, R, { 1,1,1 });

        Matrix4x4 Inv = M.InverseTR();

        Matrix4x4 I1 = M.Multiply(Inv);
        if (!Mat4Eq(I1, M4_IDENTITY, 1e-3)) { all_tr = false; break; }
    }
    S.add(all_tr, "InverseTR() Roundtrip", "M * M_inv == Identity");
}

static void EX3_Test_Decomposition(Suite& S) {
    Vec3 t_in{ 10.5, -3.2, 4.1 };
    Matrix3x3 r_in = Matrix3x3::RotationAxisAngle({ 0,1,0 }, 0.5);
    Vec3 s_in{ 2.0, 0.5, 3.0 };

    Matrix4x4 M = Matrix4x4::FromTRS(t_in, r_in, s_in);

    Vec3 t_out = GetTraFromMat(M);
    S.add(VecEq(t_in, t_out), "Get Translation (Directo)", "Lee col 3");

    Vec3 s_out = GetSclFromMat(M);
    S.add(VecEq(s_in, s_out), "Get Scale (Calculado)", "Norma columnas");

    Matrix3x3 r_out = GetRotFromMat(M);
    S.add(Mat3Eq(r_in, r_out, 1e-4), "Get Rotation (Calculado)", "Matriz normalizada");
}

// -------------------- Main -----------------------------
int main() {
    std::cout << BOLD << CYAN << "Test Bench Lab 3 (Final)" << RESET << "\n";
    int total_suites = 0, suites_ok = 0;
    auto RUN = [&](Suite& s) { total_suites++; if (s.print()) suites_ok++; };

    { Suite S("[Ex1] Transformaciones Basicas"); EX1_Test_IsAffine(S); EX1_Test_Constructors(S); EX1_Test_PointVsVector(S); RUN(S); }
    { Suite S("[Ex2] Inversas (TR)"); EX2_Test_Inverses(S); RUN(S); }
    { Suite S("[Ex3] Descomposicion (Helpers)"); EX3_Test_Decomposition(S); RUN(S); }

    std::cout << "\n" << ((suites_ok == total_suites) ? GREEN : RED)
        << "Resultado Global: " << suites_ok << "/" << total_suites << " suites OK" << RESET << "\n";
    return (suites_ok == total_suites) ? 0 : 1;
}