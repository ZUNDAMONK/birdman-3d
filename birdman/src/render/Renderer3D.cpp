#include "render/Renderer3D.hpp"
#include "core/Aircraft.hpp"
#include "core/Airframe.hpp"
#include "core/Physics.hpp"
#include "core/Weather.hpp"
#include "core/SiteConst.hpp"
#include "core/DesignIO.hpp"           // exeDirPath (assets読み込み)
#include <SFML/Graphics/Image.hpp>     // PNG読み込み(第7弾: 画像素材)
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
#include <cmath>
#include <random>
#include <algorithm>

// 古いGL/gl.h(1.1)に無い定数(第7弾: 画像テクスチャ用)
#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif
#ifndef GL_GENERATE_MIPMAP
#define GL_GENERATE_MIPMAP 0x8191
#endif

namespace bm {

static const double PI = 3.14159265358979323846;

// ---------- 小物ヘルパー ----------
static void setColor(unsigned hex, float a = 1.0f) {
    glColor4f(((hex >> 16) & 255) / 255.0f, ((hex >> 8) & 255) / 255.0f, (hex & 255) / 255.0f, a);
}
static void faceNormal(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c) {
    glm::dvec3 n = glm::cross(b - a, c - a);
    double len = glm::length(n);
    if (len > 1e-12) { n /= len; glNormal3d(n.x, n.y, n.z); }
}
static void quad(const glm::dvec3& a, const glm::dvec3& b, const glm::dvec3& c, const glm::dvec3& d) {
    faceNormal(a, b, c);
    glVertex3d(a.x, a.y, a.z); glVertex3d(b.x, b.y, b.z); glVertex3d(c.x, c.y, c.z);
    glVertex3d(a.x, a.y, a.z); glVertex3d(c.x, c.y, c.z); glVertex3d(d.x, d.y, d.z);
}
static void drawBox(double cx, double cy, double cz, double sx, double sy, double sz) {
    const double x0 = cx - sx / 2, x1 = cx + sx / 2, y0 = cy - sy / 2, y1 = cy + sy / 2, z0 = cz - sz / 2, z1 = cz + sz / 2;
    glBegin(GL_TRIANGLES);
    quad({x0,y0,z1},{x1,y0,z1},{x1,y1,z1},{x0,y1,z1});
    quad({x1,y0,z0},{x0,y0,z0},{x0,y1,z0},{x1,y1,z0});
    quad({x0,y1,z1},{x1,y1,z1},{x1,y1,z0},{x0,y1,z0});
    quad({x0,y0,z0},{x1,y0,z0},{x1,y0,z1},{x0,y0,z1});
    quad({x1,y0,z1},{x1,y0,z0},{x1,y1,z0},{x1,y1,z1});
    quad({x0,y0,z0},{x0,y0,z1},{x0,y1,z1},{x0,y1,z0});
    glEnd();
}
static void drawSphere(const glm::dvec3& c, double r, int sl = 10, int st = 8, double yScale = 1.0) {
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < st; i++) {
        double p0 = PI * i / st - PI / 2, p1 = PI * (i + 1) / st - PI / 2;
        for (int j = 0; j < sl; j++) {
            double a0 = 2 * PI * j / sl, a1 = 2 * PI * (j + 1) / sl;
            auto P = [&](double ph, double th) {
                return glm::dvec3(c.x + r * std::cos(ph) * std::cos(th),
                                  c.y + r * std::sin(ph) * yScale,
                                  c.z + r * std::cos(ph) * std::sin(th));
            };
            quad(P(p0, a0), P(p0, a1), P(p1, a1), P(p1, a0));
        }
    }
    glEnd();
}
// 円錐: 底面中心(bx,by,bz)、高さh(+y方向)、底面半径w
static void drawCone(double bx, double by, double bz, double w, double h, int seg = 8) {
    glBegin(GL_TRIANGLES);
    for (int j = 0; j < seg; j++) {
        double a0 = 2 * PI * j / seg, a1 = 2 * PI * (j + 1) / seg;
        glm::dvec3 p0(bx + w * std::cos(a0), by, bz + w * std::sin(a0));
        glm::dvec3 p1(bx + w * std::cos(a1), by, bz + w * std::sin(a1));
        glm::dvec3 tip(bx, by + h, bz);
        faceNormal(p0, p1, tip);
        glVertex3d(p0.x, p0.y, p0.z); glVertex3d(p1.x, p1.y, p1.z); glVertex3d(tip.x, tip.y, tip.z);
    }
    glEnd();
}
// 2点間のテーパー円柱 (JS strut)
static void drawStrut(const glm::dvec3& p, const glm::dvec3& q, double r0, double r1, int seg = 8) {
    glm::dvec3 d = q - p;
    double len = glm::length(d);
    if (len < 1e-6) return;
    glm::dvec3 az = d / len;
    glm::dvec3 ax = std::abs(az.y) < 0.99 ? glm::normalize(glm::cross(glm::dvec3(0, 1, 0), az))
                                          : glm::dvec3(1, 0, 0);
    glm::dvec3 ay = glm::cross(az, ax);
    glBegin(GL_TRIANGLES);
    for (int j = 0; j < seg; j++) {
        double a0 = 2 * PI * j / seg, a1 = 2 * PI * (j + 1) / seg;
        auto ring = [&](const glm::dvec3& c, double r, double a) {
            return c + ax * (r * std::cos(a)) + ay * (r * std::sin(a));
        };
        quad(ring(p, r0, a0), ring(p, r0, a1), ring(q, r1, a1), ring(q, r1, a0));
    }
    glEnd();
}
// 水平の平板(y+法線)
static void drawGroundQuad(double cx, double y, double cz, double sx, double sz) {
    // 固定機能GLの霧は「頂点で計算→面内は補間」のため、巨大な1枚クアッドだと
    // 遠方頂点の霧色が手前まで補間されて、視点・角度依存の「水色の帯」が地表に出る。
    // 400m以下のタイルへ分割して頂点密度を確保し、霧の補間誤差をなくす
    const int nx = std::max(1, (int)std::ceil(sx / 400.0));
    const int nz = std::max(1, (int)std::ceil(sz / 400.0));
    const double x0 = cx - sx / 2, z0 = cz - sz / 2, dx = sx / nx, dz = sz / nz;
    glBegin(GL_TRIANGLES);
    glNormal3d(0, 1, 0);
    for (int i = 0; i < nx; i++)
        for (int j = 0; j < nz; j++) {
            const double xa = x0 + i * dx, xb = xa + dx;
            const double za = z0 + j * dz, zb = za + dz;
            glVertex3d(xa, y, za); glVertex3d(xa, y, zb); glVertex3d(xb, y, zb);
            glVertex3d(xa, y, za); glVertex3d(xb, y, zb); glVertex3d(xb, y, za);
        }
    glEnd();
}
// 平らなリング(波紋/パイロンリング)
static void drawFlatRing(const glm::dvec3& c, double rIn, double rOut, int seg = 32) {
    glBegin(GL_TRIANGLES);
    glNormal3d(0, 1, 0);
    for (int j = 0; j < seg; j++) {
        double a0 = 2 * PI * j / seg, a1 = 2 * PI * (j + 1) / seg;
        glm::dvec3 i0(c.x + rIn * std::cos(a0), c.y, c.z + rIn * std::sin(a0));
        glm::dvec3 i1(c.x + rIn * std::cos(a1), c.y, c.z + rIn * std::sin(a1));
        glm::dvec3 o0(c.x + rOut * std::cos(a0), c.y, c.z + rOut * std::sin(a0));
        glm::dvec3 o1(c.x + rOut * std::cos(a1), c.y, c.z + rOut * std::sin(a1));
        glVertex3d(i0.x, i0.y, i0.z); glVertex3d(o0.x, o0.y, o0.z); glVertex3d(o1.x, o1.y, o1.z);
        glVertex3d(i0.x, i0.y, i0.z); glVertex3d(o1.x, o1.y, o1.z); glVertex3d(i1.x, i1.y, i1.z);
    }
    glEnd();
}

// ---------- 植生・山の描画プリセット(第5弾) ----------
// 環境ディスプレイリスト(envList_)に焼き込む静的景観部品。
// すべて決定的乱数(呼び出し側のstd::mt19937)で個体差を付け、
// ポリゴン数は1個あたり数個〜数百トライアングルに抑える。
static double vrnd(std::mt19937& rng) {
    return std::uniform_real_distribution<double>(0, 1)(rng);
}
// 2色の線形補間(標高グラデーション等)
static unsigned lerpColor(unsigned a, unsigned b, double t) {
    t = std::min(1.0, std::max(0.0, t));
    const int r = (int)(((a >> 16) & 255) * (1 - t) + ((b >> 16) & 255) * t);
    const int g = (int)(((a >> 8) & 255) * (1 - t) + ((b >> 8) & 255) * t);
    const int bl = (int)((a & 255) * (1 - t) + (b & 255) * t);
    return (unsigned)((r << 16) | (g << 8) | bl);
}

// 雑草の株: 細い三角ブレード5〜8枚を放射状に。緑〜黄緑の個体差(~6-8三角形)
static void drawGrassTuft(std::mt19937& rng, const glm::dvec3& pos, double scale) {
    const int nB = 5 + (int)(vrnd(rng) * 3.99);
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < nB; i++) {
        const double a = 2 * PI * i / nB + vrnd(rng) * 0.8;
        const double lean = 0.25 + vrnd(rng) * 0.45;             // 外側への倒れ
        const double h = scale * (0.45 + vrnd(rng) * 0.5);
        const double w = scale * 0.05;
        setColor(lerpColor(0x5d8a3c, 0x9ab54a, vrnd(rng)));      // 緑〜黄緑
        const glm::dvec3 dir(std::cos(a), 0, std::sin(a));
        const glm::dvec3 side(-dir.z * w, 0, dir.x * w);
        const glm::dvec3 b0 = pos - side, b1 = pos + side;
        const glm::dvec3 tip = pos + dir * (h * lean) + glm::dvec3(0, h, 0);
        faceNormal(b0, b1, tip);
        glVertex3d(b0.x, b0.y, b0.z); glVertex3d(b1.x, b1.y, b1.z); glVertex3d(tip.x, tip.y, tip.z);
    }
    glEnd();
}

// 灌木: 潰れた球2〜3個の重なり、暗緑(~80-120三角形)
static void drawBush(std::mt19937& rng, const glm::dvec3& pos, double r) {
    const int n = 2 + (int)(vrnd(rng) * 1.99);
    for (int i = 0; i < n; i++) {
        setColor(lerpColor(0x2f5230, 0x47703f, vrnd(rng)));
        const glm::dvec3 c = pos + glm::dvec3((vrnd(rng) - 0.5) * r * 0.9,
                                              r * (0.30 + vrnd(rng) * 0.15),
                                              (vrnd(rng) - 0.5) * r * 0.9);
        drawSphere(c, r * (0.55 + vrnd(rng) * 0.25), 6, 4, 0.62);
    }
}

// 広葉樹: 幹(細い箱)+樹冠(球2〜3個、緑の濃淡)(~90-150三角形)
static void drawTreeBroadleaf(std::mt19937& rng, const glm::dvec3& pos, double h) {
    const double trunkH = h * 0.38;
    setColor(0x6b543c);
    drawBox(pos.x, pos.y + trunkH / 2, pos.z, h * 0.055, trunkH, h * 0.055);
    const int n = 2 + (int)(vrnd(rng) * 1.99);
    for (int i = 0; i < n; i++) {
        setColor(lerpColor(0x3c6b35, 0x5d8a46, vrnd(rng)));
        const double cr = h * (0.26 + vrnd(rng) * 0.10);
        const glm::dvec3 c = pos + glm::dvec3((vrnd(rng) - 0.5) * h * 0.30,
                                              trunkH + cr * 0.55 + vrnd(rng) * h * 0.16,
                                              (vrnd(rng) - 0.5) * h * 0.30);
        drawSphere(c, cr, 6, 4, 0.85);
    }
}

// (第7弾: プロシージャルの針葉樹drawTreeConifer/ヨシ原drawReedBedは
//  画像スプライト(pine_cluster/conifer_cluster/reed_01〜04)へ置換したため削除)

// リアルな山塊: メインコーン+肩の小コーン2〜3個。標高で色を変える
// (麓=濃緑→中腹=緑灰→山頂=岩灰、h>600で山頂に薄い白=残雪)(~60-100三角形)
static void drawMountainRidge(std::mt19937& rng, const glm::dvec3& pos, double w, double h,
                              unsigned footCol) {
    const unsigned midCol = lerpColor(footCol, 0x8b95a0, 0.45);  // 中腹=緑灰
    const unsigned topCol = lerpColor(footCol, 0x9aa2ab, 0.75);  // 山頂=岩灰
    // 3段スタックのコーンで標高グラデーション(下から順に描き重ねる)
    setColor(footCol);
    drawCone(pos.x, pos.y, pos.z, w, h * 0.55, 8);
    setColor(midCol);
    drawCone(pos.x, pos.y + h * 0.30, pos.z, w * 0.62, h * 0.48, 8);
    setColor(topCol);
    drawCone(pos.x, pos.y + h * 0.60, pos.z, w * 0.34, h * 0.40, 8);
    if (h > 600) {                                               // 高山は残雪
        setColor(0xf2f5f8);
        drawCone(pos.x, pos.y + h * 0.82, pos.z, w * 0.17, h * 0.185, 8);
    }
    // 肩の小コーン2〜3個(山塊の重厚さ)
    const int sh = 2 + (int)(vrnd(rng) * 1.99);
    for (int i = 0; i < sh; i++) {
        const double a = vrnd(rng) * 2 * PI;
        const double d = w * (0.45 + vrnd(rng) * 0.35);
        const double sw = w * (0.35 + vrnd(rng) * 0.25);
        const double shH = h * (0.30 + vrnd(rng) * 0.25);
        setColor(lerpColor(footCol, midCol, vrnd(rng) * 0.5));
        drawCone(pos.x + std::cos(a) * d, pos.y, pos.z + std::sin(a) * d, sw, shH, 7);
    }
}

// ---------- 翼型 ----------
struct Profile { std::vector<std::pair<double,double>> up, lo; };
static const Profile& mainAirfoil() {   // NACA風 camber4% 厚11% (JS airfoilPts)
    static Profile P = [] {
        Profile p;
        const int n = 16;
        for (int i = 0; i <= n; i++) {
            double x = 0.5 * (1 - std::cos(PI * i / n));
            double t = 0.11, m = 0.04, pp = 0.4;
            double yt = 5 * t * (0.2969 * std::sqrt(x) - 0.126 * x - 0.3516 * x * x + 0.2843 * x * x * x - 0.1036 * x * x * x * x);
            double yc = x < pp ? m / (pp * pp) * (2 * pp * x - x * x)
                               : m / ((1 - pp) * (1 - pp)) * ((1 - 2 * pp) + 2 * pp * x - x * x);
            p.up.push_back({x, yc + yt});
            p.lo.push_back({x, yc - yt});
        }
        return p;
    }();
    return P;
}
static const Profile& thinAirfoil() {   // 尾翼用 対称8% (JS airfoilThin)
    static Profile P = [] {
        Profile p;
        const int n = 10;
        for (int i = 0; i <= n; i++) {
            double x = 0.5 * (1 - std::cos(PI * i / n));
            double yt = 5 * 0.08 * (0.2969 * std::sqrt(x) - 0.126 * x - 0.3516 * x * x + 0.2843 * x * x * x - 0.1036 * x * x * x * x);
            p.up.push_back({x, yt});
            p.lo.push_back({x, -yt});
        }
        return p;
    }();
    return P;
}

struct Station { double x, chord, y0, z0; };

// 翼ロフト: 断面ループ(上面f1→f0、下面f0→f1)をステーション間に張る (JS loftWing)
static void loftWing(const Profile& AF, const std::vector<Station>& stn, double f0, double f1, bool closed) {
    std::vector<std::pair<double,double>> prof;
    for (auto it = AF.up.rbegin(); it != AF.up.rend(); ++it)
        if (it->first >= f0 - 1e-6 && it->first <= f1 + 1e-6) prof.push_back(*it);
    for (const auto& p : AF.lo)
        if (p.first >= f0 - 1e-6 && p.first <= f1 + 1e-6) prof.push_back(p);
    const int m = (int)prof.size(), n = (int)stn.size();
    if (m < 2 || n < 2) return;
    auto P = [&](int i, int j) {
        const Station& s = stn[i];
        return glm::dvec3(s.x, s.y0 + prof[j].second * s.chord, s.z0 + prof[j].first * s.chord);
    };
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < n - 1; i++)
        for (int j = 0; j < m - 1; j++)
            quad(P(i, j), P(i + 1, j), P(i + 1, j + 1), P(i, j + 1));
    if (closed) {
        for (int j = 0; j < m - 2; j++) {
            glm::dvec3 a = P(n - 1, 0), b = P(n - 1, j + 1), c = P(n - 1, j + 2);
            faceNormal(a, b, c);
            glVertex3d(a.x, a.y, a.z); glVertex3d(b.x, b.y, b.z); glVertex3d(c.x, c.y, c.z);
        }
    }
    glEnd();
}
// (旧loftSkinはMeshEmit::loftSkinに置き換え済み)

// ---------- FlexMesh用エミッタ(主翼のたわみ変形対象ジオメトリ) ----------
namespace {
struct MeshEmit {
    FlexMesh* m = nullptr;
    const FlexBasis* fb = nullptr;
    float r = 1, g = 1, b = 1, a = 1;
    void color(unsigned hex, float alpha = 1.0f) {
        r = ((hex >> 16) & 255) / 255.0f;
        g = ((hex >> 8) & 255) / 255.0f;
        b = (hex & 255) / 255.0f;
        a = alpha;
    }
    void vert(const glm::dvec3& p, const glm::dvec3& n) {
        m->base.push_back((float)p.x); m->base.push_back((float)p.y); m->base.push_back((float)p.z);
        m->nrm.push_back((float)n.x); m->nrm.push_back((float)n.y); m->nrm.push_back((float)n.z);
        m->col.push_back(r); m->col.push_back(g); m->col.push_back(b); m->col.push_back(a);
        const double span = std::abs(p.x);
        m->dL.push_back((float)interpA(fb->ys, fb->dL, span));
        m->dW.push_back((float)interpA(fb->ys, fb->dW, span));
    }
    void tri(const glm::dvec3& p0, const glm::dvec3& p1, const glm::dvec3& p2) {
        glm::dvec3 n = glm::cross(p1 - p0, p2 - p0);
        const double len = glm::length(n);
        n = len > 1e-12 ? n / len : glm::dvec3(0, 1, 0);
        vert(p0, n); vert(p1, n); vert(p2, n);
    }
    void quad(const glm::dvec3& p0, const glm::dvec3& p1, const glm::dvec3& p2, const glm::dvec3& p3) {
        tri(p0, p1, p2); tri(p0, p2, p3);
    }
    void strut(const glm::dvec3& p, const glm::dvec3& q, double r0, double r1, int seg = 8) {
        glm::dvec3 d = q - p;
        const double len = glm::length(d);
        if (len < 1e-6) return;
        glm::dvec3 az = d / len;
        glm::dvec3 ax = std::abs(az.y) < 0.99 ? glm::normalize(glm::cross(glm::dvec3(0, 1, 0), az)) : glm::dvec3(1, 0, 0);
        glm::dvec3 ay = glm::cross(az, ax);
        for (int j = 0; j < seg; j++) {
            const double a0 = 2 * PI * j / seg, a1 = 2 * PI * (j + 1) / seg;
            auto ring = [&](const glm::dvec3& c, double rr, double ang) {
                return c + ax * (rr * std::cos(ang)) + ay * (rr * std::sin(ang));
            };
            quad(ring(p, r0, a0), ring(p, r0, a1), ring(q, r1, a1), ring(q, r1, a0));
        }
    }
    void loftWing(const Profile& AF, const std::vector<Station>& stn, double f0, double f1, bool closed) {
        std::vector<std::pair<double, double>> prof;
        for (auto it = AF.up.rbegin(); it != AF.up.rend(); ++it)
            if (it->first >= f0 - 1e-6 && it->first <= f1 + 1e-6) prof.push_back(*it);
        for (const auto& pp : AF.lo)
            if (pp.first >= f0 - 1e-6 && pp.first <= f1 + 1e-6) prof.push_back(pp);
        const int mm = (int)prof.size(), n = (int)stn.size();
        if (mm < 2 || n < 2) return;
        auto P = [&](int i, int j) {
            const Station& s = stn[i];
            return glm::dvec3(s.x, s.y0 + prof[j].second * s.chord, s.z0 + prof[j].first * s.chord);
        };
        for (int i = 0; i < n - 1; i++)
            for (int j = 0; j < mm - 1; j++)
                quad(P(i, j), P(i + 1, j), P(i + 1, j + 1), P(i, j + 1));
        if (closed)
            for (int j = 0; j < mm - 2; j++)
                tri(P(n - 1, 0), P(n - 1, j + 1), P(n - 1, j + 2));
    }
    void loftSkin(const Profile& AF, const std::vector<Station>& stn, double f0, double f1, bool up) {
        if (f1 - f0 <= 1e-4) return;
        std::vector<std::pair<double, double>> prof;
        for (const auto& pp : (up ? AF.up : AF.lo))
            if (pp.first >= f0 - 1e-6 && pp.first <= f1 + 1e-6) prof.push_back(pp);
        const int mm = (int)prof.size(), n = (int)stn.size();
        if (mm < 2 || n < 2) return;
        auto P = [&](int i, int j) {
            const Station& s = stn[i];
            return glm::dvec3(s.x, s.y0 + prof[j].second * s.chord + (up ? 0.003 : -0.003),
                              s.z0 + prof[j].first * s.chord);
        };
        for (int i = 0; i < n - 1; i++)
            for (int j = 0; j < mm - 1; j++)
                quad(P(i, j), P(i + 1, j), P(i + 1, j + 1), P(i, j + 1));
    }
};
} // namespace

static void drawFlexMesh(const FlexMesh& m) {
    if (m.pos.empty()) return;
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, m.pos.data());
    glNormalPointer(GL_FLOAT, 0, m.nrm.data());
    glColorPointer(4, GL_FLOAT, 0, m.col.data());
    glDrawArrays(GL_TRIANGLES, 0, (GLsizei)m.count());
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
}

static std::vector<Station> wingStations(const AircraftParams& p, int side, int N = 22) {
    const double half = p.span / 2;
    const double dih = (p.jig == "flat" ? 0 : p.dihedral) * PI / 180;
    std::vector<Station> arr;
    for (int i = 0; i <= N; i++) {
        double t = (double)i / N, y = t * half;
        arr.push_back({side * y, chordAt(p, t), std::tan(dih) * y, 0});
    }
    return arr;
}

// ---------- 画像素材の読み込み・スプライト描画(第7弾) ----------
// assets/textures/<name>.png を読み込んでGLテクスチャ化する。
//  - cutLeftFrac>0: 画像左端の混入オブジェクト対策(その範囲のαを0に潰す)
//  - アルファ無し画像は白色キー(R,G,B>245→α0)で透過化
//  - アルファ有効範囲のbboxを計算し、UVをその範囲へクロップ(余白除去)
const SpriteTex& Renderer3D::loadSprite(const std::string& name, double cutLeftFrac) {
    auto it = sprites_.find(name);
    if (it != sprites_.end()) return it->second;
    SpriteTex t;
    sf::Image img;
    const std::string rel = "assets/textures/" + name + ".png";
    if (!img.loadFromFile(exeDirPath() + "/" + rel) && !img.loadFromFile(rel))
        return sprites_[name] = t;                       // ok=false(呼び出し側でスキップ)
    const unsigned W = img.getSize().x, H = img.getSize().y;
    // アルファチャンネルの有無を確認(全ピクセル不透明→白背景とみなし白色キー)
    bool hasAlpha = false;
    for (unsigned y = 0; y < H && !hasAlpha; y += 4)
        for (unsigned x = 0; x < W; x += 4)
            if (img.getPixel(x, y).a < 250) { hasAlpha = true; break; }
    const unsigned cutX = (unsigned)(W * clamp(cutLeftFrac, 0.0, 0.95));
    for (unsigned y = 0; y < H; y++)
        for (unsigned x = 0; x < W; x++) {
            const sf::Color c = img.getPixel(x, y);
            if (x < cutX) img.setPixel(x, y, sf::Color(c.r, c.g, c.b, 0));
            else if (!hasAlpha && c.r > 245 && c.g > 245 && c.b > 245)
                img.setPixel(x, y, sf::Color(c.r, c.g, c.b, 0));
        }
    // アルファ有効範囲のbbox
    unsigned x0 = W, y0 = H, x1 = 0, y1 = 0;
    for (unsigned y = 0; y < H; y++)
        for (unsigned x = 0; x < W; x++)
            if (img.getPixel(x, y).a >= 32) {
                x0 = std::min(x0, x); x1 = std::max(x1, x);
                y0 = std::min(y0, y); y1 = std::max(y1, y);
            }
    if (x0 >= x1 || y0 >= y1) { x0 = 0; y0 = 0; x1 = W - 1; y1 = H - 1; }
    glGenTextures(1, &t.id);
    glBindTexture(GL_TEXTURE_2D, t.id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, (int)W, (int)H, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                 img.getPixelsPtr());
    glBindTexture(GL_TEXTURE_2D, 0);
    t.u0 = x0 / (double)W; t.u1 = (x1 + 1.0) / W;
    t.v0 = y0 / (double)H; t.v1 = (y1 + 1.0) / H;      // v0=画像上端(クワッド上辺に対応)
    t.aspect = (x1 - x0 + 1.0) / (y1 - y0 + 1.0);
    t.ok = true;
    return sprites_[name] = t;
}

// タイルテクスチャ(水面・アスファルト): GL_REPEAT+ミップマップ。失敗時0
unsigned Renderer3D::loadTileTexture(const std::string& name) {
    sf::Image img;
    const std::string rel = "assets/textures/" + name + ".png";
    if (!img.loadFromFile(exeDirPath() + "/" + rel) && !img.loadFromFile(rel)) return 0;
    unsigned id = 0;
    glGenTextures(1, &id);
    glBindTexture(GL_TEXTURE_2D, id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, (int)img.getSize().x, (int)img.getSize().y, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, img.getPixelsPtr());
    glBindTexture(GL_TEXTURE_2D, 0);
    return id;
}

// スプライト描画の共通GLステート(ライティング無効+アルファテスト=カットアウト描画。
// 深度ソート不要で envList_ に静的に焼き込める)
static void spriteStateOn() {
    glDisable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_ALPHA_TEST);
    glAlphaFunc(GL_GREATER, 0.5f);
    glColor4f(1, 1, 1, 1);
}
static void spriteStateOff() {
    glDisable(GL_ALPHA_TEST);
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);
    glEnable(GL_LIGHTING);
}

// 1枚クワッド(縦立て): pos=接地点(底辺中心)、h=高さ、yawDeg=幅方向の向き
void Renderer3D::spriteQuad(const SpriteTex& t, const glm::dvec3& pos, double h, double yawDeg) {
    if (!t.ok) return;
    const double w = h * t.aspect;
    const double a = yawDeg * PI / 180.0;
    const glm::dvec3 r(std::cos(a) * w / 2, 0, std::sin(a) * w / 2);
    const glm::dvec3 up(0, h, 0);
    glBindTexture(GL_TEXTURE_2D, t.id);
    glBegin(GL_TRIANGLES);
    glNormal3d(0, 1, 0);
    const glm::dvec3 A = pos - r, B = pos + r, C = pos + r + up, D = pos - r + up;
    glTexCoord2d(t.u0, t.v1); glVertex3d(A.x, A.y, A.z);
    glTexCoord2d(t.u1, t.v1); glVertex3d(B.x, B.y, B.z);
    glTexCoord2d(t.u1, t.v0); glVertex3d(C.x, C.y, C.z);
    glTexCoord2d(t.u0, t.v1); glVertex3d(A.x, A.y, A.z);
    glTexCoord2d(t.u1, t.v0); glVertex3d(C.x, C.y, C.z);
    glTexCoord2d(t.u0, t.v0); glVertex3d(D.x, D.y, D.z);
    glEnd();
}

// 十字クワッド(直交2枚): 植生・小物用。どの方向から見ても立体感が出る
void Renderer3D::spriteCross(const SpriteTex& t, const glm::dvec3& pos, double h) {
    spriteQuad(t, pos, h, 0);
    spriteQuad(t, pos, h, 90);
}

// 水平置き1枚(スイレン等): pos=中心(y=水面高さ)、size=奥行き
void Renderer3D::spriteFlat(const SpriteTex& t, const glm::dvec3& pos, double size) {
    if (!t.ok) return;
    const double w = size * t.aspect / 2, d = size / 2;
    glBindTexture(GL_TEXTURE_2D, t.id);
    glBegin(GL_TRIANGLES);
    glNormal3d(0, 1, 0);
    glTexCoord2d(t.u0, t.v1); glVertex3d(pos.x - w, pos.y, pos.z + d);
    glTexCoord2d(t.u1, t.v1); glVertex3d(pos.x + w, pos.y, pos.z + d);
    glTexCoord2d(t.u1, t.v0); glVertex3d(pos.x + w, pos.y, pos.z - d);
    glTexCoord2d(t.u0, t.v1); glVertex3d(pos.x - w, pos.y, pos.z + d);
    glTexCoord2d(t.u1, t.v0); glVertex3d(pos.x + w, pos.y, pos.z - d);
    glTexCoord2d(t.u0, t.v0); glVertex3d(pos.x - w, pos.y, pos.z - d);
    glEnd();
}

// ---------- 初期化 ----------
void Renderer3D::init() {
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
    const float amb[4] = {0.55f, 0.58f, 0.62f, 1.0f};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, amb);
    const float dif[4] = {0.95f, 0.90f, 0.82f, 1.0f};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, dif);
    glEnable(GL_FOG);
    glFogi(GL_FOG_MODE, GL_LINEAR);
    glFogf(GL_FOG_START, 800.0f);
    glFogf(GL_FOG_END, 11000.0f);
    glEnable(GL_NORMALIZE);

    // さざ波テクスチャ (JS waterTexture 相当を手続き生成)
    {
        const int SZ = 256;
        std::vector<unsigned char> px(SZ * SZ * 3);
        std::mt19937 rng(7);
        std::uniform_real_distribution<double> rnd(0, 1);
        for (int i = 0; i < SZ * SZ; i++) { px[i*3] = 0x6f; px[i*3+1] = 0x9e; px[i*3+2] = 0xc6; }
        auto streak = [&](int n, int rr, int gg, int bb, double a0, double av, int wMin, int wMax, int th) {
            for (int k = 0; k < n; k++) {
                int x = (int)(rnd(rng) * SZ), y = (int)(rnd(rng) * SZ);
                int w = wMin + (int)(rnd(rng) * (wMax - wMin));
                double al = a0 + rnd(rng) * av;
                for (int dx = 0; dx < w; dx++)
                    for (int dy = 0; dy < th; dy++) {
                        int xx = (x + dx) % SZ, yy = (y + dy + (int)(2 * std::sin(dx * 0.2))) % SZ;
                        if (yy < 0) yy += SZ;
                        unsigned char* p = &px[(yy * SZ + xx) * 3];
                        p[0] = (unsigned char)(p[0] * (1 - al) + rr * al);
                        p[1] = (unsigned char)(p[1] * (1 - al) + gg * al);
                        p[2] = (unsigned char)(p[2] * (1 - al) + bb * al);
                    }
            }
        };
        streak(50, 0x2b, 0x4a, 0x6e, 0.05, 0.08, 30, 90, 3);   // 暗い揺らぎ
        streak(90, 255, 255, 255, 0.06, 0.12, 14, 54, 2);      // 白い波頭
        glGenTextures(1, &waterTex_);
        glBindTexture(GL_TEXTURE_2D, waterTex_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        // ミップマップ必須: 無いと遠距離・浅い視線角でエイリアシングして灰色に見える
#ifndef GL_GENERATE_MIPMAP
#define GL_GENERATE_MIPMAP 0x8191
#endif
        glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, SZ, SZ, 0, GL_RGB, GL_UNSIGNED_BYTE, px.data());
        glBindTexture(GL_TEXTURE_2D, 0);
    }
    buildEnvironment();
    // 設計モードの中立グリッド床
    designList_ = glGenLists(1);
    glNewList(designList_, GL_COMPILE);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    for (int i = -40; i <= 40; i++) {
        bool major = i % 5 == 0;
        // 黒基調の設計モードに合わせ、薄い青グレー(alpha低め相当の暗さ)に変更
        setColor(major ? 0x3a5570 : 0x1c2733);
        glVertex3d(i, 0, -40); glVertex3d(i, 0, 40);
        glVertex3d(-40, 0, i); glVertex3d(40, 0, i);
    }
    glEnd();
    glEnable(GL_LIGHTING);
    setColor(0xd9534f); drawBox(1.5, 0.02, 0, 3, 0.04, 0.04);   // X軸マーカー
    setColor(0x4a90d9); drawBox(0, 0.02, 1.5, 0.04, 0.04, 3);   // Z軸マーカー
    glEndList();
}

void Renderer3D::resize(int w, int h) { vpW_ = std::max(1, w); vpH_ = std::max(1, h); }

// ---------- 環境(琵琶湖) ----------
void Renderer3D::buildEnvironment() {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> rnd(0, 1);
    if (!biwaWaterTex_) biwaWaterTex_ = loadTileTexture("biwako_water_tile");
    envList_ = glGenLists(1);
    glNewList(envList_, GL_COMPILE);

    // ==== 琵琶湖: 湖岸ポリゴンから水面と周囲の陸地を生成 ====
    // コース座標(x=前方, yl=右) → ワールド(X=yl, Z=-x)
    {
        const auto& shore = biwaShore();
        auto toW = [](double x, double yl) { return glm::dvec3(yl, 0, -x); };
        glm::dvec3 cen(0);
        for (const auto& p : shore) cen += toW(p.first, p.second);
        cen /= (double)shore.size();
        // 水面(非ライティング: 視点や太陽方向で灰色に濁らないよう、
        // テクスチャ色をそのまま表示。ミップマップで遠距離のギラつきも防止)
        // 第7弾: 琵琶湖はユーザー素材のタイル(biwako_water_tile)に置換。
        // 頂点色でトーンを従来の青系に寄せる。読み込み失敗時は従来のプロシージャルへ
        glDisable(GL_LIGHTING);
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, biwaWaterTex_ ? biwaWaterTex_ : waterTex_);
        setColor(biwaWaterTex_ ? 0xe6f2fa : 0xdceef8);
        const double TR = 62.5;   // テクスチャ繰り返し周期 m(~60m)
        glBegin(GL_TRIANGLES);
        for (size_t i = 0; i < shore.size(); i++) {
            const auto& a = shore[i];
            const auto& b = shore[(i + 1) % shore.size()];
            glm::dvec3 pa = toW(a.first, a.second), pb = toW(b.first, b.second);
            glTexCoord2d(cen.x / TR, cen.z / TR); glVertex3d(cen.x, -0.35, cen.z);
            glTexCoord2d(pb.x / TR, pb.z / TR);   glVertex3d(pb.x, -0.35, pb.z);
            glTexCoord2d(pa.x / TR, pa.z / TR);   glVertex3d(pa.x, -0.35, pa.z);
        }
        glEnd();
        glBindTexture(GL_TEXTURE_2D, 0);
        glDisable(GL_TEXTURE_2D);
        glEnable(GL_LIGHTING);
        // 周囲の陸地(湖岸→外周60kmへのリング)。
        // 地面レイヤーはY差がcm単位のため遠距離で深度が量子化して透ける(Zファイト)。
        // ポリゴンオフセットで重なり順を強制する(値が大きいほど手前に出る)
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(-1.0f, -1.0f);
        const unsigned landCols[3] = {0x8fae7e, 0x86a577, 0x93b184};
        for (size_t i = 0; i < shore.size(); i++) {
            const auto& a = shore[i];
            const auto& b = shore[(i + 1) % shore.size()];
            glm::dvec3 pa = toW(a.first, a.second), pb = toW(b.first, b.second);
            glm::dvec3 oa = cen + glm::normalize(pa - cen) * 60000.0;
            glm::dvec3 ob = cen + glm::normalize(pb - cen) * 60000.0;
            setColor(landCols[i % 3]);
            glBegin(GL_TRIANGLES);
            quad({pa.x, 0.02, pa.z}, {pb.x, 0.02, pb.z}, {ob.x, 0.02, ob.z}, {oa.x, 0.02, oa.z});
            glEnd();
        }
        glPolygonOffset(-2.0f, -2.0f);
        for (size_t i = 0; i < shore.size(); i++) {
            const auto& a = shore[i];
            const auto& b = shore[(i + 1) % shore.size()];
            glm::dvec3 pa = toW(a.first, a.second), pb = toW(b.first, b.second);
            glm::dvec3 oa = cen + glm::normalize(pa - cen) * 60000.0;
            glm::dvec3 ob = cen + glm::normalize(pb - cen) * 60000.0;
            // 汀線(薄い砂色の帯)
            glm::dvec3 sa = pa + glm::normalize(oa - pa) * 60.0;
            glm::dvec3 sb = pb + glm::normalize(ob - pb) * 60.0;
            setColor(0xd9cda8);
            glBegin(GL_TRIANGLES);
            quad({pa.x, 0.03, pa.z}, {pb.x, 0.03, pb.z}, {sb.x, 0.03, sb.z}, {sa.x, 0.03, sa.z});
            glEnd();
        }
    }

    // 発進地周辺の岸(東岸の会場)。岸線はx=-50(ワールドz=+50)へ後退済みで、
    // プラットフォームは湖上に立つ(発進失敗=着水)。陸リングより手前に重ねる。
    glPolygonOffset(-3.0f, -3.0f);
    setColor(0x8fae7e); drawGroundQuad(0, 0.035, 1085, 260, 2070);   // 草地(z=50..2120)
    glPolygonOffset(-4.0f, -4.0f);
    setColor(0xd9cda8); drawGroundQuad(0, 0.038, 52, 260, 10);       // 波打ち際(z≈50)
    glDisable(GL_POLYGON_OFFSET_FILL);

    // 発進プラットフォーム(物理のdeckHeightAtと一致: 前縁z=0で高10.0m、
    // 後方へ3.5°上り。助走路10m(z=0..10)+尾部を載せる延長部(z=10..13))
    // 岸から約50m沖の湖上に立つ(座標は従来通り原点。岸線の方を後退させた)
    setColor(0xb8a888);
    glPushMatrix();
    glTranslated(0, 10.167, 6);
    glRotated(-3.5, 1, 0, 0);
    drawBox(0, 0, 0, 9, 0.4, 14);
    glDisable(GL_LIGHTING);
    setColor(0xffffff); drawBox(0, 0.21, -5.7, 9, 0.02, 0.4);   // 前縁ライン
    setColor(0xcc4433); drawBox(0, 0.21, 4.0, 9, 0.02, 0.3);    // 助走開始ライン(x=-10)
    glEnable(GL_LIGHTING);
    glPopMatrix();
    // 支柱: 水面(y=-0.35)まで延長
    setColor(0x6b5d49);
    for (double dx : {-4.0, 4.0}) for (double dz : {0.5, 4.0, 8.0, 12.0}) {
        const double hL = 10.0 + 0.0612 * dz - 0.3;
        drawBox(dx, (hL - 0.4) / 2, dz, 0.4, hL + 0.4, 0.4);
    }
    // 岸→プラットフォームの木製桟橋(幅3m。板張り+水面までの支柱)
    {
        setColor(0x9a7f5f);
        drawBox(0, 0.82, 31.5, 3.0, 0.14, 39.0);                 // 板張り(z=12..51)
        setColor(0x8a6f4f);
        drawBox(-1.35, 0.98, 31.5, 0.12, 0.10, 39.0);            // 両縁の手すり土台
        drawBox( 1.35, 0.98, 31.5, 0.12, 0.10, 39.0);
        setColor(0x6b5d49);
        for (double dz : {15.0, 22.0, 29.0, 36.0, 43.0, 49.0})
            for (double dx : {-1.2, 1.2})
                drawBox(dx, 0.18, dz, 0.18, 1.3, 0.18);          // 支柱(水面下y=-0.47まで)
    }

    // マラソンコースのパイロン(北/南 約11km)とブイ
    const double NP[2] = {0, -11000}, SP[2] = {-6300, -9020};
    auto pylon = [&](double x, double z, unsigned col) {
        setColor(0xffffff); drawStrut({x, 0, z}, {x, 90, z}, 4, 2.5, 8);
        setColor(col); drawSphere({x, 98, z}, 16, 14, 12);
        glDisable(GL_LIGHTING);
        setColor(col); drawFlatRing({x, 0.5, z}, 58.5, 61.5, 40);
        glEnable(GL_LIGHTING);
    };
    auto buoyLine = [&](double px, double pz) {
        double len = std::hypot(px, pz), ux = px / len, uz = pz / len, qx = -uz, qz = ux;
        for (double d = 100; d < len - 150; d += (d < 2000 ? 100 : 500)) {
            bool km = std::fmod(std::round(d), 1000.0) == 0, big = std::fmod(std::round(d), 500.0) == 0;
            double r = km ? 1.6 : big ? 1.0 : 0.5;
            unsigned col = km ? 0xd9342b : 0xe8a23b;
            for (int s : {1, -1}) {
                double bx = ux * d + qx * 14 * s, bz = uz * d + qz * 14 * s;
                if (!insideLake(-bz, bx)) continue;   // 岸線後退で陸に載るブイはスキップ
                setColor(col); drawSphere({bx, r * 0.5, bz}, r, 8, 6);
                setColor(0xffffff); drawStrut({bx, r * 0.2, bz}, {bx, r * 2.4, bz}, 0.05, 0.05, 5);
            }
        }
    };
    buoyLine(NP[0], NP[1]); buoyLine(SP[0], SP[1]);
    pylon(NP[0], NP[1], 0xd9342b);
    pylon(SP[0], SP[1], 0x2c6fd9);

    // 遠景の山並み: 湖岸ポリラインに沿って外側へ配置。
    // 西岸=比良山地(高い)、北=湖北の山、東岸=会場側の低い丘
    {
        const auto& shore = biwaShore();
        auto toW = [](double x, double yl) { return glm::dvec3(yl, 0, -x); };
        glm::dvec3 cen(0);
        for (const auto& p : shore) cen += toW(p.first, p.second);
        cen /= (double)shore.size();
        // 第5弾: 単純コーン→drawMountainRidge(標高グラデーション+肩コーン)にグレードアップ。
        // 西岸=比良山地は高く重厚、東岸=会場側は低い丘。低い丘の麓には針葉樹クラスタを添える
        const unsigned colsWest[3] = {0x4f7057, 0x466a52, 0x59785f};   // 比良の麓=濃緑
        const unsigned colsEast[3] = {0x6d8a68, 0x628060, 0x759072};   // 東岸の丘の麓
        for (size_t i = 0; i < shore.size(); i++) {
            const auto& a = shore[i];
            const auto& b = shore[(i + 1) % shore.size()];
            glm::dvec3 pa = toW(a.first, a.second), pb = toW(b.first, b.second);
            const double len = glm::length(pb - pa);
            const int nM = std::max(1, (int)(len / 1300));
            for (int j = 0; j < nM; j++) {
                const double t = (j + 0.5) / nM;
                glm::dvec3 p = pa + (pb - pa) * t;
                glm::dvec3 out = glm::normalize(p - cen);
                const double west = -(a.second + b.second) / 2;   // yl<0側=西
                const bool isWest = west > 4000;
                const bool isEast = west < -800;
                p += out * (900 + rnd(rng) * 2200);
                double h = isWest ? (420 + rnd(rng) * 480)        // 比良山地: 残雪ラインを超える峰も
                         : isEast ? (130 + rnd(rng) * 170)
                                  : (250 + rnd(rng) * 300);
                double w = 290 + rnd(rng) * 320;
                // 会場の滑走路コリドー(z=8..2014, 草地幅±130)に重なる丘は除外
                // (岸から900..3100m外側に置くため、2km内陸まで伸びる滑走路と衝突しうる)
                if (std::abs(p.x) < w + 200 && p.z > -w && p.z < 2100 + w) continue;
                drawMountainRidge(rng, {p.x, -4, p.z}, w, h,
                                  isEast ? colsEast[(i + j) % 3] : colsWest[(i + j) % 3]);
            }
        }
        // 竹生島(北湖の小島)
        setColor(0x6f8a6a);
        drawCone(-6000, -2, -16000, 220, 70, 7);
    }
    // 雲(扁平球・2層: 低層=大きく濃い/高層=小さく淡い)
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    setColor(0xffffff, 0.5f);
    for (int i = 0; i < 12; i++)
        drawSphere({-500 + rnd(rng) * 1000, 70 + rnd(rng) * 50, -200 - rnd(rng) * 1200},
                   10 + rnd(rng) * 14, 6, 5, 0.4);
    setColor(0xffffff, 0.25f);
    for (int i = 0; i < 8; i++)
        drawSphere({-600 + rnd(rng) * 1200, 220 + rnd(rng) * 120, -300 - rnd(rng) * 1600},
                   6 + rnd(rng) * 8, 6, 5, 0.4);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glEndList();
}

void Renderer3D::setSite(const std::string& site) {
    if (site_ == site && envList_) return;
    site_ = site;
    if (envList_) { glDeleteLists(envList_, 1); envList_ = 0; }
    if (site_ == "fujikawa") buildEnvFujikawa(); else buildEnvironment();
}

// ---------- 富士川滑空場(サイト2): 河川敷滑走路・富士川・駿河湾・富士山 ----------
void Renderer3D::buildEnvFujikawa() {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> rnd(0, 1);
    if (!asphaltTex_) asphaltTex_ = loadTileTexture("fujikawa_runway_asphalt_tile");
    if (!biwaWaterTex_) biwaWaterTex_ = loadTileTexture("biwako_water_tile");
    envList_ = glGenLists(1);
    glNewList(envList_, GL_COMPILE);
    auto toW = [](double x, double yl) { return glm::dvec3(yl, 0, -x); };
    // 描画順は地表→農地・道路→水面→砂州→滑走路。
    // 水面を地表より先に描くと、後段の負のポリゴンオフセットが水面を覆うため、
    // まず不透明な陸地レイヤーをすべて完成させる。
    // ==== 地面デカール層(角度非依存の重なり) ====
    // 不具合対策の最終形: ポリゴンオフセットは視線の傾きに比例する補正のため、
    // 「見る角度によって透けたり透けなかったり」するZファイトが原理的に残る。
    // 地面の平面群はすべて同一平面上の水平デカールなので、深度を一切使わず
    // 「描画順=重なり順」のペインターズアルゴリズムで描く(角度・距離に完全に非依存)。
    // 最後に地面深度だけを1枚の平面で書き込み、後続の3D物との前後関係を成立させる
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    // 下敷きの薄茶色平面: 全マップ+視程外まで覆い、どこにもカバー漏れの穴が出ないようにする
    glDisable(GL_LIGHTING);
    setColor(0xb7ad96);
    drawGroundQuad(0, -0.3, 0, 34000, 34000);
    glEnable(GL_LIGHTING);
    // 陸: 砂利の河川敷(滑走路周辺)+緑の平野。全方位±15000へ拡張(視程14000の外まで)
    setColor(0x858a6e); drawGroundQuad(0, 0.02, 0, 30000, 30000);       // 河口平野: 灰緑の低彩度草地
    setColor(0x7b8268); drawGroundQuad(-3350, 0.03, -2500, 5000, 12000); // 東の農地
    setColor(0x748263); drawGroundQuad(3500, 0.03, -2500, 4500, 12000);  // 西側の緑地公園・農地
    // 河川敷は衛星写真の乾いた灰茶色を主色にする。濃緑の巨大面を避け、滑走路・
    // 管理路が遠距離でも読み分けられるようにする。
    setColor(0xa49a82); drawGroundQuad(260, 0.045, 430, 560, 2700);
    const double RWY_S = site::FUJI_RWY_SOUTH_Z;
    const double RWY_N = site::FUJI_RWY_NORTH_Z;
    const double RWY_CZ = (RWY_S + RWY_N) / 2;
    const double RWY_LEN2 = site::FUJI_RWY_LENGTH + 20;
    // 干し場・格納庫・管理設備は置かず、滑走路西側を開けた河川敷として残す。
    // 滑走路再設計(850×30m, z=10..860): 中心線から±15mが舗装、その外側±5m(15〜20m)が
    // 薄茶色の肩、さらに外側5〜15m(20〜30m)が芝(草)色の地面。ユーザー指定の縁取り仕様
    setColor(0xb7aa8a);   // 薄茶色の肩(舗装縁+5m)
    drawGroundQuad(-17.5, 0.052, RWY_CZ, 5, RWY_LEN2);
    drawGroundQuad(17.5, 0.052, RWY_CZ, 5, RWY_LEN2);
    setColor(0x89965f);   // 芝(舗装縁5〜15m)。西側(+X)は通常通り10m幅、東側(-X)は
    // 川の近岸(-45)まで延長して水際までの地面を切れ目なく覆う(このあと川面が
    // -45以遠を上塗りするので、境界は描画順で自然に確定する)
    drawGroundQuad(25.0, 0.051, RWY_CZ, 10, RWY_LEN2);
    drawGroundQuad(-35.0, 0.051, RWY_CZ, 30, RWY_LEN2);
    setColor(0x8e8879);   // 近岸の細い護岸・湿った砂利帯
    drawGroundQuad(-42.0, 0.052, RWY_CZ, 6, RWY_LEN2 + 120);

    // 駿河湾と富士川(デカール層の続き: 深度不使用のまま、陸の上に描画順で重ねる)
    glDisable(GL_BLEND);
    glDisable(GL_ALPHA_TEST);
    glDisable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, biwaWaterTex_ ? biwaWaterTex_ : waterTex_);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    // 土砂を含む富士川河口の、青みを抑えた灰緑色。
    glColor4f(0.64f, 0.71f, 0.66f, 1.0f);
    const double TR = 62.5;
    glBegin(GL_TRIANGLES);
    {
        const glm::dvec3 a = toW(site::FUJI_SEA_START_X, -6000), b = toW(site::FUJI_SEA_START_X, 6000),
                         c2 = toW(14000, 6000), d = toW(14000, -6000);
        auto tv = [&](const glm::dvec3& p) {
            glTexCoord2d(p.x / TR, p.z / TR);
            glVertex3d(p.x, 0.085, p.z);
        };
        tv(a); tv(b); tv(c2); tv(a); tv(c2); tv(d);

        // 共有サイト定数から生成し、物理の水域境界と同じ岸線を使う。
        for (double xs = site::FUJI_RIVER_UPSTREAM_X; xs < site::FUJI_SEA_START_X; xs += 350) {
            const double xe = std::min(site::FUJI_SEA_START_X, xs + 350);
            const double n0 = site::FUJI_RIVER_NEAR_BANK, f0 = site::fujikawaFarBank(xs);
            const double n1 = site::FUJI_RIVER_NEAR_BANK, f1 = site::fujikawaFarBank(xe);
            const glm::dvec3 ra = toW(xs, n0), rb = toW(xs, f0),
                             rc = toW(xe, f1), rd = toW(xe, n1);
            tv(ra); tv(rb); tv(rc); tv(ra); tv(rc); tv(rd);
        }
    }
    glEnd();
    glBindTexture(GL_TEXTURE_2D, 0);
    glDisable(GL_TEXTURE_2D);

    // 本流の濁りと流速差。物理上はすべて水域のまま、低コントラストの帯だけを
    // 重ねて単一色の板に見えるのを防ぐ。帯は共有岸線の内側へ収める。
    auto riverBand = [&](double x0, double x1, double frac, double halfFrac, unsigned col) {
        auto point = [&](double x, double f) {
            const double nearBank = site::FUJI_RIVER_NEAR_BANK;
            return toW(x, nearBank + (site::fujikawaFarBank(x) - nearBank) * f);
        };
        const glm::dvec3 a = point(x0, frac - halfFrac), b = point(x0, frac + halfFrac);
        const glm::dvec3 c = point(x1, frac + halfFrac), d = point(x1, frac - halfFrac);
        setColor(col);
        glBegin(GL_TRIANGLES);
        glVertex3d(a.x, 0.091, a.z); glVertex3d(b.x, 0.091, b.z); glVertex3d(c.x, 0.091, c.z);
        glVertex3d(a.x, 0.091, a.z); glVertex3d(c.x, 0.091, c.z); glVertex3d(d.x, 0.091, d.z);
        glEnd();
    };
    riverBand(-3300, 1250, 0.38, 0.055, 0x89978d);
    riverBand(-2200, 1320, 0.70, 0.035, 0x9aa69b);

    // 水面より高い砂州(川面の上に描画順で重ねる。湿った砂利色)
    for (size_t i = 5; i < site::FUJI_SANDBARS.size(); i++) {
        const auto& bar = site::FUJI_SANDBARS[i];
        setColor(0xa49a84);
        drawGroundQuad(bar.ylCenter, 0.105, -bar.courseXCenter, bar.ylWidth, bar.courseLength);
    }
    for (size_t i = 0; i < 5; i++) {
        const auto& bar = site::FUJI_SANDBARS[i];
        setColor(0x77766c);   // 濡れた縁(暗い砂利)
        drawGroundQuad(bar.ylCenter, 0.099, -bar.courseXCenter, bar.ylWidth, bar.courseLength);
        setColor(0xa99e88);   // 乾いた内側
        drawGroundQuad(bar.ylCenter, 0.108, -bar.courseXCenter,
                       bar.ylWidth - 10.0, bar.courseLength - 12.0);
    }
    glEnable(GL_LIGHTING);

    // 滑走路(850m, z=10..860, 幅30m=半幅15m): アスファルトタイル(繰り返し~10m)。
    // 読み込み失敗時は従来のダート単色。白線は従来どおりこの上に重ね描き
    // (デカール層の続き: 深度不使用、描画順で肩・芝の上に重なる)
    if (asphaltTex_) {
        glDisable(GL_LIGHTING);
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, asphaltTex_);
        setColor(0x77756f);   // 実景の黒褐色に退色した舗装へ寄せる
        const double RT = 10.0;
        glBegin(GL_TRIANGLES);
        glNormal3d(0, 1, 0);
        glTexCoord2d(-site::FUJI_RWY_HALF_WIDTH / RT, RWY_S / RT); glVertex3d(-site::FUJI_RWY_HALF_WIDTH, 0.06, RWY_S);
        glTexCoord2d(-site::FUJI_RWY_HALF_WIDTH / RT, RWY_N / RT); glVertex3d(-site::FUJI_RWY_HALF_WIDTH, 0.06, RWY_N);
        glTexCoord2d(site::FUJI_RWY_HALF_WIDTH / RT, RWY_N / RT);  glVertex3d(site::FUJI_RWY_HALF_WIDTH, 0.06, RWY_N);
        glTexCoord2d(-site::FUJI_RWY_HALF_WIDTH / RT, RWY_S / RT); glVertex3d(-site::FUJI_RWY_HALF_WIDTH, 0.06, RWY_S);
        glTexCoord2d(site::FUJI_RWY_HALF_WIDTH / RT, RWY_N / RT);  glVertex3d(site::FUJI_RWY_HALF_WIDTH, 0.06, RWY_N);
        glTexCoord2d(site::FUJI_RWY_HALF_WIDTH / RT, RWY_S / RT);  glVertex3d(site::FUJI_RWY_HALF_WIDTH, 0.06, RWY_S);
        glEnd();
        glBindTexture(GL_TEXTURE_2D, 0);
        glDisable(GL_TEXTURE_2D);
        glEnable(GL_LIGHTING);
    } else {
        setColor(0x625f59); drawGroundQuad(0, 0.06, RWY_CZ,
                                          site::FUJI_RWY_HALF_WIDTH * 2, site::FUJI_RWY_LENGTH);
    }
    glDisable(GL_LIGHTING);
    setColor(0xd7d6cf);
    for (double z = RWY_S + 24; z < RWY_N - 20; z += 48) drawGroundQuad(0, 0.075, z, 0.34, 7);   // 細く退色したセンターライン
    drawGroundQuad(0, 0.075, RWY_S + 4, 25, 1.3);
    drawGroundQuad(0, 0.075, RWY_N - 4, 25, 1.3);
    // 着陸目安線(touchdown/aiming point marking): 各端から80m内側、中心線を挟む2本の白帯
    auto drawAimMarks = [&](double cz) {
        drawGroundQuad(-4.5, 0.076, cz, 2.4, 8.0);
        drawGroundQuad(4.5, 0.076, cz, 2.4, 8.0);
    };
    drawAimMarks(RWY_S + 80);
    drawAimMarks(RWY_N - 80);
    // 滑走路番号。7セグ形状で遠距離でも判読できる白標示にする。
    // 読み手(その滑走路端から離陸するパイロット)の視点で正しく読めるよう、
    // 字形の上方向(sz)と左右方向(sx)を読み手の向きに合わせて反転する。
    // 世界座標: +X=西, +Z=北。+Z(北)を向く読み手には 上=+Z・右手=東=-X → sx=-1, sz=+1。
    // -Z(南)を向く読み手には 上=-Z・右手=西=+X → sx=+1, sz=-1。
    // (旧実装は sx=+1 固定で、南端の数字が左右鏡像になっていた)
    auto drawRwyDigit = [&](int digit, double cx, double cz, double sx, double sz) {
        const bool seg[10][7] = {
            {1,1,1,1,1,1,0}, {0,1,1,0,0,0,0}, {1,1,0,1,1,0,1}, {1,1,1,1,0,0,1},
            {0,1,1,0,0,1,1}, {1,0,1,1,0,1,1}, {1,0,1,1,1,1,1}, {1,1,1,0,0,0,0},
            {1,1,1,1,1,1,1}, {1,1,1,1,0,1,1}
        };
        const double y = 0.078, hw = 2.0, vh = 2.2, t = 0.48;
        if (seg[digit][0]) drawGroundQuad(cx, y, cz + 4.5 * sz, hw * 2, t);
        if (seg[digit][1]) drawGroundQuad(cx + hw * sx, y, cz + 2.25 * sz, t, vh * 2);
        if (seg[digit][2]) drawGroundQuad(cx + hw * sx, y, cz - 2.25 * sz, t, vh * 2);
        if (seg[digit][3]) drawGroundQuad(cx, y, cz - 4.5 * sz, hw * 2, t);
        if (seg[digit][4]) drawGroundQuad(cx - hw * sx, y, cz - 2.25 * sz, t, vh * 2);
        if (seg[digit][5]) drawGroundQuad(cx - hw * sx, y, cz + 2.25 * sz, t, vh * 2);
        if (seg[digit][6]) drawGroundQuad(cx, y, cz, hw * 2, t);
    };
    // 南端=RWY36: 読み手は北(+Z)向き。左手=西=+X なので「3」を+X側に(左から36と読める)
    drawRwyDigit(3, 5.0, RWY_S + 38.0, -1, 1);
    drawRwyDigit(6, -5.0, RWY_S + 38.0, -1, 1);
    // 北端=RWY18: 読み手は南(-Z)向き。左手=東=-X なので「1」を-X側に
    drawRwyDigit(1, -5.0, RWY_N - 38.0, 1, -1);
    drawRwyDigit(8, 5.0, RWY_N - 38.0, 1, -1);

    // 国道1号・生活道路・スポーツ広場などの平面デカール(3D物より前=この層内で描く。
    // 3D物の後に深度なしで描くと樹木の根元などを上塗りしてしまうため)
    {
        setColor(0x8e8a7d);
        const double rd1[][2] = {{380, 60}, {370, 260}, {330, 460}, {260, 640},
                                  {150, 800}, {0, RWY_N + 415}};
        for (size_t i = 0; i + 1 < 6; i++) {
            const double xa = rd1[i][0], za = rd1[i][1], xb = rd1[i + 1][0], zb = rd1[i + 1][1];
            const double mx = (xa + xb) / 2, mz = (za + zb) / 2;
            const double len = std::hypot(xb - xa, zb - za), ang = std::atan2(xb - xa, zb - za) * 180 / PI;
            glPushMatrix();
            glTranslated(mx, 0.055, mz);
            glRotated(ang, 0, 1, 0);
            drawGroundQuad(0, 0, 0, 8, len + 2);
            glPopMatrix();
        }
        setColor(0x918c80);
        drawGroundQuad(230, 0.055, 620, 5, 260);   // 生活道路のスパー
        // スポーツ広場(緑地パッチ)
        setColor(0x7f9162);
        drawGroundQuad(300, 0.053, 260, 90, 65);
        setColor(0x899a6b);
        drawGroundQuad(300, 0.0535, 260, 60, 40);   // 内側の芝目
        setColor(0x7f9162);
        drawGroundQuad(340, 0.053, 560, 100, 75);
        setColor(0xa79d84);
        drawGroundQuad(340, 0.0535, 560, 55, 30);   // ダート内野
        setColor(0x7f9162);
        drawGroundQuad(280, 0.053, 900, 80, 70);
        // 北東側(川向こう)のソフトボール場・自由広場
        setColor(0x72865e);
        drawGroundQuad(-330, 0.053, RWY_N + 380, 90, 70);
        setColor(0x7b8d63);
        drawGroundQuad(-260, 0.053, RWY_N + 560, 80, 60);
    }
    glEnable(GL_LIGHTING);

    // ==== 地面デカール層おわり ====
    // 地面の「深度」だけを1枚の平面で書き込む(色は書かない)。単一平面なので
    // 自己Zファイトは起きず、以降の3D物(山・橋・機材など)が地面より下に潜る部分を
    // 正しく隠しつつ、地面上の物は全角度で安定して手前に表示される
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glDisable(GL_LIGHTING);
    drawGroundQuad(0, 0.0, 0, 34000, 34000);
    glEnable(GL_LIGHTING);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    // 山: 西(yl>1500, ワールド+X)は天守山地=高い、東(-X)は低い丘
    // 第5弾: drawMountainRidge化(標高グラデーション+肩コーン)+麓に針葉樹クラスタ
    for (int i = 0; i < 26; i++) {
        const double t = i / 25.0;
        const double zx = -(-8000 + t * 16000);   // コースx -8000..8000 → ワールドz
        const double wv = 300 + rnd(rng) * 350;
        const double wx = 2600 + rnd(rng) * 1800, wz = zx + (rnd(rng) - 0.5) * 600;
        drawMountainRidge(rng, {wx, -4, wz}, wv, 500 + rnd(rng) * 500,
                          i % 2 ? 0x61715f : 0x566a59);
        const double ex = -3000 - rnd(rng) * 2200, ez = zx + (rnd(rng) - 0.5) * 600;
        drawMountainRidge(rng, {ex, -4, ez}, wv * 0.9, 180 + rnd(rng) * 220,
                          i % 2 ? 0x718371 : 0x667866);
    }
    // 富士山(北東=コースx≈-14000, yl≈-11000 → ワールド X=-11000, Z=+14000)
    // 第5弾美化: 中腹の肩コーン+段階的な冠雪(裾野=青灰→中腹→雪線)
    setColor(0x89979f);
    drawCone(-11000, -50, 14000, 5600, 3300, 10);
    setColor(0x98a4aa);
    drawCone(-9200, -50, 14600, 1500, 700, 8);    // 宝永山側の肩
    drawCone(-12600, -50, 14400, 1300, 560, 8);   // 西側の肩
    setColor(0xf4f7fa);
    drawCone(-11000, 2150, 14000, 1900, 1100, 10);   // 冠雪
    setColor(0xe6ecf2);
    drawCone(-11000, 1750, 14000, 2350, 700, 10);    // 雪線の裾(淡いグラデーション)
    // 雲(2層: 低層=大きく濃い/高層=小さく淡い)
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    setColor(0xffffff, 0.5f);
    for (int i = 0; i < 12; i++)
        drawSphere({-900 + rnd(rng) * 1800, 80 + rnd(rng) * 60, -300 - rnd(rng) * 1800},
                   11 + rnd(rng) * 14, 6, 5, 0.4);
    setColor(0xffffff, 0.25f);
    for (int i = 0; i < 8; i++)
        drawSphere({-1000 + rnd(rng) * 2000, 250 + rnd(rng) * 140, -400 - rnd(rng) * 2200},
                   6 + rnd(rng) * 9, 6, 5, 0.4);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glEndList();
}

void Renderer3D::drawLakeAndGrid(const glm::dvec3& camPos) {
    // 水面本体は湖岸ポリゴンとしてenvList_に静的描画済み。
    // ここではスケールグリッドのみ(カメラ付近に吸着)。
    // 浅い視線角で線が重なって灰色の膜に見えないよう、範囲を狭く・薄くする
    const double cx = camPos.x, cz = camPos.z;
    glDisable(GL_LIGHTING);
    setColor(0x9fc8e0, 0.30f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    const double gx = std::round(cx / 20) * 20, gz = std::round(cz / 20) * 20;
    glBegin(GL_LINES);
    for (int i = -60; i <= 60; i++) {
        glVertex3d(gx + i * 10, 0, gz - 600); glVertex3d(gx + i * 10, 0, gz + 600);
        glVertex3d(gx - 600, 0, gz + i * 10); glVertex3d(gx + 600, 0, gz + i * 10);
    }
    glEnd();
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
}

// 翼型プロファイル(コード比f位置)の上面/下面高さをコード比で返す
static double afYAt(const std::vector<std::pair<double, double>>& v, double f) {
    if (f <= v.front().first) return v.front().second;
    for (size_t k = 1; k < v.size(); k++)
        if (f <= v[k].first) {
            const double u = (f - v[k - 1].first) / std::max(1e-9, v[k].first - v[k - 1].first);
            return lerp(v[k - 1].second, v[k].second, u);
        }
    return v.back().second;
}

// ---------- 機体 ----------
void Renderer3D::buildAircraft(const AircraftParams& st, const Analysis& an,
                               const AirframeGraph& graph) {
    if (acOpaque_) { glDeleteLists(acOpaque_, 1); glDeleteLists(acFilm_, 1); glDeleteLists(propList_, 1); }
    acOpaque_ = acFilm_ = propList_ = 0;
    const Profile& AF = mainAirfoil();
    const auto layout = graph.resolve();
    auto placement = [&](const char* id) -> const AirframeGraph::Placed* {
        for (const auto& placed : layout)
            if (!placed.mirrored && placed.part->id == id) return &placed;
        return nullptr;
    };
    const Part* root = graph.find("fuselage");
    const auto* wingPlaced = placement("wing.main");
    const auto* htailPlaced = placement("tail.h");
    const auto* vtailPlaced = placement("tail.v");
    const auto* propPlaced = placement("prop.main");
    const auto* cockpitPlaced = placement("cockpit");
    const auto* pilotPlaced = placement("pilot");
    if (!root || !wingPlaced || !htailPlaced || !vtailPlaced || !propPlaced || !cockpitPlaced)
        return;
    auto hardpointPos = [&](const char* id) {
        for (const auto& hp : root->hardpoints)
            if (hp.id == id) return glm::dvec3(transformMatrix(hp.t)[3]);
        return glm::dvec3(0.0);
    };
    auto correctionFrom = [](const glm::dmat4& world, const glm::dvec3& generatedOrigin) {
        return world * glm::translate(glm::dmat4(1.0), -generatedOrigin);
    };
    const glm::dvec3 wingPos(wingPlaced->world[3]);
    const glm::dvec3 htailPos(htailPlaced->world[3]);
    const glm::dvec3 propPos(propPlaced->world[3]);
    const glm::dvec3 cockpitPos(cockpitPlaced->world[3]);
    const glm::dmat4 htailCorrection = correctionFrom(htailPlaced->world, {0.0, htailPos.y, htailPos.z});
    const glm::dmat4 pilotCorrection = pilotPlaced
        ? correctionFrom(pilotPlaced->world, {0.0, 0.0, an.pilotCGx}) : glm::dmat4(1.0);
    const auto* fairingPlaced = placement("fairing");
    const glm::dmat4 fairingCorrection = fairingPlaced
        ? correctionFrom(fairingPlaced->world, {0.0, 0.0, st.seatX}) : glm::dmat4(1.0);
    const double hWing = wingPos.y;
    const double hBoom = hardpointPos("hp.tail.v").y;
    const double half = st.span / 2;
    const double dihT = std::tan((st.jig == "flat" ? 0 : st.dihedral) * PI / 180);
    const double seat = cockpitPos.z;
    const SparDesign sparSpec = wingPlaced->part->design.spar.value_or(
        SparDesign{1, 0.30, st.rootDia, st.tipDia, std::max(0, st.segments - 1), "tube"});
    wingH_ = wingPos.y;
    wingZ_ = wingPos.z;
    wingDrawTransform_ = correctionFrom(wingPlaced->world, {0.0, wingPos.y, wingPos.z});
    propWorld_ = propPlaced->world;
    propH_ = propPos.y;
    propZ_ = propPos.z;

    // ---- 主翼: たわみ変形メッシュとして構築 (JS applyFlex相当の頂点変形) ----
    flexBasis_ = computeFlexBasis(st, an);
    flexKey_ = -1;
    wingOpq_.clear();
    wingFilm_.clear();
    {
        MeshEmit em;
        em.fb = &flexBasis_;
        em.m = &wingOpq_;
        for (int side : {1, -1}) {
            auto stn = wingStations(st, side);
            for (auto& s : stn) { s.y0 += hWing; s.z0 += wingPos.z; }
            // プランク(上下前縁+後縁)
            em.color(0xe7d9b8);
            em.loftSkin(AF, stn, 0, st.plankTop / 100, true);
            em.loftSkin(AF, stn, 0, st.plankBot / 100, false);
            if (st.plankRearTop > 0) em.loftSkin(AF, stn, 1 - st.plankRearTop / 100, 1, true);
            if (st.plankRearBot > 0) em.loftSkin(AF, stn, 1 - st.plankRearBot / 100, 1, false);
            // リブ(扇形分割)。翼表面はステーション間の線形補間のため、弦長分布が
            // 曲線的な平面形(楕円/三日月)ではリブが外皮を貫通しうる → 少し内側へインセット
            em.color(0x2c5f9e);
            for (double y = st.ribPitch; y < half * 0.985; y += st.ribPitch) {
                const double t = y / half, ch = chordAt(st, t);
                const double bx = side * y, by = hWing + dihT * y, bz = wingPos.z;
                std::vector<std::pair<double, double>> poly;
                for (auto it = AF.up.rbegin(); it != AF.up.rend(); ++it) poly.push_back(*it);
                for (const auto& p : AF.lo) poly.push_back(p);
                auto rp = [&](size_t j) {
                    return glm::dvec3(bx, by + poly[j].second * 0.95 * ch,
                                      bz + (0.006 + poly[j].first * 0.988) * ch);
                };
                for (size_t j = 1; j + 1 < poly.size(); j++)
                    em.tri(rp(0), rp(j), rp(j + 1));
            }
            // 主桁: 本数・翼弦位置・断面・テーパー・継手をPart固有仕様から描画。
            const int sparCount = std::max(1, sparSpec.count);
            for (int sparIndex = 0; sparIndex < sparCount; ++sparIndex) {
                const double chordFrac = clamp(sparSpec.chordFrac
                    + (sparIndex - (sparCount - 1) * 0.5) * 0.11, 0.10, 0.80);
                const double upF = afYAt(AF.up, chordFrac), loF = afYAt(AF.lo, chordFrac);
                auto sparAt = [&](double y, double verticalOffset = 0.0) {
                    const double ch = chordAt(st, y / half);
                    return glm::dvec3(side * y,
                                      hWing + dihT * y + (0.5 * (upF + loF) + verticalOffset) * ch,
                                      wingPos.z + chordFrac * ch);
                };
                auto sparR = [&](double y) {
                    const double ch = chordAt(st, y / half);
                    const double rmax = 0.46 * (upF - loF) * ch;
                    return clamp(lerp(sparSpec.rootDiaMm, sparSpec.tipDiaMm, y / half) / 2000,
                                 0.006, rmax);
                };
                em.color(sparSpec.section == "tube" ? 0x23282e
                         : sparSpec.section == "box" ? 0x50483f : 0x33465a);
                const int NS = 12;
                for (int i = 0; i < NS; i++) {
                    const double y0 = half * i / NS, y1 = half * (i + 1) / NS;
                    if (sparSpec.section == "i-beam") {
                        const double o = 0.32 * (upF - loF);
                        em.strut(sparAt(y0), sparAt(y1), sparR(y0) * 0.38, sparR(y1) * 0.38, 4);
                        em.strut(sparAt(y0, o), sparAt(y1, o), sparR(y0) * 0.32, sparR(y1) * 0.32, 4);
                        em.strut(sparAt(y0, -o), sparAt(y1, -o), sparR(y0) * 0.32, sparR(y1) * 0.32, 4);
                    } else {
                        em.strut(sparAt(y0), sparAt(y1), sparR(y0), sparR(y1),
                                 sparSpec.section == "box" ? 4 : 10);
                    }
                }
                em.color(0xe8590c);
                for (int joint = 1; joint <= sparSpec.jointCount; ++joint) {
                    const double jy = half * joint / (sparSpec.jointCount + 1.0);
                    const double ch = chordAt(st, jy / half);
                    const double rj = std::min(sparR(jy) * 1.25, 0.48 * (upF - loF) * ch);
                    em.strut(sparAt(std::max(0.0, jy - 0.1)), sparAt(std::min(half, jy + 0.1)),
                             rj, rj, 12);
                }
            }
            // フィルム+エルロン(半透明メッシュ)
            em.m = &wingFilm_;
            em.color(0xeef4fa, 0.32f);
            em.loftWing(AF, stn, 0, 1, true);
            if (st.ailMode != "none") {
                em.color(0xe8590c, 0.6f);
                const double yIn = half * (1 - st.ailSpanFrac);
                std::vector<Station> sub;
                for (int i = 0; i <= 8; i++) {
                    const double y = lerp(yIn, half * 0.98, i / 8.0), t = y / half;
                    sub.push_back({(double)side * y, chordAt(st, t), hWing + dihT * y, wingPos.z});
                }
                em.loftWing(AF, sub, 1 - st.ailChordFrac, 1, false);
            }
            em.m = &wingOpq_;
        }
    }
    wingOpq_.pos = wingOpq_.base;
    wingFilm_.pos = wingFilm_.base;
    applyFlex(0, 1);   // 初期は駐機(自重たれ)

    // ---- 不透明部(剛体・ディスプレイリスト) ----
    acOpaque_ = glGenLists(1);
    glNewList(acOpaque_, GL_COMPILE);
    // コックピット枠+テールビーム (JSのトラス)
    {
        setColor(0x1b2a4a);
        const glm::dvec3 FT = hardpointPos("hp.frame.front.top");
        const glm::dvec3 FB = hardpointPos("hp.frame.front.bottom");
        const glm::dvec3 RT = hardpointPos("hp.frame.rear.top");
        const glm::dvec3 RB = hardpointPos("hp.frame.rear.bottom");
        drawStrut(FB, FT, 0.026, 0.026);
        drawStrut(RB, RT, 0.026, 0.026);
        drawStrut(FB, RB, 0.022, 0.022);
        drawStrut(FB, RT, 0.020, 0.020);
        const glm::dvec3 tailEnd = hardpointPos("hp.tail.end");
        drawStrut(FT, tailEnd, 0.05, 0.026);
        // 桁ルート(翼側の主桁中心と一致させる: 局所コード30%・翼型中心線)
        const Profile& AFr = mainAirfoil();
        const double upR = afYAt(AFr.up, 0.30), loR = afYAt(AFr.lo, 0.30);
        const glm::dvec3 generatedSparRoot(0, hWing + 0.5 * (upR + loR) * st.rootChord,
                                           wingPos.z + 0.30 * st.rootChord);
        const glm::dvec3 sparRoot(wingDrawTransform_ * glm::dvec4(generatedSparRoot, 1.0));
        drawStrut(FT, sparRoot, 0.032, 0.032);
        drawStrut(RT, sparRoot, 0.024, 0.024);
        setColor(0x23282e);
        for (const auto& nd : {FT, RT, sparRoot}) drawSphere(nd, 0.05, 10, 8);
        // 機首ブーム(牽引式)
        if (st.propConfig == "tractor") {
            setColor(0x1b2a4a);
            drawStrut(FT, propPos, 0.04, 0.028);
        }
        if (st.propConfig == "pylon") {
            setColor(0x1b2a4a);
            drawStrut({propPos.x, hBoom, propPos.z}, propPos, 0.04, 0.03);
        }
    }
    // パイロット(体幹ポリライン。フェアリングの包含サイズ算出にも使う)
    std::vector<glm::dvec3> pilotBody;
    PilotStationDesign defaultPilot;
    if (st.posture == "upright") defaultPilot = {0.68, -0.30, 0.80, -0.28, 0.42};
    else if (st.posture == "semi") defaultPilot = {0.62, -0.50, 0.85, -0.80, 0.58};
    else defaultPilot = {0.60, -0.55, 0.92, -0.95, 0.68};
    const PilotStationDesign pilotSpec = pilotPlaced
        ? pilotPlaced->part->design.pilot.value_or(defaultPilot) : defaultPilot;
    auto Pv = [&](double z, double y) { return glm::dvec3(0, y, z); };
    if (st.posture == "upright")
        pilotBody = {Pv(seat+0.02,1.55), Pv(seat+0.02,1.30), Pv(seat,0.68), Pv(seat-0.30,0.80), Pv(seat-0.28,0.42)};
    else if (st.posture == "semi")
        pilotBody = {Pv(seat+0.35,1.18), Pv(seat+0.18,1.02), Pv(seat,0.62), Pv(seat-0.50,0.85), Pv(seat-0.80,0.58)};
    else
        pilotBody = {Pv(seat+0.50,0.86), Pv(seat+0.30,0.78), Pv(seat,0.60), Pv(seat-0.55,0.92), Pv(seat-0.95,0.68)};
    pilotBody[2].y = pilotSpec.seatHeightM;
    pilotBody[3] = Pv(seat + pilotSpec.pedalZM, pilotSpec.pedalHeightM);
    pilotBody[4] = Pv(seat + pilotSpec.crankZM, pilotSpec.crankHeightM);
    if (pilotPlaced) {
        glPushMatrix();
        glMultMatrixd(glm::value_ptr(pilotCorrection));
        setColor(0x2c5f9e);
        auto& body = pilotBody;
        for (size_t i = 0; i + 1 < body.size(); i++) {
            drawStrut(body[i], body[i + 1], 0.075, 0.075, 8);
            drawSphere(body[i], 0.075, 8, 6);
        }
        drawSphere(body[0] + glm::dvec3(0, 0.13, 0.03), 0.11, 12, 10);   // 頭
        // クランク
        setColor(0x23282e);
        drawSphere(body[4], 0.06, 8, 6);
        drawStrut(body[4] + glm::dvec3(-0.16, 0, 0), body[4] + glm::dvec3(0.16, 0, 0), 0.02, 0.02, 6);
        glPopMatrix();
        // 駆動系: クランク→プロペラ軸のチェーン/シャフトライン
        glm::dvec3 crankP(pilotCorrection * glm::dvec4(body[4], 1.0)), propP = propPos;
        glm::dvec3 midP(0, clamp(crankP.y, 0.9, propH_), (crankP.z + propP.z) / 2);
        setColor(0x33383f);
        drawStrut(crankP, midP, st.drive == "shaft" ? 0.02 : 0.011, st.drive == "shaft" ? 0.02 : 0.011, 6);
        drawStrut(midP, propP, st.drive == "shaft" ? 0.02 : 0.011, st.drive == "shaft" ? 0.02 : 0.011, 6);
    }
    // 着陸装置
    {
        auto wheel = [&](const glm::dvec3& c, double rad) {
            setColor(0x23282e);
            // タイヤ(横向き円盤)
            drawStrut(c + glm::dvec3(-0.02, 0, 0), c + glm::dvec3(0.02, 0, 0), rad, rad, 14);
            setColor(0x1b2a4a);
            drawStrut(c, c + glm::dvec3(0, 0.55, 0), 0.018, 0.018, 6);
        };
        for (const auto& placed : layout) {
            if (placed.part->kind != PartKind::Gear) continue;
            double radius = 0.15;
            if (placed.part->id == "gear.front") radius = st.gear == "tandem" ? 0.14 : 0.13;
            else if (placed.part->id == "gear.main" && st.gear == "mono") radius = 0.16;
            else if (placed.part->id == "gear.tail") radius = 0.07;
            glPushMatrix();
            glMultMatrixd(glm::value_ptr(placed.world));
            wheel({0.0, 0.0, 0.0}, radius);
            glPopMatrix();
        }
    }
    // 水平尾翼(薄い対称翼型ロフト)
    {
        glPushMatrix();
        glMultMatrixd(glm::value_ptr(htailCorrection));
        const Profile& TH = thinAirfoil();
        const double hHalf = st.hSpan / 2, hLE = htailPos.z - st.hChord * 0.4;
        const int N = 12;
        std::vector<Station> stn;
        for (int i = 0; i <= N; i++) {
            double u = (double)i / N * 2 - 1, ax = std::abs(u);
            double le = hLE, ch = st.hChord;
            if (st.hShape == "taper") { le = hLE + st.hChord * 0.25 * ax; ch = st.hChord * (1 - 0.4 * ax); }
            else if (st.hShape == "ellipse") { double w = std::sqrt(std::max(0.05, 1 - ax * ax)); le = hLE + st.hChord * (1 - w) / 2; ch = st.hChord * w; }
            else if (st.hShape == "swept") { le = hLE + st.hChord * 0.38 * ax; ch = st.hChord * (1 - 0.3 * ax); }
            else if (st.hShape == "delta") { le = hLE + st.hChord * 0.55 * ax; ch = st.hChord * (1 - 0.65 * ax); }
            stn.push_back({u * hHalf, ch, htailPos.y, le});
        }
        setColor(0xe8590c);   // 全可動(制御面色)
        loftWing(TH, stn, st.elevRatio < 1 ? 1 - st.elevRatio : 0.0, 1.0, false);
        if (st.elevRatio < 1) { setColor(0xf4f8fc); loftWing(TH, stn, 0.0, 1 - st.elevRatio, false); }
        glPopMatrix();
    }
    // 垂直尾翼(平板ポリゴン)。Pairを含む全配置を同じgraphから描く。
    {
        const double vc = st.vChord, vh = st.vHeight;
        std::vector<std::pair<double,double>> sh;   // (z, y) 機体ローカル
        if (st.vShape == "swept") sh = {{-vc*0.2,0},{vc*0.55,vh},{vc*1.05,vh},{vc*0.8,0}};
        else if (st.vShape == "ellipse") sh = {{0,0},{vc*0.05,vh*0.6},{vc*0.5,vh},{vc*0.95,vh*0.6},{vc,0}};
        else if (st.vShape == "delta") sh = {{0,0},{vc*0.4,vh},{vc*0.65,vh},{vc,0}};
        else if (st.vShape == "dorsal") sh = {{-vc*0.1,0},{vc*0.25,vh*0.55},{vc*0.45,vh},{vc*0.7,vh},{vc*0.9,vh*0.4},{vc*0.85,0}};
        else sh = {{0,0},{0,vh},{vc,vh},{vc,0}};
        auto drawFin = [&] {
            setColor(st.rudRatio == 1.0 ? 0xe8590c : 0xf4f8fc);
            glBegin(GL_TRIANGLES);
            glNormal3d(1, 0, 0);
            for (size_t j = 1; j + 1 < sh.size(); j++) {
                glVertex3d(0.015, sh[0].second, sh[0].first);
                glVertex3d(0.015, sh[j].second, sh[j].first);
                glVertex3d(0.015, sh[j + 1].second, sh[j + 1].first);
            }
            glEnd();
            if (st.rudRatio < 1) {   // ラダー部
                const double rx = vc * (1 - st.rudRatio);
                setColor(0xe8590c);
                glBegin(GL_TRIANGLES);
                glNormal3d(1, 0, 0);
                glm::dvec2 q0(rx, 0.03), q1(rx + vc * 0.22, vh * 0.95),
                           q2(vc * (st.vShape == "swept" ? 1.0 : 0.98), vh * 0.95),
                           q3(vc * (st.vShape == "swept" ? 0.79 : 0.99), 0.03);
                glVertex3d(0.018, q0.y, q0.x); glVertex3d(0.018, q1.y, q1.x); glVertex3d(0.018, q2.y, q2.x);
                glVertex3d(0.018, q0.y, q0.x); glVertex3d(0.018, q2.y, q2.x); glVertex3d(0.018, q3.y, q3.x);
                glEnd();
            }
        };
        for (const auto& placed : layout) {
            if (placed.part->id != "tail.v") continue;
            glPushMatrix();
            glMultMatrixd(glm::value_ptr(placed.world));
            drawFin();
            glPopMatrix();
        }
    }
    // テールビーム翼
    // 尾翼支持材: Part固有の取付方式・本数・径を各配置インスタンスへ描画する。
    for (const auto& placed : layout) {
        if (placed.part->id != "tail.h" && placed.part->id != "tail.v") continue;
        if (!placed.part->design.tailSupport) continue;
        const TailSupportDesign& support = *placed.part->design.tailSupport;
        if (support.mounting == "cantilever" || support.supportCount <= 0) continue;
        const double radius = support.mounting == "wire"
            ? std::max(0.0015, support.supportDiaMm / 2000.0)
            : support.supportDiaMm / 2000.0;
        setColor(support.mounting == "wire" ? 0x707984 : 0x33465a);
        glPushMatrix();
        glMultMatrixd(glm::value_ptr(placed.world));
        for (int i = 0; i < support.supportCount; ++i) {
            const int side = i % 2 ? -1 : 1;
            const double lane = 0.35 + 0.5 * ((i / 2 + 1.0) / (support.supportCount / 2.0 + 1.0));
            if (placed.part->id == "tail.h") {
                const glm::dvec3 tip{side * st.hSpan * 0.5 * lane, 0.0, st.hChord * 0.15};
                const glm::dvec3 base{0.0, -0.45, st.hChord * 0.05};
                drawStrut(base, tip, radius, radius, support.mounting == "wire" ? 5 : 8);
            } else {
                const glm::dvec3 tip{0.0, st.vHeight * lane, st.vChord * 0.4};
                const glm::dvec3 base{side * 0.32, 0.0, st.vChord * 0.12};
                drawStrut(base, tip, radius, radius, support.mounting == "wire" ? 5 : 8);
            }
        }
        glPopMatrix();
    }

    {
        setColor(0xc8d8e8);
        auto mkBW = [&](int side, const glm::dvec3& origin) {
            std::vector<Station> stn;
            for (int i = 0; i <= 8; i++) {
                double t = i / 8.0, y = t * st.boomWingSpan / 2;
                stn.push_back({origin.x + side * y, st.boomWingChord * (1 - 0.35 * t), origin.y, origin.z});
            }
            loftWing(thinAirfoil(), stn, 0, 1, true);
        };
        for (const auto& placed : layout) {
            if (placed.part->id != "boomwing") continue;
            const int side = st.boomWing == "R" ? -1 : (placed.mirrored ? -1 : 1);
            glPushMatrix();
            glMultMatrixd(glm::value_ptr(placed.world));
            mkBW(side, {0.0, 0.0, 0.0});
            glPopMatrix();
        }
    }
    glEndList();

    // ---- 半透明部(フェアリングのみ。翼フィルムはwingFilm_メッシュ側) ----
    acFilm_ = glGenLists(1);
    glNewList(acFilm_, GL_COMPILE);
    if (fairingPlaced) {
        glPushMatrix();
        glMultMatrixd(glm::value_ptr(fairingCorrection));
        // 流線型の涙滴フェアリング: パイロット体幹ポリラインから包含要件を採取し、
        // 「先端は丸く(楕円ノーズ)→最大断面(胸〜腰)→後方へ滑らかに絞る(cosテーパー)」
        // の断面プロファイルでロフトする。サイズは姿勢ごとの体格を包含するよう自動決定。
        // 1) パイロットのサンプル点(z, yMin, yMax, x半幅)を採取
        struct Samp { double z, yLo, yHi, xh; };
        std::vector<Samp> samps;
        const double bodyR = 0.075;
        for (size_t i = 0; i + 1 < pilotBody.size(); i++)
            for (int k = 0; k <= 4; k++) {
                const double t = k / 4.0;
                glm::dvec3 p = pilotBody[i] + (pilotBody[i + 1] - pilotBody[i]) * t;
                samps.push_back({p.z, p.y - bodyR, p.y + bodyR, bodyR});
            }
        {   // 頭(球) と クランク(横に張り出す)
            glm::dvec3 hd = pilotBody[0] + glm::dvec3(0, 0.13, 0.03);
            samps.push_back({hd.z, hd.y - 0.11, hd.y + 0.11, 0.11});
            const glm::dvec3& ck = pilotBody[4];
            samps.push_back({ck.z, ck.y - 0.06, ck.y + 0.06, 0.18});
        }
        const FairingDesign fairingSpec = fairingPlaced->part->design.fairing.value_or(FairingDesign{});
        // 2) 前後範囲。ユーザー指定全長を人体中心の周囲へ配置する。
        double zMin = 1e9, zMax = -1e9;
        for (const auto& s : samps) { zMin = std::min(zMin, s.z); zMax = std::max(zMax, s.z); }
        const double zMid = 0.5 * (zMin + zMax);
        const double zLen = fairingSpec.lengthM;
        const double zF = zMid - zLen * 0.5;
        // 3) 断面プロファイル f(t): t<tm=楕円ノーズ、t>=tm=cosテーパー(最大断面はtm)
        const double tm = fairingSpec.noseRatio;
        auto prof = [&](double t) {
            t = clamp(t, 0.0, 1.0);
            const double f = t < tm ? std::sqrt(std::max(0.0, 1 - ((tm - t) / tm) * ((tm - t) / tm)))
                                    : std::pow(std::cos((t - tm) / (1 - tm) * PI / 2),
                                               0.45 + 0.8 * fairingSpec.tailRatio);
            return std::max(f, 0.02);
        };
        // 4) 中心線 yCen(t): サンプル中点の最小二乗直線(リカンベントの寝姿勢に追従)
        double sw = 0, swt = 0, swy = 0, swtt = 0, swty = 0;
        for (const auto& s : samps) {
            const double t = (s.z - zF) / zLen, ym = (s.yLo + s.yHi) / 2;
            sw += 1; swt += t; swy += ym; swtt += t * t; swty += t * ym;
        }
        const double det = sw * swtt - swt * swt;
        const double cB = det != 0 ? (sw * swty - swt * swy) / det : 0;   // 傾き
        const double cA = (swy - cB * swt) / sw;                          // 切片
        auto yCen = [&](double t) { return cA + cB * t; };
        // 5) 包含に必要な最大半幅/半高(サンプル点を必ず内包するよう構成的に決定)
        const double Wmax = fairingSpec.widthM * 0.5;
        const double Hmax = fairingSpec.heightM * 0.5;
        // 6) 検証: 全サンプル点が断面楕円に収まるか(構成上収まるはずだが数値保険)
        for (const auto& s : samps) {
            const double t = (s.z - zF) / zLen, f = prof(t);
            if (s.xh > Wmax * f + 1e-9 || s.yHi > yCen(t) + Hmax * f + 1e-9 ||
                s.yLo < yCen(t) - Hmax * f - 1e-9)
                std::fprintf(stderr, "warning: fairing containment violated at z=%.2f\n", s.z);
        }
        // 7) ロフト描画(断面12個×周16分割の帯)
        setColor(0xd0e4f0, 0.38f);
        const int NSEC = 12, NRAD = 16;
        auto ringPt = [&](int i, int j) {
            const double t = (double)i / NSEC, f = prof(t);
            const double a = 2 * PI * j / NRAD;
            return glm::dvec3(Wmax * f * std::cos(a), yCen(t) + Hmax * f * std::sin(a),
                              zF + t * zLen);
        };
        glBegin(GL_TRIANGLES);
        for (int i = 0; i < NSEC; i++)
            for (int j = 0; j < NRAD; j++)
                quad(ringPt(i, j), ringPt(i, j + 1), ringPt(i + 1, j + 1), ringPt(i + 1, j));
        glEnd();
        glPopMatrix();
    }
    glEndList();

    // ---- プロペラ(回転部のみ・原点=ハブ) ----
    propList_ = glGenLists(1);
    glNewList(propList_, GL_COMPILE);
    setColor(0x23282e);
    drawSphere({0, 0, 0}, 0.07, 10, 8);
    if (st.propMat == "carbon") setColor(0x2f3338); else setColor(0xcdb280);
    const int NB = std::max(1, st.propBlades);
    for (int i = 0; i < NB; i++) {
        glPushMatrix();
        glRotated(i * 360.0 / NB, 0, 0, 1);
        glRotated(14, 0, 1, 0);   // ピッチ捻り
        drawBox(0, st.propDia / 4, 0, 0.09, st.propDia / 2 * 0.96, 0.018);
        glPopMatrix();
    }
    if (NB == 1) { setColor(0x23282e); drawBox(0, -st.propDia / 6, 0, 0.1, 0.13, 0.1); }
    // スピナー
    setColor(0x1b2a4a);
    glPushMatrix();
    if (st.propConfig == "tractor") { glTranslated(0, 0, -0.12); glRotated(90, 1, 0, 0); }
    else { glTranslated(0, 0, 0.12); glRotated(-90, 1, 0, 0); }
    drawCone(0, 0, 0, 0.07, 0.22, 10);
    glPopMatrix();
    glEndList();
}

// ---------- お遊びモード: 小型プロペラ機(高翼・前輪式の超軽量機) ----------
void Renderer3D::buildFunPlane() {
    funBody_ = glGenLists(1);
    glNewList(funBody_, GL_COMPILE);
    // 胴体: 円形断面のロフト(カウル→キャビン→テールコーン)。赤/クリームのツートン
    setColor(0xd8402f);
    {
        const double fz[7] = {-2.80, -2.30, -1.40, -0.20, 1.00, 2.20, 3.15};
        const double fr[7] = {0.26, 0.44, 0.55, 0.54, 0.40, 0.20, 0.07};
        const double fy[7] = {1.02, 1.02, 1.00, 0.98, 1.02, 1.12, 1.22};
        for (int i = 0; i < 6; i++)
            drawStrut({0, fy[i], fz[i]}, {0, fy[i + 1], fz[i + 1]}, fr[i], fr[i + 1], 16);
        drawSphere({0, 1.22, 3.15}, 0.07, 8, 6);      // テール端
    }
    // キャビン(クリーム色の帯)と窓
    setColor(0xf4efe2);
    drawStrut({0, 1.06, -1.55}, {0, 1.06, 0.75}, 0.50, 0.47, 16);
    setColor(0x28394f);
    drawBox(0, 1.50, -0.35, 0.90, 0.34, 1.15);        // 側窓
    glPushMatrix();                                    // 前風防(傾斜)
    glTranslated(0, 1.52, -1.18);
    glRotated(-35, 1, 0, 0);
    drawBox(0, 0, 0, 0.82, 0.58, 0.05);
    glPopMatrix();
    // 主翼: 高翼・内翼一定+外翼テーパー、翼端キャップ(赤)
    for (int s : {-1, 1}) {
        setColor(0xf4efe2);
        drawBox(s * 1.70, 1.90, -0.55, 3.40, 0.13, 1.42);                    // 内翼
        const int NT = 4;                                                     // 外翼テーパー
        for (int i = 0; i < NT; i++) {
            const double u0 = i / (double)NT, u1 = (i + 1) / (double)NT;
            const double y0 = 3.40 + u0 * 1.55, y1 = 3.40 + u1 * 1.55;
            const double c0 = lerp(1.42, 0.90, u0), c1 = lerp(1.42, 0.90, u1);
            const double cm = (c0 + c1) / 2, ym = (y0 + y1) / 2;
            drawBox(s * ym, 1.90, -0.55 - (cm - 1.42) * 0.25, y1 - y0 + 0.02, 0.12, cm);
        }
        setColor(0xd8402f);                            // 翼端キャップ
        drawSphere({s * 4.97, 1.90, -0.43}, 0.10, 8, 6, 0.35);
        drawBox(s * 4.90, 1.90, -0.43, 0.16, 0.115, 0.88);
        // 翼支柱(V字)
        setColor(0x8f9499);
        drawStrut({s * 0.52, 0.85, -0.35}, {s * 2.55, 1.86, -0.75}, 0.035, 0.030, 8);
        drawStrut({s * 0.52, 0.85, -0.35}, {s * 2.55, 1.86, -0.05}, 0.035, 0.030, 8);
    }
    // 水平尾翼(テーパー・翼端赤)+垂直尾翼(後退・赤/クリームのラダー)
    for (int s : {-1, 1}) {
        setColor(0xf4efe2);
        drawBox(s * 0.72, 1.30, 2.72, 1.44, 0.07, 0.86);
        drawBox(s * 1.53, 1.30, 2.78, 0.20, 0.065, 0.70);
        setColor(0xd8402f);
        drawBox(s * 1.66, 1.30, 2.82, 0.07, 0.06, 0.60);
    }
    // 垂直尾翼: 輪郭ポリゴンの押し出し(後退フィン+ヒンジで繋がるラダー+フィレット)
    {
        auto finPoly = [&](const std::vector<std::pair<double, double>>& sh, double th) {
            glBegin(GL_TRIANGLES);
            for (int s2 : {-1, 1}) {                   // 側面(両面)
                glNormal3d(s2, 0, 0);
                for (size_t j = 1; j + 1 < sh.size(); j++) {
                    glVertex3d(s2 * th, sh[0].second, sh[0].first);
                    glVertex3d(s2 * th, sh[j].second, sh[j].first);
                    glVertex3d(s2 * th, sh[j + 1].second, sh[j + 1].first);
                }
            }
            for (size_t j = 0; j < sh.size(); j++) {   // 縁(厚み)
                const size_t k = (j + 1) % sh.size();
                quad({-th, sh[j].second, sh[j].first}, {th, sh[j].second, sh[j].first},
                     {th, sh[k].second, sh[k].first}, {-th, sh[k].second, sh[k].first});
            }
            glEnd();
        };
        setColor(0xd8402f);                            // フィン(赤・後退角)
        finPoly({{2.32, 1.10}, {3.02, 2.34}, {3.34, 2.34}, {3.27, 1.14}}, 0.032);
        finPoly({{1.55, 1.06}, {2.32, 1.10}, {2.55, 1.48}}, 0.028);   // ドーサルフィレット
        setColor(0xf4efe2);                            // ラダー(クリーム・ヒンジで接続)
        finPoly({{3.29, 1.12}, {3.36, 2.32}, {3.58, 2.26}, {3.70, 1.10}}, 0.028);
    }
    // 降着装置(前輪式・スパッツ付き)
    setColor(0x3a3f45);
    drawStrut({0, 0.75, -1.75}, {0, 0.26, -1.85}, 0.045, 0.035, 8);
    for (int s : {-1, 1})
        drawStrut({s * 0.42, 0.62, 0.45}, {s * 1.05, 0.27, 0.52}, 0.05, 0.04, 8);
    setColor(0x23282e);
    drawSphere({0, 0.21, -1.85}, 0.20, 12, 8);
    for (int s : {-1, 1}) drawSphere({s * 1.05, 0.24, 0.52}, 0.24, 12, 8);
    setColor(0xd8402f);                                // ホイールスパッツ
    drawSphere({0, 0.26, -1.85}, 0.145, 10, 7, 0.75);
    for (int s : {-1, 1}) drawSphere({s * 1.05, 0.30, 0.52}, 0.17, 10, 7, 0.75);
    // 排気管
    setColor(0x6a6f75);
    drawStrut({0.32, 0.72, -2.0}, {0.36, 0.62, -1.55}, 0.035, 0.035, 6);
    glEndList();

    funProp_ = glGenLists(1);
    glNewList(funProp_, GL_COMPILE);
    setColor(0x2b2f36);
    for (int i = 0; i < 2; i++) {                      // 2枚ブレード(ピッチ角付き)
        glPushMatrix();
        glRotated(180.0 * i, 0, 0, 1);
        glTranslated(0, 0.50, 0);
        glRotated(17, 0, 1, 0);                        // 幾何ピッチのひねり
        drawBox(0, 0, 0, 0.15, 0.82, 0.045);
        drawSphere({0, 0.40, 0}, 0.072, 8, 6, 0.5);    // 先端の丸み
        glPopMatrix();
    }
    setColor(0xd8402f);                                // スピナー(赤)
    glPushMatrix();
    glTranslated(0, 0, -0.06);
    glRotated(90, 1, 0, 0);
    drawCone(0, 0, 0, 0.115, 0.30, 12);
    glPopMatrix();
    glEndList();
}

void Renderer3D::applyFlex(double fl, double fi) {
    if (flexBasis_.ys.empty()) return;
    const long key = std::lround(fl * 100) * 100000L + std::lround(fi * 100);
    if (key == flexKey_) return;
    flexKey_ = key;
    auto deform = [&](FlexMesh& m) {
        const float ffl = (float)fl, ffi = (float)fi;
        for (size_t i = 0, v = 0; i < m.base.size(); i += 3, v++) {
            m.pos[i] = m.base[i];
            m.pos[i + 1] = m.base[i + 1] + ffl * m.dL[v] - ffi * m.dW[v];
            m.pos[i + 2] = m.base[i + 2];
        }
    };
    deform(wingOpq_);
    deform(wingFilm_);
}

// ---------- 風の可視化 ----------
// 粒子は「その場所の実際の風」(基本風+地形風+サーマル)で移流する。
// 上昇流の中では緑に上っていき、比良おろしの下では橙に沈む — 風場がそのまま見える
void Renderer3D::drawWind(const SimParams& prm, const glm::dvec3& center, double z0, double simT) {
    const double R = 380, dt = 1.0 / 60;
    const int N = 240;
    static thread_local std::mt19937 rng(99);
    std::uniform_real_distribution<double> rnd(0, 1);
    // 粒子の高度帯は機体高度に追従(高高度でも視界に粒子が入る)
    const double yLo = std::max(1.0, center.y - 45.0);
    const double yHi = std::max(center.y + 35.0, 56.0);
    auto respawn = [&](glm::dvec3& p) {
        p = {center.x + (rnd(rng) * 2 - 1) * R,
             yLo + rnd(rng) * (yHi - yLo),
             center.z + (rnd(rng) * 2 - 1) * R};
    };
    if ((int)windPts_.size() != N) {
        windPts_.resize(N);
        for (auto& p : windPts_) respawn(p);
    }
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glLineWidth(1.5f);
    glBegin(GL_LINES);
    for (auto& p : windPts_) {
        // ワールド → コース座標(x=前方, yl=右)
        const double cx = z0 - p.z, yl = p.x;
        double aw = 0, ax = 0, av = 0;
        if (prm.terrainWind) localWind(prm, cx, yl, p.y, aw, ax, av);
        double tvz = prm.realThermal ? thermalFieldVz(prm, cx, yl, simT)
                                     : thermalCellVz(prm, cx, yl);
        // 物理(stepThermal)と同じ安定成層の高度減衰を可視化にも反映
        if (prm.realThermal && tvz > 0) tvz *= 0.30 + 0.70 * std::exp(-p.y / 60.0);
        const double vz = tvz + av;
        // ワールド速度: コース前方+x は ワールド-Z
        const glm::dvec3 v(prm.xwind + ax, vz, -(prm.wind + aw));
        p += v * dt;
        const double sp = glm::length(v);
        if (std::abs(p.x - center.x) > R || std::abs(p.z - center.z) > R ||
            p.y < 0.5 || p.y > yHi + 25) respawn(p);
        if (sp < 0.02) continue;                    // 完全無風のみ描かない
        // 微風でも見えるよう透明度と線長に下限を設ける(向き・速度情報は正確なまま)
        const float al = (float)clamp(0.25 + sp / 3.0, 0.30, 0.80);
        if (v.y > 0.15) glColor4f(0.45f, 0.95f, 0.55f, al);        // 上昇流=緑
        else if (v.y < -0.15) glColor4f(1.0f, 0.62f, 0.30f, al);   // 下降流=橙
        else glColor4f(0.92f, 0.96f, 1.0f, al * 0.9f);             // 水平風=白
        const glm::dvec3 tail = p - v / sp * std::max(sp * 1.6, 1.0);
        glVertex3d(p.x, p.y, p.z);
        glVertex3d(tail.x, tail.y, tail.z);
    }
    glEnd();
    glLineWidth(1.0f);
    // アクティブなサーマルの上に薄い積雲マーカー(グライダー乗りの目印)
    if (prm.realThermal && prm.thermal > 0) {
        double sx[24], syl[24], sstr[24], srad[24];
        const int n = thermalSitesNear(prm, z0 - center.z, center.x, simT, 24, sx, syl, sstr, srad);
        for (int i = 0; i < n; i++) {
            const float al = (float)clamp(sstr[i] * 0.5, 0.10, 0.30);
            glColor4f(1, 1, 1, al);
            drawSphere({syl[i], 110 + srad[i] * 0.2, -(sx[i])}, srad[i] * 0.55, 8, 5, 0.35);
        }
    }
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
}

// 平らな楕円ディスク(機体の影用)
static void drawFlatDisc(double cx, double y, double cz, double rx, double rz, int seg = 24) {
    glBegin(GL_TRIANGLE_FAN);
    glNormal3d(0, 1, 0);
    glVertex3d(cx, y, cz);
    for (int i = 0; i <= seg; i++) {
        const double a = 2 * PI * i / seg;
        glVertex3d(cx + rx * std::cos(a), y, cz + rz * std::sin(a));
    }
    glEnd();
}

void Renderer3D::drawGroundFx(const glm::dvec3& p, double span, bool overWater) {
    const double h = p.y;
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    const double gy = overWater ? -0.30 : 0.07;
    // 機体の影: 低高度ほど濃く(地面効果域の高度感の手がかり)
    if (h < 30) {
        glColor4f(0.04f, 0.09f, 0.14f, (float)(0.30 * (1 - h / 30.0)));
        drawFlatDisc(p.x, gy, p.z, span * 0.5, 1.6);
    }
    // 地面効果域(h<3.5)では水面にさざ波リングが走る
    rippleTimer_ += 1.0 / 60;
    if (overWater && h < 3.5 && h > 0.05 && rippleTimer_ > 0.30) {
        rippleTimer_ = 0;
        CrashRing r;
        r.pos = {p.x, -0.29, p.z};
        r.t = 0;
        r.max = clamp(1.0 - h / 3.5, 0.0, 1.0);   // 低いほど強い(強度として使用)
        r.delay = span;
        r.water = true;
        ripples_.push_back(r);
    }
    for (auto& r : ripples_) {
        r.t += 1.0 / 60;
        const double life = 1.3;
        if (r.t > life) continue;
        const float al = (float)(0.34 * r.max * (1 - r.t / life));
        glColor4f(0.92f, 0.98f, 1.0f, al);
        const double rr = r.delay * 0.12 + r.t * r.delay * 0.30;   // delay=翼幅
        drawFlatRing(r.pos, rr, rr + 0.6 + r.t * 0.8, 28);
    }
    ripples_.erase(std::remove_if(ripples_.begin(), ripples_.end(),
                                  [](const CrashRing& r) { return r.t > 1.3; }),
                   ripples_.end());
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
}

void Renderer3D::drawSimplePlane(const glm::dvec3& pos, double span, unsigned color) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    setColor(color, 0.70f);
    drawBox(pos.x, pos.y, pos.z, span * 0.9, 0.14, 0.6);          // 主翼
    drawBox(pos.x, pos.y - 0.5, pos.z + 1.2, 0.5, 1.0, 2.5);      // 胴体
    drawBox(pos.x, pos.y + 0.3, pos.z + 4.2, 2.4, 0.1, 0.5);      // 水平尾翼
    drawBox(pos.x, pos.y + 0.7, pos.z + 4.2, 0.08, 0.9, 0.55);    // 垂直尾翼
    glDisable(GL_BLEND);
}

// 発進方向矢印(富士川の自由発進UI・第5弾)。機体足元の地面(y≈0.1)に
// 半透明青の平たい矢印を機首方位hdgRadへ向けて描く
void Renderer3D::drawStartArrow(const glm::dvec3& pos, double hdgRad) {
    // コース前方=ワールド-Z。hdgRad>0(右旋回方向)はワールド+Xへ振れる
    const glm::dvec3 dir(std::sin(hdgRad), 0, -std::cos(hdgRad));
    const glm::dvec3 right(std::cos(hdgRad), 0, std::sin(hdgRad));
    const double y = 0.12;
    const glm::dvec3 o(pos.x, y, pos.z);
    const double shaftL = 6.0, shaftW = 0.6, headL = 3.0, headW = 1.6;
    glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    setColor(0x2c6fd9, 0.55f);
    glBegin(GL_TRIANGLES);
    glNormal3d(0, 1, 0);
    // 軸(細長い矩形)
    {
        const glm::dvec3 a = o - right * shaftW, b = o + right * shaftW;
        const glm::dvec3 c = b + dir * shaftL, d = a + dir * shaftL;
        glVertex3d(a.x, a.y, a.z); glVertex3d(b.x, b.y, b.z); glVertex3d(c.x, c.y, c.z);
        glVertex3d(a.x, a.y, a.z); glVertex3d(c.x, c.y, c.z); glVertex3d(d.x, d.y, d.z);
    }
    // 矢じり(三角)
    {
        const glm::dvec3 baseC = o + dir * shaftL;
        const glm::dvec3 a = baseC - right * headW, b = baseC + right * headW;
        const glm::dvec3 tip = baseC + dir * headL;
        glVertex3d(a.x, a.y, a.z); glVertex3d(b.x, b.y, b.z); glVertex3d(tip.x, tip.y, tip.z);
    }
    glEnd();
    glDepthMask(GL_TRUE);
    glPopAttrib();
    glEnable(GL_LIGHTING);
}

// 設計モード: フラップ区間の緑半透明ハイライト(内翼後縁の帯)
void Renderer3D::drawFlapHighlight(const AircraftParams& st, const Analysis& an) {
    if (st.flapSpanFrac <= 0.005) return;
    const double half = st.span / 2, xw = st.flapSpanFrac * half;
    // 後縁30%相当(内翼はほぼ翼根コード)
    const double z0 = wingZ_ + st.rootChord * 0.68, z1 = wingZ_ + st.rootChord * 1.04;
    glPushAttrib(GL_ENABLE_BIT | GL_DEPTH_BUFFER_BIT | GL_CURRENT_BIT | GL_LINE_BIT);
    glPushMatrix();
    glMultMatrixd(glm::value_ptr(wingDrawTransform_));
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    setColor(0x35c97e, 0.28f);
    drawBox(0, wingH_ + 0.02, (z0 + z1) / 2, xw * 2, 0.20, z1 - z0);
    // 上面に枠線(区間の視認性を上げる)
    setColor(0x35c97e, 0.9f);
    glLineWidth(1.5f);
    glBegin(GL_LINE_LOOP);
    const double yTop = wingH_ + 0.13;
    glVertex3d(-xw, yTop, z0); glVertex3d(xw, yTop, z0);
    glVertex3d(xw, yTop, z1);  glVertex3d(-xw, yTop, z1);
    glEnd();
    glDepthMask(GL_TRUE);
    glPopMatrix();
    glPopAttrib();
    glEnable(GL_LIGHTING);
}

void Renderer3D::spawnCrashFx(const glm::dvec3& pos, bool water) {
    clearFx();
    const int n = water ? 90 : 60;
    for (int i = 0; i < n; i++) {
        CrashParticle p;
        p.water = water;
        p.debris = !water && frand() < 0.5;
        p.size = water ? (0.05 + frand() * 0.11) : (0.05 + frand() * 0.13);
        unsigned col = water ? (frand() < 0.45 ? 0xffffff : 0xbfe3f5)
                             : (p.debris ? (frand() < 0.5 ? 0xe7d9b8 : 0x23282e) : 0xcdbd97);
        p.r = ((col >> 16) & 255) / 255.0f; p.g = ((col >> 8) & 255) / 255.0f; p.b = (col & 255) / 255.0f;
        double ang = frand() * PI * 2;
        double sp = water ? (2.5 + frand() * 6.5) : (1.8 + frand() * 5);
        double up = water ? (5 + frand() * 8) : (2 + frand() * 5);
        p.pos = pos;
        p.vel = {std::cos(ang) * sp, up, std::sin(ang) * sp};
        p.life = 1;
        p.spin = p.debris ? (frand() - 0.5) * 12 : 0;
        p.spinAxis = glm::normalize(glm::dvec3(frand() + 0.01, frand(), frand()));
        parts_.push_back(p);
    }
    const int rn = water ? 3 : 1;
    for (int k = 0; k < rn; k++)
        rings_.push_back({k * 0.18, water ? (9.0 + k * 3) : 5.0, 0, {pos.x, 0.05, pos.z}, water});
    if (water) { colPos_ = {pos.x, 0, pos.z}; colLife_ = 1.2; colT_ = 0; }
}
void Renderer3D::clearFx() { parts_.clear(); rings_.clear(); colLife_ = 0; }
void Renderer3D::tickFx(double dt) {
    for (auto& p : parts_) {
        if (p.life <= 0) continue;
        p.vel.y -= 14 * dt;
        p.pos += p.vel * dt;
        const double floor = p.water ? 0.02 : 0.0;
        if (p.pos.y < floor) { p.pos.y = floor; p.vel.y *= -0.35; p.vel.x *= 0.6; p.vel.z *= 0.6; }
        p.life -= dt * 0.5;
    }
    for (auto& r : rings_) r.t += dt;
    if (colLife_ > 0) colT_ += dt;
    if (!rings_.empty() && rings_[0].t > 2.6) clearFx();
}
void Renderer3D::drawFx() {
    if (parts_.empty() && rings_.empty()) return;
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    for (const auto& p : parts_) {
        if (p.life <= 0) continue;
        glColor4f(p.r, p.g, p.b, (float)std::max(0.0, p.life * 0.92));
        double s = p.spin ? p.size : std::max(0.1, p.life) * p.size;
        if (p.debris) drawBox(p.pos.x, p.pos.y, p.pos.z, s * 1.8, s * 0.3, s * 0.6);
        else drawSphere(p.pos, s, 5, 4);
    }
    for (const auto& r : rings_) {
        double tt = r.t - r.delay;
        if (tt < 0) continue;
        double s = 1 + tt * r.max;
        float al = (float)std::max(0.0, 0.7 * (1 - tt / 1.5));
        if (r.water) setColor(0xffffff, al); else setColor(0xc9b78f, al);
        drawFlatRing({r.pos.x, r.pos.y, r.pos.z}, 0.3 * s, 0.55 * s, 32);
    }
    if (colLife_ > 0 && colT_ < 1.2) {
        setColor(0xeaf6ff, (float)std::max(0.0, 0.85 * (1 - colT_ / 1.2)));
        double sy = std::max(0.05, 1 - colT_ * 1.1), sr = 1 + colT_ * 0.5;
        glm::dvec3 base(colPos_.x, 0, colPos_.z), top(colPos_.x, std::max(0.0, 1.2 - colT_) + 2.4 * sy * 0.5, colPos_.z);
        drawStrut(base, top, 0.5 * sr, 0.25 * sr, 10);
    }
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

// ---------- フレーム描画 ----------
// ---------- 画面全面グラデーション空(深度無効の正射影クアッド) ----------
static void drawSkyGradient(int zr, int zg, int zb, int hr, int hg, int hb) {
    glPushAttrib(GL_ENABLE_BIT | GL_DEPTH_BUFFER_BIT | GL_CURRENT_BIT);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_FOG);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glBegin(GL_TRIANGLES);
    glColor3ub((GLubyte)zr, (GLubyte)zg, (GLubyte)zb); glVertex3f(-1, 1, 0);
    glColor3ub((GLubyte)hr, (GLubyte)hg, (GLubyte)hb); glVertex3f(-1, -1, 0);
    glColor3ub((GLubyte)hr, (GLubyte)hg, (GLubyte)hb); glVertex3f(1, -1, 0);
    glColor3ub((GLubyte)zr, (GLubyte)zg, (GLubyte)zb); glVertex3f(-1, 1, 0);
    glColor3ub((GLubyte)hr, (GLubyte)hg, (GLubyte)hb); glVertex3f(1, -1, 0);
    glColor3ub((GLubyte)zr, (GLubyte)zg, (GLubyte)zb); glVertex3f(1, 1, 0);
    glEnd();
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopAttrib();
}

// ---------- 太陽(flightモード。todから方位・高度を概算し加算ブレンドで描画) ----------
static void drawSun(const glm::mat4& view, double tod) {
    // 6時=東(低)/12時=南(高)/17時=西(低)。高度は日中に最大~70°まで持ち上がる簡易カーブ
    const double elevDeg = std::max(2.0, 70.0 * std::sin(clamp((tod - 5.0) / 13.0, 0.0, 1.0) * PI));
    const double azDeg = 90.0 + (tod - 6.0) / 12.0 * 180.0;   // 6h→90°(東) 12h→180°(南) 18h→270°(西)
    const double elev = elevDeg * PI / 180.0, az = azDeg * PI / 180.0;
    const glm::dvec3 dir(std::sin(az) * std::cos(elev), std::sin(elev), -std::cos(az) * std::cos(elev));
    const glm::dvec3 sunPos = dir * 9000.0;
    // カメラのright/up(ワールド座標)をビュー行列の回転部から取得(ビルボード用)
    const glm::dvec3 right(view[0][0], view[1][0], view[2][0]);
    const glm::dvec3 up(view[0][1], view[1][1], view[2][1]);

    glPushAttrib(GL_ENABLE_BIT | GL_DEPTH_BUFFER_BIT | GL_CURRENT_BIT);
    glDisable(GL_LIGHTING);
    glDisable(GL_FOG);
    glDisable(GL_TEXTURE_2D);
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);   // 加算ブレンド
    auto billboard = [&](double r, unsigned hex, float a) {
        setColor(hex, a);
        glBegin(GL_TRIANGLE_FAN);
        glVertex3d(sunPos.x, sunPos.y, sunPos.z);
        for (int i = 0; i <= 20; i++) {
            const double t = 2 * PI * i / 20;
            glm::dvec3 p = sunPos + (right * std::cos(t) + up * std::sin(t)) * r;
            glVertex3d(p.x, p.y, p.z);
        }
        glEnd();
    };
    billboard(650, 0xfff3d0, 0.35f);   // グロー(外側・淡い)
    billboard(220, 0xffffff, 0.9f);    // コア(内側・明るい)
    glDepthMask(GL_TRUE);
    glPopAttrib();
}

// ---------- 設計モードのスタジオ床(機体下の円形床+ソフト影) ----------
static void drawStudioFloor(double span) {
    if (span <= 0) return;
    glPushAttrib(GL_ENABLE_BIT | GL_DEPTH_BUFFER_BIT | GL_CURRENT_BIT);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    const int seg = 48;
    // 円形床(放射グラデーション。半径=span*0.7)。design背景は元々暗いため、
    // 中心をほぼ黒(alpha高め)にして周囲のグラデ空との明度差でシルエットを出す
    const double R = std::max(6.0, span * 0.7);
    glBegin(GL_TRIANGLE_FAN);
    glColor4f(0, 0, 0, 0.72f); glVertex3d(0, -0.02, 0);
    glColor4f(0, 0, 0, 0.0f);
    for (int i = 0; i <= seg; i++) {
        const double a = 2 * PI * i / seg;
        glVertex3d(R * std::cos(a), -0.02, R * std::sin(a));
    }
    glEnd();
    // 機体直下のソフト楕円影(円形床よりさらに濃く、alpha~130/255)
    const double sx = std::max(2.0, span * 0.26), sz = sx * 0.55;
    glBegin(GL_TRIANGLE_FAN);
    glColor4f(0, 0, 0, 130.f / 255.f); glVertex3d(0, -0.01, 0);
    glColor4f(0, 0, 0, 0.0f);
    for (int i = 0; i <= seg; i++) {
        const double a = 2 * PI * i / seg;
        glVertex3d(sx * std::cos(a), -0.01, sz * std::sin(a));
    }
    glEnd();
    glDepthMask(GL_TRUE);
    glPopAttrib();
    glEnable(GL_LIGHTING);
}

void Renderer3D::drawFrame(const std::string& mode, const SimParams& prm, const Camera& cam,
                           const glm::dvec3& acPos, const glm::dvec3& acRot, bool acVisible,
                           double propAngle,
                           const std::vector<glm::dvec3>& trail,
                           bool ghostVisible, const glm::dvec3& ghostPos, double ghostSpan) {
    glViewport(0, 0, vpW_, vpH_);
    // 空・霧の色(時刻)
    int r, g, b;
    if (mode == "flight") skyColor(prm.tod, r, g, b);
    else { r = 10; g = 13; b = 18; }   // designモード: 機体が映える黒基調
    // 琵琶湖・夏モードの早朝は靄(もや)がかかる(枝川1986の知見(2): 湖上の湿度が春季に
    // 最大という観測の視覚的表現)。tod<9時で視程を段階的に狭め、空にわずかな乳白を足す。
    // 9時以降は通常(視程14km)に戻る。富士川・design画面・非夏モードは対象外
    const bool biwaHaze = mode == "flight" && prm.site != "fujikawa" && prm.summer && prm.tod < 9.0;
    float fogEnd = 11000.0f, fogStart = 800.0f;
    if (biwaHaze) {
        const double hf = clamp(1.0 - (prm.tod - 6.0) / 3.0, 0.0, 1.0);   // 6時=1(濃霧)→9時=0
        const double vis = 6000.0 + (14000.0 - 6000.0) * (1.0 - hf);      // 6時6km→9時14km
        fogEnd = (float)vis;
        fogStart = (float)(vis * 0.10);
        r = (int)clamp(r + 40 * hf, 0.0, 255.0);
        g = (int)clamp(g + 35 * hf, 0.0, 255.0);
        b = (int)clamp(b + 25 * hf, 0.0, 255.0);
    } else if (mode == "flight" && prm.site != "fujikawa" && prm.summer) {
        fogEnd = 14000.0f;   // 琵琶湖・夏・9時以降は通常視程
    } else if (mode == "flight" && prm.site == "fujikawa") {
        // 蒲原の河口は海由来の湿った霞があり、遠景は灰青へ溶け込む。
        // 不具合修正: 旧値(fogStart=650m)は霧が近すぎ、2〜3km先の川面が
        // ほぼ空色に塗り潰されて「水色の帯(穴)」に見える主因だった。
        // 開始を2.5kmへ後退・視程も伸ばし、近〜中距離の川の色を保つ
        const double daylight = clamp(1.0 - std::abs(prm.tod - 12.0) / 9.0, 0.15, 1.0);
        fogStart = 2500.0f;
        fogEnd = (float)(12000.0 + 1500.0 * daylight);
        r = (int)clamp(r + 4 + 5 * daylight, 0.0, 255.0);
        g = (int)clamp(g + 5 + 6 * daylight, 0.0, 255.0);
        b = (int)clamp(b + 4 + 5 * daylight, 0.0, 255.0);
    }
    glFogf(GL_FOG_START, fogStart);
    glFogf(GL_FOG_END, fogEnd);
    glClearColor(r / 255.0f, g / 255.0f, b / 255.0f, 1);
    const float fogc[4] = {r / 255.0f, g / 255.0f, b / 255.0f, 1};
    glFogfv(GL_FOG_COLOR, fogc);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // 画面全面のグラデーション空(天頂→地平線)。霧色=地平線色に合わせて馴染ませる
    if (mode == "flight")
        drawSkyGradient((int)clamp(r * 0.72, 0.0, 255.0), (int)clamp(g * 0.72, 0.0, 255.0), (int)clamp(b * 0.72, 0.0, 255.0),
                        (int)clamp(r * 1.12, 0.0, 255.0), (int)clamp(g * 1.12, 0.0, 255.0), (int)clamp(b * 1.12, 0.0, 255.0));
    else
        drawSkyGradient(6, 8, 12, 24, 30, 42);

    glMatrixMode(GL_PROJECTION);
    // near=0.4: 0.1だと遠距離の深度精度が数mまで落ち、地面レイヤーが透ける
    // (最小ズーム3m/FPVは自機非表示のためクリッピングの実害なし)
    glm::mat4 proj = glm::perspective(glm::radians(45.0f), (float)vpW_ / vpH_, 0.4f, 14000.0f);
    glLoadMatrixf(glm::value_ptr(proj));
    glMatrixMode(GL_MODELVIEW);
    glm::mat4 view = cam.viewMatrix();
    glLoadMatrixf(glm::value_ptr(view));

    // 太陽光の向き(ビュー行列適用後に設定)
    const float lp[4] = {20, 30, -15, 0};
    glLightfv(GL_LIGHT0, GL_POSITION, lp);
    // designモードは背景が黒基調のため、環境光を少し上げて機体の陰を持ち上げる
    if (mode == "design") {
        const float ambDesign[4] = {0.72f, 0.74f, 0.78f, 1.0f};
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambDesign);
    } else {
        const float ambFlight[4] = {0.55f, 0.58f, 0.62f, 1.0f};
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambFlight);
    }

    glm::dvec3 camPos = cam.fpv ? glm::dvec3(cam.fpvPose.x, cam.fpvPose.y, cam.fpvPose.z) : cam.eye();

    if (mode == "flight") {
        drawSun(view, prm.tod);
        glCallList(envList_);
        // 湖面スケールグリッドは琵琶湖専用。富士川では川面の色と流路を隠すため重ねない。
        if (prm.site != "fujikawa") drawLakeAndGrid(camPos);
    } else {
        glCallList(designList_);
        // 設計モードのスタジオ演出: 円形床+機体直下のソフト影(半径は現在の翼幅から算出)
        // ghostSpanは設計モードでは常にGame側から現在のst_.spanが渡される
        drawStudioFloor(ghostSpan);
    }

    // 航跡
    if (!trail.empty()) {
        glDisable(GL_LIGHTING);
        setColor(0xe8590c);
        glBegin(GL_LINE_STRIP);
        for (const auto& p : trail) glVertex3d(p.x, p.y, p.z);
        glEnd();
        glEnable(GL_LIGHTING);
    }
    // ゴースト機(自己ベスト)
    if (ghostVisible) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        setColor(0x44aaff, 0.35f);
        drawBox(ghostPos.x, ghostPos.y, ghostPos.z, ghostSpan * 0.9, 0.12, 0.5);
        drawBox(ghostPos.x, ghostPos.y, ghostPos.z + 3, 2, 0.1, 0.4);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }

    // 機体(位置・姿勢はJSの aircraft.position / rotation.set(gam,-psi,phi) と同順)
    if (acVisible && acOpaque_ && prm.funPlane && mode == "flight") {
        // お遊びモード: 小型プロペラ機
        if (!funBody_) buildFunPlane();
        glPushMatrix();
        glTranslated(acPos.x, acPos.y, acPos.z);
        glRotated(acRot.y * 180 / PI, 0, 1, 0);   // YXZ順: ヨー(最外)
        glRotated(acRot.x * 180 / PI, 1, 0, 0);   // → ピッチ
        glRotated(acRot.z * 180 / PI, 0, 0, 1);   // → ロール(機体軸)
        glCallList(funBody_);
        glPushMatrix();
        glTranslated(0, 1.02, -2.92);
        glRotated(propAngle * 180 / PI, 0, 0, 1);
        glCallList(funProp_);
        glPopMatrix();
        glPopMatrix();
    } else if (acVisible && acOpaque_) {
        glPushMatrix();
        glTranslated(acPos.x, acPos.y, acPos.z);
        // YXZ順(ヨー→ピッチ→ロール): FPVカメラと同一。JSのXYZ順は方位が
        // 変わるとピッチ/ロールが混ざる(180°で上昇が機首下げに見える)ため修正
        glRotated(acRot.y * 180 / PI, 0, 1, 0);
        glRotated(acRot.x * 180 / PI, 1, 0, 0);
        glRotated(acRot.z * 180 / PI, 0, 0, 1);
        glCallList(acOpaque_);
        glPushMatrix();
        glMultMatrixd(glm::value_ptr(wingDrawTransform_));
        drawFlexMesh(wingOpq_);          // 主翼(たわみ変形)
        glPopMatrix();
        // プロペラ(回転)
        glPushMatrix();
        glMultMatrixd(glm::value_ptr(propWorld_));
        glRotated(propAngle * 180 / PI, 0, 0, 1);
        glCallList(propList_);
        glPopMatrix();
        // 半透明フィルム
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        glCallList(acFilm_);
        glPushMatrix();
        glMultMatrixd(glm::value_ptr(wingDrawTransform_));
        drawFlexMesh(wingFilm_);         // 翼フィルム(たわみ変形)
        glPopMatrix();
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        glPopMatrix();
    }
    drawFx();
}

} // namespace bm
