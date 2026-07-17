#pragma once
// オービット/FPVカメラ (JSの orbit / applyCam / applyFPV)
#include <glm/glm.hpp>

namespace bm {

struct FpvPose { double x = 0, y = 1, z = 0, gam = 0, psi = 0, phi = 0; };

class Camera {
public:
    // オービットパラメータ (JS: orbit)
    double theta = -0.7, phi = 1.15, dist = 26;
    glm::dvec3 target{0, 2, 0};
    bool fpv = false;
    FpvPose fpvPose;

    void rotate(double dx, double dy);          // ドラッグ回転
    void pan(double dx, double dy);             // shift+ドラッグ平行移動
    void zoom(double wheelDelta, double minD, double maxD);
    glm::dvec3 eye() const;                     // オービット時の視点位置
    glm::mat4 viewMatrix() const;               // FPV考慮済みビュー行列
};

} // namespace bm
