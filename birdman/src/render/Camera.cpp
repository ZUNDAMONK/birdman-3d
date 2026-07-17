#include "render/Camera.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>

namespace bm {

void Camera::rotate(double dx, double dy) {
    theta -= dx * 0.006;
    phi = std::min(3.14159265 - 0.05, std::max(0.05, phi - dy * 0.006));
}

void Camera::pan(double dx, double dy) {
    // カメラのright/upベクトルに沿って注視点を移動 (JSと同じ係数)
    const double s = dist * 0.0012;
    glm::dvec3 fwd = glm::normalize(target - eye());
    glm::dvec3 right = glm::normalize(glm::cross(fwd, glm::dvec3(0, 1, 0)));
    glm::dvec3 up = glm::cross(right, fwd);
    target += right * (-dx * s) + up * (dy * s);
}

void Camera::zoom(double wheelDelta, double minD, double maxD) {
    dist = std::min(maxD, std::max(minD, dist * (1 + wheelDelta * 0.0012)));
}

glm::dvec3 Camera::eye() const {
    return {target.x + dist * std::sin(phi) * std::sin(theta),
            target.y + dist * std::cos(phi),
            target.z + dist * std::sin(phi) * std::cos(theta)};
}

glm::mat4 Camera::viewMatrix() const {
    if (fpv) {
        // JS applyFPV: Euler(gam, -psi, phi, "YXZ")、目線は機体位置+1.15m
        const FpvPose& s = fpvPose;
        glm::dvec3 eyeP(s.x, s.y + 1.15, s.z + 0.2);
        glm::dmat4 R(1.0);
        R = glm::rotate(R, -s.psi, glm::dvec3(0, 1, 0));
        R = glm::rotate(R, s.gam, glm::dvec3(1, 0, 0));
        // -φ: φ>0(右バンク)で視界の地平線が左上がりになる正しい傾き
        // (機体モデルと同じくJS版からの符号バグを修正)
        R = glm::rotate(R, -s.phi, glm::dvec3(0, 0, 1));
        glm::dvec3 fwd = glm::dvec3(R * glm::dvec4(0, 0, -1, 0));
        glm::dvec3 up = glm::dvec3(R * glm::dvec4(0, 1, 0, 0));
        return glm::lookAt(glm::vec3(eyeP), glm::vec3(eyeP + fwd), glm::vec3(up));
    }
    return glm::lookAt(glm::vec3(eye()), glm::vec3(target), glm::vec3(0, 1, 0));
}

} // namespace bm
