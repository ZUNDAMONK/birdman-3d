#include "render/HUD.hpp"
#include "ui/Theme.hpp"
#include "core/Weather.hpp"
#include "core/DesignIO.hpp"
#include "core/Aircraft.hpp"   // clamp()
#include <cmath>
#include <cstdio>
#include <algorithm>

namespace bm {

static const double PI = 3.14159265358979323846;
// テーマ色は関数経由で取得(flight=ライト/design=ダークへ毎フレーム追従する。
// static const 参照ではモード切替に追従しないため必ず関数化)
static sf::Color INK()     { return theme::text(); }
static sf::Color INKSOFT() { return theme::textDim(); }
static sf::Color ACCENT()  { return theme::blue(); }
static sf::Color OKC()     { return theme::ok(); }
static sf::Color WARNC()   { return theme::warn(); }
static sf::Color BADC()    { return theme::bad(); }
// グラフ緑/黄はライト背景で読める彩度に落とす
static sf::Color GREEN()   { return theme::light ? sf::Color(0x18, 0x8A, 0x50) : sf::Color(0x7C, 0xFC, 0x9A); }
static sf::Color GOLD()    { return theme::light ? sf::Color(0xB0, 0x78, 0x00) : sf::Color(0xFF, 0xD2, 0x3F); }

bool HUD::init() {
    // 同梱フォント(assets/、exe位置基準)を最優先 → システムフォントにフォールバック。
    // 同梱によりどのWindows/Linux環境でも同じ見た目で動くポータブル配布が可能
    const std::string bundled[] = {
        exeDirPath() + "/assets/fonts/NotoSansJP-Regular.otf",
        "assets/fonts/NotoSansJP-Regular.otf"};
    for (const auto& p : bundled)
        if (font_.loadFromFile(p)) return true;
    const char* paths[] = {
        "C:/Windows/Fonts/YuGothM.ttc", "C:/Windows/Fonts/meiryo.ttc",
        "C:/Windows/Fonts/msgothic.ttc", "C:/Windows/Fonts/YuGothR.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"};
    for (const char* p : paths)
        if (font_.loadFromFile(p)) return true;
    return false;
}

// ライト時=濃紺/ダーク時=白 の単色(半透明ライン・目盛り用)
static sf::Color mono(int a) {
    return theme::light ? sf::Color(27, 42, 74, (sf::Uint8)a) : sf::Color(255, 255, 255, (sf::Uint8)a);
}

static void line(sf::RenderTarget& rt, float x0, float y0, float x1, float y1, sf::Color c, float w = 1.5f) {
    sf::Vector2f d(x1 - x0, y1 - y0);
    float len = std::sqrt(d.x * d.x + d.y * d.y);
    if (len < 1e-4f) return;
    sf::RectangleShape r({len, w});
    r.setOrigin(0, w / 2);
    r.setPosition(x0, y0);
    r.setRotation(std::atan2(d.y, d.x) * 180 / (float)PI);
    r.setFillColor(c);
    rt.draw(r);
}

void HUD::drawWindArrow(sf::RenderTarget& rt, float cx, float cy, const SimParams& prm, double psi) {
    sf::CircleShape ring(13);
    ring.setOrigin(13, 13);
    ring.setPosition(cx, cy);
    ring.setFillColor(sf::Color(0, 0, 0, 40));
    ring.setOutlineColor(mono(80));
    ring.setOutlineThickness(1.5f);
    rt.draw(ring);
    if (prm.wspd > 0.1) {
        const double ww = prm.wind, wx = prm.xwind;
        const double fwd = ww * std::cos(psi) + wx * std::sin(psi);
        const double rgt = -ww * std::sin(psi) + wx * std::cos(psi);
        const double rel = std::atan2(-rgt, -fwd);
        // 機首マーク
        line(rt, cx, cy - 2, cx, cy + 6, mono(100), 1.5f);
        const float s = (float)std::sin(rel), c = (float)std::cos(rel);
        auto rot = [&](float x, float y) { return sf::Vector2f(cx + x * c - y * s, cy + x * s + y * c); };
        sf::Vector2f a = rot(0, -12), b = rot(0, -4);
        line(rt, a.x, a.y, b.x, b.y, GREEN(), 2.5f);
        sf::ConvexShape tri(3);
        tri.setPoint(0, rot(0, -1)); tri.setPoint(1, rot(4, -7)); tri.setPoint(2, rot(-4, -7));
        tri.setFillColor(GREEN());
        rt.draw(tri);
    } else {
        sf::CircleShape dot(2.5f);
        dot.setOrigin(2.5f, 2.5f);
        dot.setPosition(cx, cy);
        dot.setFillColor(mono(100));
        rt.draw(dot);
    }
}

float HUD::drawFlightBar(sf::RenderTarget& rt, float W, float H,
                         const FlightState* L, const AircraftConstants* c,
                         const SimParams& prm, const SimResult& res,
                         bool flying, bool waiting, const std::string& finMsg) {
    const float barH = finMsg.empty() ? 58.f : 84.f;
    const float top = H - barH;
    drawPanelRect(rt, {0, top, W, barH}, theme::panelSoft(), theme::line(), 1);
    auto item = [&](float x, const std::string& lab, const std::string& val, sf::Color vc) {
        drawText(rt, font_, lab, x, top + 8, 11, INKSOFT(), 1);
        drawText(rt, font_, val, x, top + 22, 19, vc, 1, true);
        return x;
    };
    char b1[64], b2[64];
    if (flying && !waiting && L) {
        const double dist = L->path;
        // 対地速度は真の水平地上速度の大きさ(機首方位・横風・追い/向かい風を全て含む)。
        // 従来は横風成分を無視していた
        const double vAir = L->V;
        const double vh = vAir * std::cos(L->gam);
        const double vgx = vh * std::cos(L->psi) + prm.wind;
        const double vgy = vh * std::sin(L->psi) + prm.xwind;
        const double vGnd = std::sqrt(vgx * vgx + vgy * vgy);
        const bool stall = vAir < res.Vs && !L->ground;
        const bool hasFlap = c && c->hasFlap;
        float x = W / 2 - 330 - (c && c->varPitch ? 48 : 0) - (hasFlap ? 48 : 0);
        std::snprintf(b1, sizeof(b1), "%.0f m", dist); item(x, u8"距離", b1, INK()); x += 110;
        std::snprintf(b1, sizeof(b1), "%.1f m/s", vAir); item(x, u8"対気速度", b1, stall ? BADC() : INK()); x += 110;
        std::snprintf(b1, sizeof(b1), "%.1f m/s", vGnd); item(x, u8"対地速度", b1, INK()); x += 110;
        if (prm.funPlane) std::snprintf(b1, sizeof(b1), u8"%.0f %%", L->Pnow / 350);   // 35kW=100%
        else std::snprintf(b1, sizeof(b1), "%.0f W", L->Pnow);
        item(x, prm.funPlane ? u8"スロットル" : u8"出力", b1, INK()); x += 95;
        std::snprintf(b1, sizeof(b1), "%.0f rpm", L->rpm); item(x, u8"回転", b1, INK()); x += 100;
        if (c && c->varPitch) {
            std::snprintf(b1, sizeof(b1), "%.1f m", c->propPitch0 + L->pitchOfs);
            item(x, u8"ペラピッチ", b1, INK()); x += 96;
        }
        if (hasFlap) {
            // フラップ装備機は展開率0%でも常時表示: 数値+3セグメントゲージ(0/50/100)+Gキーヒント
            const bool deployed = L->flap > 0.005;
            std::snprintf(b1, sizeof(b1), u8"%.0f %%", L->flap * 100);
            drawText(rt, font_, u8"フラップ [G]", x, top + 8, 11, INKSOFT(), 1);
            drawText(rt, font_, b1, x, top + 20, deployed ? 16u : 14u,
                     deployed ? GREEN() : INK(), 1, deployed);
            // 3セグメントゲージ: 目標値(flapTgt)基準で 50%=1セグ, 100%=2セグ…ではなく
            // 0/50/100の3段階を表すため、実展開率で 1/3 ずつ塗る
            const float gw = 60, gh = 5, gx0 = x - gw / 2, gy = top + 42;
            for (int sgi = 0; sgi < 3; sgi++) {
                const float sx = gx0 + sgi * (gw / 3 + 2) - 2;
                const double segFill = clamp(L->flap * 3 - sgi, 0.0, 1.0);
                drawPanelRect(rt, {sx, gy, gw / 3, gh},
                              theme::light ? sf::Color(205, 215, 228) : sf::Color(50, 58, 72),
                              sf::Color::Transparent, 0);
                if (segFill > 0)
                    drawPanelRect(rt, {sx, gy, (float)(gw / 3 * segFill), gh}, GREEN(),
                                  sf::Color::Transparent, 0);
            }
            x += 96;
        }
        std::snprintf(b1, sizeof(b1), "%.1f m", L->h); item(x, u8"高度", b1, INK()); x += 95;
        sf::Color nc = INK();
        if (c && L->n > 0.85 * c->nFail) nc = BADC();
        else if (c && L->n > 0.6 * c->nFail) nc = WARNC();
        std::snprintf(b1, sizeof(b1), "%.2f", L->n); item(x, u8"荷重", b1, nc); x += 90;
        drawWindArrow(rt, x, top + 20, prm, L->psi);
        std::snprintf(b2, sizeof(b2), "wind %.2f", prm.wspd);
        drawText(rt, font_, b2, x, top + 36, 10, INKSOFT(), 1);
    } else if (waiting) {
        drawText(rt, font_, u8"発進待機中…", W / 2, top + 16, 16, INK(), 1, true);
        drawWindArrow(rt, W / 2 + 160, top + 22, prm, 0);
        std::snprintf(b2, sizeof(b2), "wind %.2f", prm.wspd);
        drawText(rt, font_, b2, W / 2 + 160, top + 38, 10, INKSOFT(), 1);
    } else {
        const double dist = res.dist;
        std::snprintf(b1, sizeof(b1), "%.0f m", dist);
        item(W / 2 + 60, u8"飛行距離", b1, INK());
        drawWindArrow(rt, W / 2 + 175, top + 22, prm, 0);
        std::snprintf(b2, sizeof(b2), "wind %.2f", prm.wspd);
        drawText(rt, font_, b2, W / 2 + 175, top + 40, 10, INKSOFT(), 1);
        if (!finMsg.empty())
            drawText(rt, font_, finMsg, W / 2, top + 56, 14, INK(), 1, true);
    }
    return top;
}

void HUD::drawChart(sf::RenderTarget& rt, sf::FloatRect r,
                    const std::vector<ChartSeries>& series,
                    const std::string& xLabel, const std::string& yLabel,
                    bool yZero, const std::vector<ChartMark>& marks, double redBelowY) {
    drawPanelRect(rt, r, theme::panel(), theme::line(), 1);
    const float PL = 48, PR = 12, PT = 18, PB = 26;
    double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300;
    for (const auto& s : series)
        for (const auto& p : s.pts) {
            if (!std::isfinite(p.first) || !std::isfinite(p.second)) continue;
            x0 = std::min(x0, p.first); x1 = std::max(x1, p.first);
            y0 = std::min(y0, p.second); y1 = std::max(y1, p.second);
        }
    if (x0 > x1) return;
    if (yZero) y0 = std::min(0.0, y0);
    if (x1 - x0 < 1e-9) x1 = x0 + 1;
    if (y1 - y0 < 1e-9) y1 = y0 + 1;
    const double py = (y1 - y0) * 0.08;
    y1 += py;
    if (!(yZero && y0 >= 0)) y0 -= py;
    auto X = [&](double v) { return r.left + PL + (float)((v - x0) / (x1 - x0)) * (r.width - PL - PR); };
    auto Y = [&](double v) { return r.top + r.height - PB - (float)((v - y0) / (y1 - y0)) * (r.height - PT - PB); };
    // 危険域(失速速度帯など)
    if (redBelowY > y0 && redBelowY > -1e299) {
        const float yy = Y(std::min(redBelowY, y1));
        drawPanelRect(rt, {r.left + PL, yy, r.width - PL - PR, r.top + r.height - PB - yy},
                      sf::Color(192, 57, 43, 26), sf::Color::Transparent, 0);
    }
    char buf[32];
    auto fmt = [&](double v) {
        const double a = std::abs(v);
        if (a >= 10000) std::snprintf(buf, sizeof(buf), "%.0fk", v / 1000);
        else if (a >= 1000) std::snprintf(buf, sizeof(buf), "%.1fk", v / 1000);
        else if (a >= 100) std::snprintf(buf, sizeof(buf), "%.0f", v);
        else if (a >= 10) std::snprintf(buf, sizeof(buf), "%.1f", v);
        else std::snprintf(buf, sizeof(buf), "%.2f", v);
        return std::string(buf);
    };
    for (int i = 0; i <= 5; i++) {
        const double xv = x0 + (x1 - x0) * i / 5, yv = y0 + (y1 - y0) * i / 5;
        const sf::Color grid = theme::light ? sf::Color(0xDC, 0xE4, 0xEC) : sf::Color(60, 70, 86, 160);
        line(rt, X(xv), r.top + PT, X(xv), r.top + r.height - PB, grid, 1);
        line(rt, r.left + PL, Y(yv), r.left + r.width - PR, Y(yv), grid, 1);
        drawText(rt, font_, fmt(xv), X(xv), r.top + r.height - PB + 3, 9, INKSOFT(), 1);
        drawText(rt, font_, fmt(yv), r.left + PL - 4, Y(yv) - 5, 9, INKSOFT(), 2);
    }
    if (!yLabel.empty()) drawText(rt, font_, yLabel, r.left + 4, r.top + 2, 10, INK());
    if (!xLabel.empty()) drawText(rt, font_, xLabel, r.left + r.width - PR, r.top + r.height - 14, 10, INK(), 2);
    for (const auto& s : series) {
        if (s.pts.size() < 2) continue;
        for (size_t i = 0; i + 1 < s.pts.size(); i++) {
            if (s.dash && i % 2) continue;
            line(rt, X(s.pts[i].first), Y(s.pts[i].second),
                 X(s.pts[i + 1].first), Y(s.pts[i + 1].second), s.color, 1.8f);
        }
    }
    for (const auto& m : marks) {
        sf::CircleShape d(3.5f);
        d.setOrigin(3.5f, 3.5f);
        d.setPosition(X(m.x), Y(m.y));
        d.setFillColor(m.color);
        rt.draw(d);
        drawText(rt, font_, m.label, X(m.x) + 6, Y(m.y) - 14, 10, INK());
    }
    // 凡例
    float lx = r.left + PL + 6;
    for (const auto& s : series) {
        if (s.label.empty()) continue;
        line(rt, lx, r.top + 8, lx + 16, r.top + 8, s.color, 2);
        drawText(rt, font_, s.label, lx + 20, r.top + 2, 10, INK());
        lx += 26 + textWidth(font_, s.label, 10);
    }
}

void HUD::miniGraph(sf::RenderTarget& rt, sf::FloatRect r, const std::vector<double>& vals,
                    sf::Color color, double yMin, double refLine, bool hasRef) {
    if (vals.empty()) return;
    double lo = yMin, hi = -1e18;
    for (double v : vals) { lo = std::min(lo, v); hi = std::max(hi, v); }
    if (hasRef) { lo = std::min(lo, refLine); hi = std::max(hi, refLine); }
    if (hi - lo < 0.5) hi = lo + 1;
    double pad = (hi - lo) * 0.12; hi += pad; lo -= pad;
    if (lo < yMin) lo = yMin;
    const float gx = r.left + 26;
    auto X = [&](size_t i) { return gx + (float)i / std::max<size_t>(1, vals.size() - 1) * (r.width - 30); };
    auto Y = [&](double v) { return r.top + r.height - 6 - (float)((v - lo) / (hi - lo)) * (r.height - 12); };
    char buf[24];
    for (int i = 0; i <= 3; i++) {
        double v = lo + (hi - lo) * i / 3;
        line(rt, gx, Y(v), r.left + r.width - 4, Y(v), mono(38), 1);
        std::snprintf(buf, sizeof(buf), std::abs(v) < 10 ? "%.1f" : "%.0f", v);
        drawText(rt, font_, buf, gx - 3, Y(v) - 6, 9, mono(200), 2);
    }
    if (hasRef)
        line(rt, gx, Y(refLine), r.left + r.width - 4, Y(refLine), sf::Color(255, 90, 90, 150), 1.2f);
    sf::VertexArray va(sf::LineStrip, vals.size());
    for (size_t i = 0; i < vals.size(); i++) va[i] = sf::Vertex({X(i), Y(vals[i])}, color);
    rt.draw(va);
    sf::CircleShape dot(3);
    dot.setOrigin(3, 3);
    dot.setPosition(X(vals.size() - 1), Y(vals.back()));
    dot.setFillColor(color);
    rt.draw(dot);
}

void HUD::drawSidePanels(sf::RenderTarget& rt, float W, float H, const SimResult& res,
                         int courseLeg, const SimParams& prm) {
    if (res.out.empty()) return;
    // py=44: 右上の「設計に戻る」ボタン(y=10..38)と重ならない位置から開始
    const float panW = 300, px = W - panW - 10, py = 44;
    const float botY = H - 78;
    const float rowH = (botY - py - 8) / 2;
    auto box = [&](sf::FloatRect r) {
        drawPanelRect(rt, r, theme::panelSoft(), theme::line(), 1);
    };
    // 直近120サンプル
    const size_t n = res.out.size();
    const size_t i0 = n > 120 ? n - 120 : 0;
    std::vector<double> hs, vs;
    for (size_t i = i0; i < n; i++) { hs.push_back(res.out[i].h); vs.push_back(res.out[i].V); }
    const SimSample& cur = res.out.back();
    char buf[48];
    // 高度
    sf::FloatRect rAlt(px, py, panW, rowH);
    box(rAlt);
    drawText(rt, font_, "Altitude", rAlt.left + 10, rAlt.top + 6, 13, INK(), 0, true);
    std::snprintf(buf, sizeof(buf), "%.1f m", cur.h);
    drawText(rt, font_, buf, rAlt.left + panW - 10, rAlt.top + 4, 16, GREEN(), 2, true);
    miniGraph(rt, {rAlt.left + 8, rAlt.top + 26, panW - 16, rowH - 34}, hs, GREEN(), 0, 0, false);
    // GPS + 速度
    const float y2 = py + rowH + 8;
    sf::FloatRect rGps(px, y2, 130, rowH), rSpd(px + 138, y2, panW - 138, rowH);
    box(rGps); box(rSpd);
    drawText(rt, font_, "GPS", rGps.left + 8, rGps.top + 6, 13, INK(), 0, true);
    std::snprintf(buf, sizeof(buf), "%.1fkm", cur.path / 1000);
    drawText(rt, font_, buf, rGps.left + rGps.width - 8, rGps.top + 6, 12, GREEN(), 2, true);
    {   // GPS俯瞰マップ (JS drawGpsMap + 湖岸形状・コース目標)
        sf::FloatRect m(rGps.left + 6, rGps.top + 26, rGps.width - 12, rowH - 34);
        drawPanelRect(rt, m, sf::Color(40, 70, 100, 100), sf::Color::Transparent, 0);
        sf::Vector2f pf(m.left + 0.80f * m.width, m.top + 0.5f * m.height);
        sf::Vector2f north(m.left + 0.26f * m.width, m.top + 0.16f * m.height);
        const bool fuji = prm.site == "fujikawa";
        // コース座標(x前方, yl右) → マップ座標(前方基準点方向を基準の等方スケール)
        // 琵琶湖=北パイロン(11000)、富士川=コース前方4000mを基準
        const double PYLON = fuji ? 4000 : 11000;
        sf::Vector2f along((north.x - pf.x) / (float)PYLON, (north.y - pf.y) / (float)PYLON);
        float aLen = std::sqrt(along.x * along.x + along.y * along.y);
        const sf::Vector2f perp(-along.y / aLen, along.x / aLen);
        auto toMap = [&](double x, double yl) {
            return sf::Vector2f(pf.x + along.x * (float)x + perp.x * (float)yl * aLen,
                                pf.y + along.y * (float)x + perp.y * (float)yl * aLen);
        };
        sf::Vector2f south;
        if (!fuji) {
            // 南パイロン(9020, -6300)は上の等方変換だと枠外に出るため、
            // PF・北・南の3点が収まるよう一様スケール+平行移動で全体を補正する
            const sf::Vector2f s0 = toMap(9020, -6300);
            const float pad = 10;
            const float minX = std::min({pf.x, north.x, s0.x}), maxX = std::max({pf.x, north.x, s0.x});
            const float minY = std::min({pf.y, north.y, s0.y}), maxY = std::max({pf.y, north.y, s0.y});
            const float k = std::min({1.0f, (m.width - 2 * pad) / std::max(1.0f, maxX - minX),
                                      (m.height - 2 * pad) / std::max(1.0f, maxY - minY)});
            const sf::Vector2f c0((minX + maxX) / 2, (minY + maxY) / 2);
            const sf::Vector2f c1(m.left + m.width / 2, m.top + m.height / 2);
            pf = c1 + (pf - c0) * k;
            north = c1 + (north - c0) * k;
            along *= k; aLen *= k;
            // 南パイロンは実コース座標を同じ変換で投影(3D世界とズレない)
            south = toMap(9020, -6300);
        }
        auto clampM = [&](sf::Vector2f p) {
            return sf::Vector2f(std::max(m.left + 2, std::min(m.left + m.width - 2, p.x)),
                                std::max(m.top + 2, std::min(m.top + m.height - 2, p.y)));
        };
        // 湖岸/水域ライン
        {
            const auto& shore = fuji ? fujikawaWater() : biwaShore();
            sf::VertexArray va(sf::LineStrip, shore.size() + 1);
            for (size_t i = 0; i <= shore.size(); i++) {
                const auto& p = shore[i % shore.size()];
                va[i] = sf::Vertex(clampM(toMap(p.first, p.second)), sf::Color(180, 220, 255, 110));
            }
            rt.draw(va);
        }
        auto dot = [&](sf::Vector2f p, sf::Color c, float r) {
            sf::CircleShape d(r); d.setOrigin(r, r); d.setPosition(p); d.setFillColor(c); rt.draw(d);
        };
        if (!fuji) {
            line(rt, pf.x, pf.y, north.x, north.y, sf::Color(255, 255, 255, 100), 1.2f);
            line(rt, pf.x, pf.y, south.x, south.y, sf::Color(255, 255, 255, 70), 1.2f);
            // コースの現在目標を点滅リングで強調
            if (courseLeg == 0 || courseLeg == 1) {
                const sf::Vector2f tgt = courseLeg == 0 ? north : pf;
                sf::CircleShape ring(8);
                ring.setOrigin(8, 8); ring.setPosition(tgt);
                ring.setFillColor(sf::Color::Transparent);
                ring.setOutlineColor(sf::Color(0xFF, 0xD2, 0x3F, 220));
                ring.setOutlineThickness(2);
                rt.draw(ring);
            }
            dot(north, sf::Color(0xff, 0x5a, 0x5a), 4); dot(south, sf::Color(0x5a, 0xa0, 0xff), 4);
            drawText(rt, font_, u8"北", north.x, north.y - 16, 9, sf::Color(0xcc, 0xff, 0xee), 1);
            drawText(rt, font_, u8"南", south.x, south.y + 5, 9, sf::Color(0xcc, 0xff, 0xee), 1);
        }
        dot(pf, GREEN(), 4);
        drawText(rt, font_, fuji ? "HG" : "PF", pf.x + 8, pf.y - 5, 9, sf::Color(0xcc, 0xff, 0xee), 0);
        const sf::Vector2f me = clampM(toMap(cur.x, cur.yl));
        sf::CircleShape med(4);
        med.setOrigin(4, 4); med.setPosition(me);
        med.setFillColor(ACCENT()); med.setOutlineColor(sf::Color::White); med.setOutlineThickness(1);
        rt.draw(med);
    }
    drawText(rt, font_, "Speed", rSpd.left + 10, rSpd.top + 6, 13, INK(), 0, true);
    std::snprintf(buf, sizeof(buf), "%.1f m/s", cur.V);
    drawText(rt, font_, buf, rSpd.left + rSpd.width - 10, rSpd.top + 4, 16, GOLD(), 2, true);
    miniGraph(rt, {rSpd.left + 8, rSpd.top + 26, rSpd.width - 16, rowH - 34}, vs,
              GOLD(), 0, res.Vs, true);
}

void HUD::drawInstruments(sf::RenderTarget& rt, float W, float H,
                          const FlightState& L, const AircraftConstants& c,
                          const SimParams& prm, const SimResult& res) {
    // JSのviewBox 1000x600 を画面にスケール
    const float sx = W / 1000.f, sy = H / 600.f;
    auto TX = [&](float x) { return x * sx; };
    auto TY = [&](float y) { return y * sy; };
    char buf[64];
    const double V = L.V, h = L.h, gam = L.gam, phi = L.phi, n = L.n;
    const double Vs = res.Vs, nF = c.nFail, VNE = c.VNE;
    const bool stall = V < Vs;
    const float cx = TX(500), cy = TY(300), R = std::min(TX(130), TY(130) * 1.4f);
    // --- 人工水平儀 ---
    {
        const float rollDeg = (float)(-phi * 180 / PI);
        const float pitchPx = (float)(gam * 180 / PI * 3) * sy;
        sf::CircleShape disk(R, 48);
        disk.setOrigin(R, R); disk.setPosition(cx, cy);
        disk.setFillColor(sf::Color(58, 123, 213, 115));   // 空
        rt.draw(disk);
        // 地面(半円近似): 回転した矩形をクリップなしで水平線以下に描く
        sf::Transform tr;
        tr.rotate(rollDeg, cx, cy);
        const float hy = cy - pitchPx;
        sf::VertexArray ground(sf::TriangleStrip, 4);
        const float gTop = std::max(cy - R, std::min(cy + R, hy));
        ground[0] = sf::Vertex(tr.transformPoint({cx - R, gTop}), sf::Color(138, 109, 59, 128));
        ground[1] = sf::Vertex(tr.transformPoint({cx + R, gTop}), sf::Color(138, 109, 59, 128));
        ground[2] = sf::Vertex(tr.transformPoint({cx - R, cy + R}), sf::Color(138, 109, 59, 128));
        ground[3] = sf::Vertex(tr.transformPoint({cx + R, cy + R}), sf::Color(138, 109, 59, 128));
        rt.draw(ground);
        // 水平線+ピッチラダー
        auto rline = [&](float x0, float y0, float x1, float y1, sf::Color col, float w) {
            sf::Vector2f a = tr.transformPoint({x0, y0}), b = tr.transformPoint({x1, y1});
            line(rt, a.x, a.y, b.x, b.y, col, w);
        };
        if (std::abs(hy - cy) < R)
            rline(cx - R, hy, cx + R, hy, sf::Color::White, 2);
        for (int p = -30; p <= 30; p += 10) {
            if (p == 0) continue;
            const float yy = cy + p * 3 * sy - pitchPx;
            if (std::abs(yy - cy) > R * 0.92f) continue;
            const float w = p % 20 == 0 ? 44 * sx : 26 * sx;
            rline(cx - w, yy, cx + w, yy, sf::Color(GREEN().r, GREEN().g, GREEN().b, 180), 1.5f);
            std::snprintf(buf, sizeof(buf), "%+d", p);
            sf::Vector2f tp = tr.transformPoint({cx + w + 6, yy - 7});
            drawText(rt, font_, buf, tp.x, tp.y, 12, sf::Color(GREEN().r, GREEN().g, GREEN().b, 190));
        }
        // 外周リング
        sf::CircleShape ringO(R, 48);
        ringO.setOrigin(R, R); ringO.setPosition(cx, cy);
        ringO.setFillColor(sf::Color::Transparent);
        ringO.setOutlineColor(sf::Color(255, 255, 255, 216));
        ringO.setOutlineThickness(2);
        rt.draw(ringO);
        // 機体マーク(固定)
        sf::Color yellow(0xFF, 0xD2, 0x3F);
        line(rt, cx - 50 * sx, cy, cx - 22 * sx, cy, yellow, 3.5f);
        line(rt, cx - 22 * sx, cy, cx - 14 * sx, cy + 8 * sy, yellow, 3.5f);
        line(rt, cx - 14 * sx, cy + 8 * sy, cx - 6 * sx, cy, yellow, 3.5f);
        line(rt, cx - 6 * sx, cy, cx + 22 * sx, cy, yellow, 3.5f);
        sf::CircleShape cdot(3); cdot.setOrigin(3, 3); cdot.setPosition(cx, cy); cdot.setFillColor(yellow);
        rt.draw(cdot);
        // バンク指標
        sf::ConvexShape tri(3);
        sf::Transform tb; tb.rotate(rollDeg, cx, cy);
        tri.setPoint(0, tb.transformPoint({cx, cy - R + 2}));
        tri.setPoint(1, tb.transformPoint({cx - 7, cy - R + 16}));
        tri.setPoint(2, tb.transformPoint({cx + 7, cy - R + 16}));
        tri.setFillColor(yellow);
        rt.draw(tri);
        const double psiDeg = std::fmod(std::fmod(L.psi * 180 / PI, 360.0) + 360.0, 360.0);
        std::snprintf(buf, sizeof(buf), u8"バンク %.0f° / 方位 %.0f°", phi * 180 / PI, psiDeg);
        drawText(rt, font_, buf, cx, cy + R + 10, 13, INK(), 1);
        if (L.inThermal)
            drawText(rt, font_, u8"↑ 上昇気流 ↑", cx, cy - R - 30, 15, GREEN(), 1, true);
    }
    // --- 上部: 距離 ---
    drawPanelRect(rt, {TX(380), TY(18), TX(240), 34}, theme::panelSoft(), sf::Color::Transparent, 0);
    std::snprintf(buf, sizeof(buf), "%.0f m", L.path);
    drawText(rt, font_, buf, TX(500), TY(18) + 6, 20, INK(), 1, true);
    // --- 速度テープ(左) ---
    {
        const float tapeX = TX(235), tapeH = 240 * sy, tapeY = cy - tapeH / 2;
        drawPanelRect(rt, {tapeX - 58, tapeY, 58, tapeH}, theme::panelSoft(), sf::Color::Transparent, 0);
        for (int s = (int)std::floor(V - 6); s <= (int)std::ceil(V + 6); s++) {
            if (s < 0) continue;
            const float yy = cy + (float)(V - s) * 18 * sy;
            if (yy < tapeY || yy > tapeY + tapeH) continue;
            line(rt, tapeX - 8, yy, tapeX, yy, mono(230), 1.5f);
            if (s % 2 == 0) {
                std::snprintf(buf, sizeof(buf), "%d", s);
                drawText(rt, font_, buf, tapeX - 12, yy - 7, 13, INK(), 2);
            }
        }
        const float vsY = cy + (float)(V - Vs) * 18 * sy;
        if (vsY > tapeY && vsY < tapeY + tapeH)
            drawPanelRect(rt, {tapeX - 58, vsY, 6, tapeY + tapeH - vsY}, sf::Color(255, 77, 77, 128), sf::Color::Transparent, 0);
        const float vneY = cy + (float)(V - VNE) * 18 * sy;
        if (vneY > tapeY && vneY < tapeY + tapeH)
            drawPanelRect(rt, {tapeX - 58, tapeY, 6, vneY - tapeY}, sf::Color(255, 77, 77, 153), sf::Color::Transparent, 0);
        drawPanelRect(rt, {tapeX - 52, cy - 12, 38, 24}, stall ? sf::Color(255, 77, 77) : theme::panel(),
                      theme::line(), 1.5f);
        std::snprintf(buf, sizeof(buf), "%.1f", V);
        drawText(rt, font_, buf, tapeX - 33, cy - 10, 17, stall ? sf::Color::White : INK(), 1, true);
        drawText(rt, font_, u8"対気 m/s", tapeX - 29, tapeY - 18, 12, INKSOFT(), 1);
        if (stall) drawText(rt, font_, u8"失速!", tapeX - 29, tapeY + tapeH + 6, 14, sf::Color(255, 77, 77), 1, true);
    }
    // --- 高度テープ(右) ---
    {
        const float altX = TX(765), tapeH = 240 * sy, tapeY = cy - tapeH / 2;
        drawPanelRect(rt, {altX, tapeY, 58, tapeH}, theme::panelSoft(), sf::Color::Transparent, 0);
        for (int a = (int)std::floor(h - 6); a <= (int)std::ceil(h + 6); a++) {
            if (a < 0) continue;
            const float yy = cy + (float)(h - a) * 18 * sy;
            if (yy < tapeY || yy > tapeY + tapeH) continue;
            line(rt, altX, yy, altX + 8, yy, mono(230), 1.5f);
            if (a % 2 == 0) {
                std::snprintf(buf, sizeof(buf), "%d", a);
                drawText(rt, font_, buf, altX + 12, yy - 7, 13, INK());
            }
        }
        drawPanelRect(rt, {altX + 14, cy - 12, 38, 24}, theme::panel(), theme::line(), 1.5f);
        std::snprintf(buf, sizeof(buf), "%.1f", h);
        drawText(rt, font_, buf, altX + 33, cy - 10, 17, INK(), 1, true);
        drawText(rt, font_, u8"高度 m", altX + 29, tapeY - 18, 12, INKSOFT(), 1);
    }
    // --- 荷重メーター 右下 ---
    {
        const float gx = TX(820), gy = TY(430);
        drawPanelRect(rt, {gx, gy, 150, 58}, theme::panelSoft(), sf::Color::Transparent, 0);
        drawText(rt, font_, u8"荷重 n", gx + 10, gy + 6, 12, INKSOFT());
        sf::Color gc = n > 0.85 * nF ? BADC() : n > 0.6 * nF ? WARNC() : GREEN();
        std::snprintf(buf, sizeof(buf), "%.2f", n);
        drawText(rt, font_, buf, gx + 140, gy + 4, 16, gc, 2, true);
        drawPanelRect(rt, {gx + 10, gy + 30, 130, 14}, mono(38), sf::Color::Transparent, 0);
        const float gf = (float)std::max(0.0, std::min(1.0, n / nF));
        drawPanelRect(rt, {gx + 10, gy + 30, 130 * gf, 14}, gc, sf::Color::Transparent, 0);
        line(rt, gx + 10 + 130 / (float)nF, gy + 28, gx + 10 + 130 / (float)nF, gy + 46, mono(128), 1);
        std::snprintf(buf, sizeof(buf), u8"限界 %.2f", nF);
        drawText(rt, font_, buf, gx + 10, gy + 46, 10, INKSOFT());
    }
    // --- 出力/体力 左下 ---
    {
        const float px = TX(30), py = TY(430);
        const bool hasSta = L.auto_ || prm.stamina;
        drawPanelRect(rt, {px, py, 150, hasSta ? 74.f : 58.f}, theme::panelSoft(), sf::Color::Transparent, 0);
        drawText(rt, font_, prm.funPlane ? u8"スロットル" : u8"出力", px + 10, py + 6, 12, INKSOFT());
        if (prm.funPlane) std::snprintf(buf, sizeof(buf), u8"%.0f%%", L.Pnow / 350);
        else std::snprintf(buf, sizeof(buf), "%.0fW", L.Pnow);
        drawText(rt, font_, buf, px + 140, py + 4, 16, GREEN(), 2, true);
        drawPanelRect(rt, {px + 10, py + 30, 130, 14}, mono(38), sf::Color::Transparent, 0);
        const float pf = (float)std::max(0.0, std::min(1.0, L.Pnow / (prm.funPlane ? 35000.0 : 700.0)));
        drawPanelRect(rt, {px + 10, py + 30, 130 * pf, 14}, sf::Color(0x3F, 0xA7, 0xFF), sf::Color::Transparent, 0);
        if (hasSta) {
            // W'(無酸素バッテリー: 回復する)
            std::snprintf(buf, sizeof(buf), u8"W' %.1fkJ", L.wbal / 1000);
            drawText(rt, font_, buf, px + 10, py + 46, 10,
                     L.wbal < 3000 ? BADC() : INKSOFT());
            const float wf = (float)std::max(0.0, std::min(1.0, L.wbal / (prm.wcap * 1000)));
            drawPanelRect(rt, {px + 78, py + 48, 62, 6}, mono(38), sf::Color::Transparent, 0);
            drawPanelRect(rt, {px + 78, py + 48, 62 * wf, 6},
                          L.wbal < 3000 ? BADC() : GOLD(), sf::Color::Transparent, 0);
            // 持久力(グリコーゲン: 減る一方。枯渇で持続出力が落ちる)
            std::snprintf(buf, sizeof(buf), u8"持久 %.0f%%", L.gly * 100);
            drawText(rt, font_, buf, px + 10, py + 60, 10,
                     L.gly < 0.2 ? BADC() : INKSOFT());
            drawPanelRect(rt, {px + 78, py + 62, 62, 6}, mono(38), sf::Color::Transparent, 0);
            drawPanelRect(rt, {px + 78, py + 62, 62 * (float)L.gly, 6},
                          L.gly < 0.2 ? BADC() : GREEN(), sf::Color::Transparent, 0);
        } else {
            drawText(rt, font_, u8"手動操縦", px + 10, py + 46, 10, INKSOFT());
        }
    }
    // --- 操舵表示 下中央 ---
    {
        const float stX = TX(500), stY = TY(540), stR = 34 * std::min(sx, sy);
        drawPanelRect(rt, {stX - stR - 2, stY - stR - 2, stR * 2 + 4, stR * 2 + 4}, theme::panelSoft(), sf::Color::Transparent, 0);
        line(rt, stX - stR, stY, stX + stR, stY, mono(90), 0.8f);
        line(rt, stX, stY - stR, stX, stY + stR, mono(90), 0.8f);
        sf::CircleShape k(6);
        k.setOrigin(6, 6);
        k.setPosition(stX + (float)L.ail * stR, stY - (float)L.e * stR);
        k.setFillColor(GOLD());
        rt.draw(k);
        drawText(rt, font_, u8"操縦桿", stX, stY + stR + 4, 11, INKSOFT(), 1);
        // ラダーバー
        const float ry = stY + stR + 26;
        drawPanelRect(rt, {stX - 50, ry - 6, 100, 12}, theme::panelSoft(), sf::Color::Transparent, 0);
        line(rt, stX, ry - 7, stX, ry + 7, mono(110), 0.8f);
        const float rw = (float)L.rud * 48;
        drawPanelRect(rt, {rw < 0 ? stX + rw : stX, ry - 5, std::abs(rw), 10}, sf::Color(0x3F, 0xA7, 0xFF), sf::Color::Transparent, 0);
        drawText(rt, font_, u8"ラダー", stX - 58, ry - 6, 10, INKSOFT(), 2);
        // ブレーキ表示(地上でS/↓)
        if (L.brake > 0.01)
            drawText(rt, font_, u8"● ブレーキ", stX + 58, ry - 6, 11, sf::Color(0xFF, 0x5A, 0x5A), 0, true);
    }
}

void HUD::drawMetricsBar(sf::RenderTarget& rt, float W, float y, const AircraftParams& st, const Analysis& a) {
    struct Chip { std::string l, v, h; int s; };   // s: 0=neutral 1=ok 2=warn 3=bad
    char b[96];
    std::vector<Chip> chips;
    int sm = a.SM < 0 ? 3 : a.SM < 5 ? 2 : a.SM <= 18 ? 1 : 2;
    const char* smh = a.SM < 0 ? u8"不安定!座席を前へ/尾翼延長" : a.SM < 5 ? u8"やや敏感(5〜18%推奨)" : a.SM <= 18 ? u8"良好" : u8"安定すぎ・舵が重い";
    std::snprintf(b, sizeof(b), "%.1f %%MAC", a.SM);
    chips.push_back({u8"静的安定余裕 SM", b, smh, sm});
    std::snprintf(b, sizeof(b), "%.0f / %+.0f W", a.Preq, a.margin);
    chips.push_back({u8"所要パワー/余裕", b, a.margin < 0 ? u8"出力不足" : u8"巡航時", a.margin > 40 ? 1 : a.margin > 0 ? 2 : 3});
    std::snprintf(b, sizeof(b), "%.1f kg", a.W);
    char h2[48]; std::snprintf(h2, sizeof(h2), u8"機体 %.1f kg", a.wEmpty);
    chips.push_back({u8"全備重量", b, h2, a.wEmpty < 32 ? 1 : a.wEmpty < 40 ? 2 : 3});
    std::snprintf(b, sizeof(b), "%.1f m/s", a.V);
    chips.push_back({u8"巡航速度", b, u8"CL=1.0 / 7〜9が扱いやすい", (a.V >= 6.5 && a.V <= 9.5) ? 1 : 2});
    std::snprintf(b, sizeof(b), u8"%.1f m² / %.0f", a.S, a.AR);
    chips.push_back({u8"翼面積 / AR", b, u8"AR25+が主流", a.AR >= 20 ? 1 : 2});
    std::snprintf(b, sizeof(b), "%.2f", a.Vh);
    chips.push_back({u8"尾翼容積 Vh", b, u8"推奨 0.35〜0.70", (a.Vh >= 0.35 && a.Vh <= 0.7) ? 1 : 2});
    std::snprintf(b, sizeof(b), "%.4f", a.Vv);
    chips.push_back({u8"尾翼容積 Vv", b, u8"推奨 0.004〜0.020", (a.Vv >= 0.004 && a.Vv <= 0.02) ? 1 : 2});
    std::snprintf(b, sizeof(b), "%.2f", a.sparSF);
    chips.push_back({u8"桁 安全率(簡易)", b, u8"根元曲げ・楕円分布近似", a.sparSF >= 1.5 ? 1 : a.sparSF >= 1 ? 2 : 3});
    std::snprintf(b, sizeof(b), "%.2f / %.2f m", a.xCG, a.xNP);
    chips.push_back({u8"CG / NP", b, u8"機首からの距離", 0});
    (void)st;

    // 画面全幅の半透過グレー帯(左パネル廃止により340起点をやめ、左端12から詰める)
    drawPanelRect(rt, {0, y - 6, W, 64}, theme::panelSoft(), sf::Color::Transparent, 0);
    {
        sf::RectangleShape top({W, 1});
        top.setPosition(0, y - 6);
        top.setFillColor(theme::line());
        rt.draw(top);
    }
    float x = 12;
    for (const auto& ch : chips) {
        const float cw = 148;
        sf::Color edge = ch.s == 1 ? OKC() : ch.s == 2 ? WARNC() : ch.s == 3 ? BADC() : theme::textDim();
        sf::Color vc = ch.s == 1 ? OKC() : ch.s == 2 ? WARNC() : ch.s == 3 ? BADC() : theme::text();
        drawPanelRect(rt, {x, y, cw, 52}, sf::Color(30, 36, 46, 200), sf::Color::Transparent, 0);
        drawPanelRect(rt, {x, y, 4, 52}, edge, sf::Color::Transparent, 0);
        drawText(rt, font_, ch.l, x + 9, y + 3, 10, theme::textDim());
        drawText(rt, font_, ch.v, x + 9, y + 15, 15, vc, 0, true);
        drawText(rt, font_, ch.h, x + 9, y + 37, 9, theme::textDim());
        x += cw + 8;
    }
}

} // namespace bm
