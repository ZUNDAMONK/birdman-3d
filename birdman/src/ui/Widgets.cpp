#include "ui/Widgets.hpp"
#include "ui/Theme.hpp"
#include <cmath>
#include <algorithm>

namespace bm {

sf::String utf8(const std::string& s) {
    return sf::String::fromUtf8(s.begin(), s.end());
}

using namespace theme;

void drawPanelRect(sf::RenderTarget& rt, sf::FloatRect r, sf::Color fill, sf::Color outline, float thick) {
    sf::RectangleShape rs({r.width, r.height});
    rs.setPosition(r.left, r.top);
    rs.setFillColor(fill);
    rs.setOutlineColor(outline);
    rs.setOutlineThickness(thick);
    rt.draw(rs);
}

void drawText(sf::RenderTarget& rt, const sf::Font& font, const std::string& s,
              float x, float y, unsigned size, sf::Color color, int align, bool bold) {
    sf::Text t(utf8(s), font, size);
    t.setFillColor(color);
    if (bold) t.setStyle(sf::Text::Bold);
    sf::FloatRect b = t.getLocalBounds();
    float ox = align == 1 ? b.width / 2 : align == 2 ? b.width : 0;
    t.setPosition(std::round(x - ox), std::round(y));
    rt.draw(t);
}

float textWidth(const sf::Font& font, const std::string& s, unsigned size, bool bold) {
    sf::Text t(utf8(s), font, size);
    if (bold) t.setStyle(sf::Text::Bold);
    return t.getLocalBounds().width;
}

bool Button::handle(const sf::Event& ev, sf::Vector2f m) {
    if (!visible || !enabled) { hovered = false; return false; }
    const bool inside = rect.contains(m);
    if (ev.type == sf::Event::MouseMoved) { hovered = inside; return false; }
    if (ev.type == sf::Event::MouseButtonPressed && ev.mouseButton.button == sf::Mouse::Left && inside) {
        if (onClick) onClick();
        return true;
    }
    return false;
}

// アルファ係数(0〜255)を色に掛ける(ポップアップのフェードイン用)
static sf::Color mulA(sf::Color c, sf::Uint8 alphaMul) {
    c.a = (sf::Uint8)((int)c.a * alphaMul / 255);
    return c;
}

void Button::draw(sf::RenderTarget& rt, const sf::Font& font, sf::Uint8 alphaMul) const {
    if (!visible) return;
    sf::Color fill, fg, outline;
    switch (style) {
        case 1: fill = orange(); fg = sf::Color::White; outline = orange(); break;
        case 2: fill = blue(); fg = sf::Color::White; outline = blue(); break;
        case 3:
            fill = light ? sf::Color(255, 255, 255, 205) : sf::Color(40, 46, 58, 200);
            fg = text(); outline = line();
            break;
        default: fill = panelSoft(); fg = text(); outline = on ? blue() : line(); break;
    }
    if (style == 0 && on) fill = blueDim();
    if (!enabled) { fill.a = 120; fg.a = 120; }
    else if (hovered) {
        if (style == 0) outline = blue();
        else if (light)   // ライトでは加算で白飛びするため減算で沈ませる
            fill = sf::Color((sf::Uint8)std::max(0, fill.r - 20), (sf::Uint8)std::max(0, fill.g - 12),
                             (sf::Uint8)std::max(0, fill.b - 4), fill.a);
        else
            fill = sf::Color(std::min(255, fill.r + 25), std::min(255, fill.g + 25), std::min(255, fill.b + 25), fill.a);
    }
    drawPanelRect(rt, rect, mulA(fill, alphaMul), mulA(outline, alphaMul), on && style == 0 ? 2.f : 1.f);
    drawText(rt, font, label, rect.left + rect.width / 2,
             rect.top + rect.height / 2 - charSize * 0.68f, charSize, mulA(fg, alphaMul), 1, true);
}

void Slider::applyMouse(float mx) {
    const float x0 = rect.left, w = rect.width;
    double t = std::max(0.0, std::min(1.0, (double)(mx - x0) / w));
    double v = minV + (maxV - minV) * t;
    v = std::round(v / step) * step;
    v = std::max(minV, std::min(maxV, v));
    if (set) set(v);
    if (onChange) onChange();
}

bool Slider::handle(const sf::Event& ev, sf::Vector2f m) {
    if (!visible() || !enabled) { dragging = false; return false; }
    const sf::FloatRect track(rect.left, rect.top + 18, rect.width, rect.height - 18);
    if (ev.type == sf::Event::MouseButtonPressed && ev.mouseButton.button == sf::Mouse::Left && track.contains(m)) {
        dragging = true;
        applyMouse(m.x);
        return true;
    }
    if (ev.type == sf::Event::MouseMoved && dragging) { applyMouse(m.x); return true; }
    if (ev.type == sf::Event::MouseButtonReleased) dragging = false;
    return false;
}

void Slider::draw(sf::RenderTarget& rt, const sf::Font& font, sf::Uint8 alphaMul) const {
    if (!visible()) return;
    const sf::Uint8 dim = enabled ? 255 : 110;   // ロック中は薄く
    (void)dim;
    const double v = get ? get() : 0;
    char buf[48];
    const double shown = v * mul;
    const bool dec = step * mul < 1;
    std::snprintf(buf, sizeof(buf), dec ? "%.2f%s" : "%.0f%s", shown, unit.c_str());
    drawText(rt, font, label, rect.left, rect.top, 12, mulA(text(), alphaMul));
    // 値バッジ(青系地。「バーの色は青っぽい感じ」の要望に合わせる)
    const float bw = textWidth(font, buf, 12, true) + 12;
    drawPanelRect(rt, {rect.left + rect.width - bw, rect.top, bw, 16}, mulA(blueDim(), alphaMul), sf::Color::Transparent, 0);
    drawText(rt, font, buf, rect.left + rect.width - bw + 6, rect.top + 1, 12,
             mulA(light ? text() : sf::Color::White, alphaMul), 0, true);
    // トラック
    const float ty = rect.top + 24;
    drawPanelRect(rt, {rect.left, ty, rect.width, 5},
                  mulA(light ? sf::Color(205, 215, 228) : sf::Color(50, 58, 72), alphaMul), sf::Color::Transparent, 0);
    const float t = (float)((v - minV) / (maxV - minV));
    drawPanelRect(rt, {rect.left, ty, rect.width * std::max(0.f, std::min(1.f, t)), 5}, mulA(blue(), alphaMul), sf::Color::Transparent, 0);
    // つまみ
    sf::CircleShape knob(7);
    knob.setOrigin(7, 7);
    knob.setPosition(rect.left + rect.width * std::max(0.f, std::min(1.f, t)), ty + 2.5f);
    knob.setFillColor(mulA(sf::Color::White, alphaMul));
    knob.setOutlineColor(mulA(blue(), alphaMul));
    knob.setOutlineThickness(2);
    rt.draw(knob);
}

} // namespace bm
