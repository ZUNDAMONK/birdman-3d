#pragma once
// 永続UIウィジェット。
// 重要: JS版の「ボタンを毎フレームDOM再生成する」バグを踏まないため、
// ウィジェットはすべて生成後保持し、状態変化のみ反映する。
#include <SFML/Graphics.hpp>
#include <functional>
#include <string>
#include <vector>
#include <memory>

namespace bm {

sf::String utf8(const std::string& s);

struct Button {
    sf::FloatRect rect;
    std::string label;
    std::function<void()> onClick;
    bool visible = true, enabled = true, on = false;
    int style = 0;            // 0=白/枠, 1=アクセント(橙), 2=ダーク, 3=半透明グレー
    unsigned charSize = 13;
    bool hovered = false;

    bool handle(const sf::Event& ev, sf::Vector2f mouse);
    // alphaMul: 0〜255。ポップアップの開閉アニメでフェードさせるための全体アルファ係数
    void draw(sf::RenderTarget& rt, const sf::Font& font, sf::Uint8 alphaMul = 255) const;
};

struct Slider {
    sf::FloatRect rect;       // トラック領域(ラベル行含む高さ≈34px)
    std::string label, unit;
    double minV = 0, maxV = 1, step = 0.1, mul = 1;
    std::function<double()> get;
    std::function<void(double)> set;
    std::function<void()> onChange;
    std::function<bool()> visibleIf;   // nullなら常に表示
    bool dragging = false;
    bool enabled = true;               // false=表示のみ(大会中のロック等)

    bool visible() const { return !visibleIf || visibleIf(); }
    bool handle(const sf::Event& ev, sf::Vector2f mouse);
    void draw(sf::RenderTarget& rt, const sf::Font& font, sf::Uint8 alphaMul = 255) const;
private:
    void applyMouse(float mx);
};

// 角丸っぽい矩形+枠(SFMLの単純矩形で代用)
void drawPanelRect(sf::RenderTarget& rt, sf::FloatRect r, sf::Color fill, sf::Color outline, float thick = 1.f);

// テキスト描画ヘルパー align: 0=左 1=中央 2=右
void drawText(sf::RenderTarget& rt, const sf::Font& font, const std::string& s,
              float x, float y, unsigned size, sf::Color color, int align = 0, bool bold = false);
float textWidth(const sf::Font& font, const std::string& s, unsigned size, bool bold = false);

} // namespace bm
