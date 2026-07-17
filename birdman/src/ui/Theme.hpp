#pragma once
// 共通テーマ。ダーク(設計モード)/ライト(テストフライト)の2パレット。
// Game::drawUI() 冒頭で theme::light を設定し、UI各所は関数(panel()等)経由で
// 色を取得する(static const 参照束縛ではモード切替に追従しないため関数化)。
// DesignPanel/DesignTools/drawMetricsBar は設計モード専用なのでダーク定数直参照でもよい。
#include <SFML/Graphics.hpp>

namespace bm::theme {

inline bool light = false;   // false=ダーク(design) / true=ライト(flight)

// ---- ダークパレット(設計モード / 既定) ----
inline const sf::Color BG0(10, 13, 18);                 // 最深背景
inline const sf::Color PANEL(18, 22, 30, 235);          // ポップアップ/パネル地
inline const sf::Color PANEL_SOFT(24, 28, 38, 190);     // 半透過バー(グレー透過)
inline const sf::Color LINE(70, 84, 105, 160);          // 枠線
inline const sf::Color TEXT(230, 237, 245);             // 主文字
inline const sf::Color TEXT_DIM(150, 163, 180);         // 補助文字
inline const sf::Color BLUE(77, 163, 255);              // アクセント青(スライダー/選択/ハイライト)
inline const sf::Color BLUE_DIM(38, 70, 110);
inline const sf::Color ORANGE(232, 89, 12);             // 発進などの強調のみ残す
inline const sf::Color OK(0x35, 0xC9, 0x7E);
inline const sf::Color WARN(0xE5, 0xB8, 0x4B);
inline const sf::Color BAD(0xE4, 0x5B, 0x4B);

// ---- ライトパレット(テストフライト) ----
inline const sf::Color L_PANEL(250, 251, 253, 242);
inline const sf::Color L_PANEL_SOFT(255, 255, 255, 200);
inline const sf::Color L_LINE(199, 210, 222);
inline const sf::Color L_TEXT(27, 42, 74);              // 旧INK(濃紺)
inline const sf::Color L_TEXT_DIM(70, 88, 122);
inline const sf::Color L_BLUE(31, 108, 204);            // 白地で読める濃い青
inline const sf::Color L_BLUE_DIM(208, 226, 246);       // 選択地(淡青)
inline const sf::Color L_OK(0x1E, 0x7F, 0x4F);          // 白地向けに暗め
inline const sf::Color L_WARN(0xB8, 0x86, 0x0B);
inline const sf::Color L_BAD(0xC0, 0x39, 0x2B);

// ---- モード追従の色取得関数 ----
inline sf::Color panel()     { return light ? L_PANEL : PANEL; }
inline sf::Color panelSoft() { return light ? L_PANEL_SOFT : PANEL_SOFT; }
inline sf::Color line()      { return light ? L_LINE : LINE; }
inline sf::Color text()      { return light ? L_TEXT : TEXT; }
inline sf::Color textDim()   { return light ? L_TEXT_DIM : TEXT_DIM; }
inline sf::Color blue()      { return light ? L_BLUE : BLUE; }
inline sf::Color blueDim()   { return light ? L_BLUE_DIM : BLUE_DIM; }
inline sf::Color orange()    { return ORANGE; }
inline sf::Color ok()        { return light ? L_OK : OK; }
inline sf::Color warn()      { return light ? L_WARN : WARN; }
inline sf::Color bad()       { return light ? L_BAD : BAD; }

} // namespace bm::theme
