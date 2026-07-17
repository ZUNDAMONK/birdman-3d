#pragma once
// 2D HUDオーバーレイ(SFML)。飛行データバー・サイドパネル・FPV計器・メトリクスバー。
#include <SFML/Graphics.hpp>
#include "core/Types.hpp"
#include "ui/Widgets.hpp"

namespace bm {

class HUD {
public:
    bool init();                        // フォント読込
    const sf::Font& font() const { return font_; }

    // 下部フライトデータバー(飛行中の数値行)。ボタンは呼び出し側が別途描画。
    // 戻り値: バー上端のy(ボタン配置用)
    float drawFlightBar(sf::RenderTarget& rt, float W, float H,
                        const FlightState* L, const AircraftConstants* c,
                        const SimParams& prm, const SimResult& res,
                        bool flying, bool waiting, const std::string& finMsg);

    // 右側パネル(高度/GPS/速度)。courseLeg: -1=コース無効, 0=北パイロンへ, 1=PFへ
    void drawSidePanels(sf::RenderTarget& rt, float W, float H, const SimResult& res,
                        int courseLeg, const SimParams& prm);

    // FPV計器(人工水平儀・速度/高度テープ・荷重/出力・操舵)
    void drawInstruments(sf::RenderTarget& rt, float W, float H,
                         const FlightState& L, const AircraftConstants& c,
                         const SimParams& prm, const SimResult& res);

    // 設計モードのメトリクスチップ(画面下部・ツールバー直上に描く)
    void drawMetricsBar(sf::RenderTarget& rt, float W, float y, const AircraftParams& st, const Analysis& a);

    // 機首相対の風向計(JS makeWindSvg)
    void drawWindArrow(sf::RenderTarget& rt, float cx, float cy, const SimParams& prm, double psi);

    // 汎用2Dチャート (JS drawChart)
    struct ChartSeries {
        std::vector<std::pair<double, double>> pts;
        sf::Color color;
        std::string label;
        bool dash = false;
    };
    struct ChartMark { double x, y; std::string label; sf::Color color; };
    void drawChart(sf::RenderTarget& rt, sf::FloatRect r,
                   const std::vector<ChartSeries>& series,
                   const std::string& xLabel, const std::string& yLabel,
                   bool yZero,
                   const std::vector<ChartMark>& marks = {},
                   double redBelowY = -1e300);

private:
    void miniGraph(sf::RenderTarget& rt, sf::FloatRect r, const std::vector<double>& vals,
                   sf::Color color, double yMin, double refLine, bool hasRef);
    sf::Font font_;
};

} // namespace bm
