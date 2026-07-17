#include "game/Audio.hpp"
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>
#include <algorithm>

namespace bm {

static const double PI = 3.14159265358979323846;
static const unsigned SR = 44100;

static sf::Int16 toI16(double v) {
    return (sf::Int16)std::lround(std::max(-1.0, std::min(1.0, v)) * 32000);
}

// 端でクロスフェードしてシームレスなループにする
static void loopFade(std::vector<sf::Int16>& s, int fadeN) {
    const int n = (int)s.size();
    for (int i = 0; i < fadeN && i < n / 2; i++) {
        const double t = (double)i / fadeN;
        s[i] = (sf::Int16)std::lround(s[i] * t + s[n - fadeN + i] * (1 - t));
    }
}

void Audio::init() {
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> uni(-1, 1);

    // ---- 風切り音: ローパスした有色ノイズ 2秒ループ ----
    {
        const int N = SR * 2;
        std::vector<sf::Int16> s(N);
        double lp = 0, lp2 = 0;
        for (int i = 0; i < N; i++) {
            lp += 0.10 * (uni(rng) - lp);      // 1段目: ざらつき
            lp2 += 0.03 * (lp - lp2);          // 2段目: ゴーという成分
            s[i] = toI16(lp2 * 6.0 + lp * 0.8);
        }
        loopFade(s, SR / 10);
        windBuf_.loadFromSamples(s.data(), N, 1, SR);
        wind_.setBuffer(windBuf_);
        wind_.setLoop(true);
        wind_.setVolume(0);
    }
    // ---- プロペラ音: 基準30Hzの倍音バズ 1秒ループ(整数周期でシームレス) ----
    {
        const int N = SR;   // 30Hz × 30周期 = 1.0s
        std::vector<sf::Int16> s(N);
        double lp = 0;
        for (int i = 0; i < N; i++) {
            const double t = (double)i / SR;
            double v = 0;
            for (int k = 1; k <= 6; k++)
                v += std::sin(2 * PI * PROP_F0 * k * t) / (k * 1.2);
            // ブレード通過のパルス感 + わずかなノイズ
            v += 0.35 * std::pow(std::max(0.0, std::sin(2 * PI * PROP_F0 * t)), 8.0);
            lp += 0.25 * (uni(rng) - lp);
            v += lp * 0.25;
            s[i] = toI16(v * 0.35);
        }
        propBuf_.loadFromSamples(s.data(), N, 1, SR);
        prop_.setBuffer(propBuf_);
        prop_.setLoop(true);
        prop_.setVolume(0);
    }
    // ---- 着水: ノイズバースト+減衰(こもっていく) ----
    {
        const int N = (int)(SR * 1.4);
        std::vector<sf::Int16> s(N);
        double lp = 0;
        for (int i = 0; i < N; i++) {
            const double t = (double)i / SR;
            const double env = std::exp(-t * 3.2) * (t < 0.02 ? t / 0.02 : 1.0);
            const double cutoff = 0.35 * std::exp(-t * 2.0) + 0.02;
            lp += cutoff * (uni(rng) - lp);
            // 泡のゴボゴボ感
            const double bub = 0.25 * std::sin(2 * PI * (90 + 40 * std::sin(t * 9)) * t) * std::exp(-t * 2.5);
            s[i] = toI16((lp * 2.2 + bub) * env);
        }
        splashBuf_.loadFromSamples(s.data(), N, 1, SR);
        splash_.setBuffer(splashBuf_);
        splash_.setVolume(90);
    }
    // ---- 桁折損: 鋭いクラック2連 ----
    {
        const int N = (int)(SR * 0.35);
        std::vector<sf::Int16> s(N);
        for (int i = 0; i < N; i++) {
            const double t = (double)i / SR;
            double v = uni(rng) * std::exp(-t * 40);
            if (t > 0.08) v += uni(rng) * std::exp(-(t - 0.08) * 55) * 0.8;
            s[i] = toI16(v * 0.95);
        }
        crackBuf_.loadFromSamples(s.data(), N, 1, SR);
        crack_.setBuffer(crackBuf_);
        crack_.setVolume(95);
    }
    // ---- 接地: 低いドスン+タイヤノイズ ----
    {
        const int N = (int)(SR * 0.3);
        std::vector<sf::Int16> s(N);
        double lp = 0;
        for (int i = 0; i < N; i++) {
            const double t = (double)i / SR;
            lp += 0.06 * (uni(rng) - lp);
            const double v = 0.8 * std::sin(2 * PI * 55 * t) * std::exp(-t * 14) + lp * 1.5 * std::exp(-t * 8);
            s[i] = toI16(v);
        }
        thudBuf_.loadFromSamples(s.data(), N, 1, SR);
        thud_.setBuffer(thudBuf_);
        thud_.setVolume(70);
    }
    ok_ = true;
}

void Audio::setEnabled(bool on) {
    enabled_ = on;
    if (!on && ok_) { wind_.setVolume(0); prop_.setVolume(0); }
}

void Audio::update(bool flying, double V, double rpm, int propBlades, bool ground) {
    if (!ok_ || !enabled_) return;
    if (flying) {
        if (wind_.getStatus() != sf::Sound::Playing) wind_.play();
        if (prop_.getStatus() != sf::Sound::Playing) prop_.play();
        // 風切り: 5m/sから聞こえ始め、速いほど大きく高く
        const double wv = std::max(0.0, std::min(1.0, (V - 3.5) / 11.0));
        wind_.setVolume((float)(wv * wv * 75));
        wind_.setPitch((float)(0.65 + V / 22.0));
        // プロペラ: ブレード通過周波数に合わせてピッチ、rpmで音量
        const double bpf = rpm / 60.0 * std::max(1, propBlades);
        const double pv = std::max(0.0, std::min(1.0, rpm / 250.0));
        prop_.setVolume((float)(pv * (ground ? 20 : 34)));
        prop_.setPitch((float)std::max(0.2, bpf / PROP_F0));
    } else {
        wind_.setVolume(std::max(0.f, wind_.getVolume() - 3));
        prop_.setVolume(std::max(0.f, prop_.getVolume() - 3));
        if (wind_.getVolume() <= 0.5f && wind_.getStatus() == sf::Sound::Playing) wind_.stop();
        if (prop_.getVolume() <= 0.5f && prop_.getStatus() == sf::Sound::Playing) prop_.stop();
    }
}

void Audio::onSplash() { if (ok_ && enabled_) splash_.play(); }
void Audio::onCrack() { if (ok_ && enabled_) crack_.play(); }
void Audio::onTouchdown() { if (ok_ && enabled_) thud_.play(); }

} // namespace bm
