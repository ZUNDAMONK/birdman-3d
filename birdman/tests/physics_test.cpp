// 物理コア単体テスト
// JS版(Birdman3Dβ.html)と数値を比較するための出力を行う。
// ブラウザ側ではコンソールで analyze(st) / aeroPack() / stepSim を同条件で呼べば比較できる。
// 乱流(turb)とサーマルのノイズ項はランダムのため、turb=0・pjit=false・summer=false で決定的にする。
#include "core/Aircraft.hpp"
#include "core/Airframe.hpp"
#include "core/Physics.hpp"
#include "core/Weather.hpp"
#include "core/SiteConst.hpp"
#include "core/Material.hpp"
#include "core/DesignIO.hpp"
#include <cstdio>
#include <cmath>
#include <cassert>
#include <tuple>
#include <limits>

using namespace bm;

static int failures = 0;
static void check(bool ok, const char* msg) {
    if (!ok) { std::printf("  [FAIL] %s\n", msg); failures++; }
    else     { std::printf("  [ok]   %s\n", msg); }
}
static bool near(double a, double b, double tol) { return std::abs(a - b) <= tol; }

int main() {
    std::printf("=== Birdman3D physics core test ===\n\n");

    AircraftParams st;   // デフォルト機体(JSのst初期値と同一)
    SimParams prm;
    prm.turb = 0; prm.thermal = 0; prm.summer = false; prm.pjit = false;
    prm.wind = 0; prm.xwind = 0; prm.temp = 20;
    prm.terrainWind = false;

    // ---- analyze ----
    Analysis a = analyze(st);
    std::printf("[analyze] S=%.3f MAC=%.4f AR=%.3f\n", a.S, a.MAC, a.AR);
    std::printf("          wEmpty=%.3f kg  W=%.3f kg\n", a.wEmpty, a.W);
    std::printf("          xCG=%.4f xNP=%.4f SM=%.2f%%  Vh=%.3f Vv=%.5f\n", a.xCG, a.xNP, a.SM, a.Vh, a.Vv);
    std::printf("          V=%.3f m/s  Preq=%.2f W  margin=%.2f W  sparSF=%.3f\n\n", a.V, a.Preq, a.margin, a.sparSF);
    check(a.S > 15 && a.S < 30, "wing area plausible (15-30 m^2)");
    check(a.AR > 25 && a.AR < 45, "aspect ratio plausible");
    check(a.W > 80 && a.W < 110, "gross weight plausible (80-110 kg)");
    check(a.SM > 0, "statically stable (SM>0)");
    check(a.Preq < st.powerMax, "cruise power below pilot max");

    // ---- groundEffect(修正版Wieselberger) ----
    std::printf("\n[groundEffect] span=%.0f: h=0 -> %.4f, h=2 -> %.4f, h=10 -> %.4f, h=100 -> %.4f\n",
        st.span, groundEffect(0, st.span), groundEffect(2, st.span),
        groundEffect(10, st.span), groundEffect(100, st.span));
    check(near(groundEffect(0, st.span), (16.0*2/28)*(16.0*2/28)/(1+(16.0*2/28)*(16.0*2/28)), 1e-12), "groundEffect h=0 formula");
    check(groundEffect(0, st.span) < groundEffect(5, st.span), "ground effect reduces CDi near water");
    check(groundEffect(1000, st.span) > 0.99, "no effect at altitude");

    // ---- aeroPack ----
    AircraftConstants c = aeroPack(st, a, prm);
    std::printf("\n[aeroPack] rho=%.5f Vs=%.3f Vt=%.3f Vmp=%.3f Pmin=%.2f\n", c.rho, c.Vs, c.Vt, c.Vmp, c.Pmin);
    std::printf("           CD0=%.5f e=%.4f propEff=%.4f driveEff=%.3f nFail=%.3f VNE=%.2f\n\n",
        c.CD0, c.e, c.propEff, c.driveEff, c.nFail, c.VNE);
    check(c.Vs > 5 && c.Vs < 8, "stall speed plausible");
    check(c.nFail > 1.0, "spar takes at least 1G");
    check(c.Pmin < a.Preq * 1.05, "Pmin <= cruise power");

    // ---- Phase 0C-1: カスタム配置の質量・重心・慣性を飛行定数へ接続 ----
    const MassBreakdown referenceMass = aggregateMass(buildDefaultLayout(st, a));
    check(referenceMass.totalKg > 0 && referenceMass.I[0][0] > 0
          && referenceMass.I[1][1] > 0 && referenceMass.I[2][2] > 0,
          "default layout exposes finite mass properties");
    const AircraftConstants sameMass = aeroPackCustomMass(st, a, prm, referenceMass, referenceMass);
    check(near(sameMass.m, c.m, 1e-12) && near(sameMass.a.xCG, c.a.xCG, 1e-12),
          "custom path with default mass preserves mass and CG");
    check(near(sameMass.Ixx, c.Ixx, 1e-12) && near(sameMass.Iyy, c.Iyy, 1e-12)
          && near(sameMass.Izz, c.Izz, 1e-12),
          "custom path with default layout preserves legacy inertia");

    MassBreakdown movedMass = referenceMass;
    movedMass.totalKg += 8.0;
    movedMass.cg.z += 0.25;
    movedMass.I[0][0] += 11.0;  // 左右軸まわり -> pitch Iyy
    movedMass.I[1][1] += 22.0;  // 上下軸まわり -> yaw Izz
    movedMass.I[2][2] += 33.0;  // 前後軸まわり -> roll Ixx
    const AircraftConstants moved = aeroPackCustomMass(st, a, prm, movedMass, referenceMass);
    check(near(moved.m, movedMass.totalKg, 1e-12)
          && near(moved.a.xCG, movedMass.cg.z, 1e-12),
          "custom aggregate changes flight mass and longitudinal CG");
    check(moved.a.SM < c.a.SM && moved.a.Vh < c.a.Vh && moved.Vs > c.Vs,
          "aft/heavier custom layout changes stability and speed in expected direction");
    check(near(moved.Ixx, c.Ixx + 33.0, 1e-12)
          && near(moved.Iyy, c.Iyy + 11.0, 1e-12)
          && near(moved.Izz, c.Izz + 22.0, 1e-12),
          "airframe axes map to roll, pitch, and yaw inertia explicitly");

    MassBreakdown invalidMass = movedMass;
    invalidMass.totalKg = std::numeric_limits<double>::quiet_NaN();
    const AircraftConstants fallback = aeroPackCustomMass(st, a, prm, invalidMass, referenceMass);
    check(near(fallback.m, c.m, 1e-12) && near(fallback.a.xCG, c.a.xCG, 1e-12)
          && near(fallback.Ixx, c.Ixx, 1e-12),
          "invalid custom mass properties fall back to legacy constants");

    // ---- Phase 0C-2: 実配置を空力・操縦安定へ接続 ----
    const AirframeGraph standardLayout = buildDefaultLayout(st, a);
    const AeroLayoutProperties standardAero = aggregateAeroLayout(standardLayout);
    const AircraftConstants sameLayout = aeroPackCustomLayout(
        st, a, prm, referenceMass, referenceMass, standardAero);
    check(near(sameLayout.a.xAC, c.a.xAC, 1e-12)
          && near(sameLayout.a.xHT, c.a.xHT, 1e-12)
          && near(sameLayout.a.xVT, c.a.xVT, 1e-12)
          && near(sameLayout.CLg0, c.CLg0, 1e-12),
          "default aero layout preserves legacy geometry and incidence");
    check(near(sameLayout.Cma, c.Cma, 1e-12) && near(sameLayout.Cmq, c.Cmq, 1e-12)
          && near(sameLayout.Cnb, c.Cnb, 1e-12) && near(sameLayout.Cnr, c.Cnr, 1e-12),
          "default aero layout preserves legacy stability derivatives");

    AirframeGraph aftHTail = standardLayout;
    Mount hMount = aftHTail.find("tail.h")->mount;
    hMount.offset.pos.z = 0.8;
    check(aftHTail.setMount("tail.h", hMount), "physics fixture moves horizontal tail");
    const AircraftConstants aftH = aeroPackCustomLayout(
        st, a, prm, referenceMass, referenceMass, aggregateAeroLayout(aftHTail));
    check(aftH.a.xHT > c.a.xHT && aftH.a.Vh > c.a.Vh && aftH.a.SM > c.a.SM
          && aftH.Cmq < c.Cmq,
          "aft horizontal tail increases volume, static stability, and pitch damping");

    AirframeGraph twinVTail = standardLayout;
    Mount vMount = twinVTail.find("tail.v")->mount;
    vMount.parentId = "tail.h"; vMount.hardpointId = "hp.tip";
    vMount.mirror = MirrorMode::Pair;
    check(twinVTail.setMount("tail.v", vMount), "physics fixture mounts twin fins");
    const AircraftConstants twinV = aeroPackCustomLayout(
        st, a, prm, referenceMass, referenceMass, aggregateAeroLayout(twinVTail));
    check(near(twinV.a.Sv, 2.0 * a.Sv, 1e-12) && twinV.Cnb > c.Cnb
          && twinV.Cnr < c.Cnr,
          "twin fins increase effective area and directional stability");

    AirframeGraph flatVTail = standardLayout;
    vMount = flatVTail.find("tail.v")->mount;
    vMount.offset.rotDeg.z = 90.0;
    check(flatVTail.setMount("tail.v", vMount), "physics fixture tilts fin horizontal");
    const AircraftConstants flatV = aeroPackCustomLayout(
        st, a, prm, referenceMass, referenceMass, aggregateAeroLayout(flatVTail));
    check(flatV.a.Sv < 1e-12 && std::abs(flatV.Cnb) < 1e-12,
          "horizontal fin loses vertical projected area and weathercock stability");

    AirframeGraph movedWing = standardLayout;
    Mount wingMount = movedWing.find("wing.main")->mount;
    wingMount.offset.pos.z = 0.5;
    wingMount.offset.rotDeg.x = 2.0;
    check(movedWing.setMount("wing.main", wingMount), "physics fixture moves and rotates wing");
    const AircraftConstants shiftedWing = aeroPackCustomLayout(
        st, a, prm, referenceMass, referenceMass, aggregateAeroLayout(movedWing));
    check(shiftedWing.a.xAC > c.a.xAC && shiftedWing.a.SM > c.a.SM
          && shiftedWing.CLg0 > c.CLg0,
          "wing position and incidence feed static margin and ground lift");

    AeroLayoutProperties invalidAero;
    const AircraftConstants aeroFallback = aeroPackCustomLayout(
        st, a, prm, referenceMass, referenceMass, invalidAero);
    check(near(aeroFallback.a.xAC, c.a.xAC, 1e-12)
          && near(aeroFallback.Cnb, c.Cnb, 1e-12),
          "invalid aero layout falls back to legacy constants");
    invalidAero.valid = true;
    invalidAero.wingLEZ = std::numeric_limits<double>::quiet_NaN();
    const AircraftConstants nanAeroFallback = aeroPackCustomLayout(
        st, a, prm, referenceMass, referenceMass, invalidAero);
    check(near(nanAeroFallback.a.xAC, c.a.xAC, 1e-12)
          && near(nanAeroFallback.Cnb, c.Cnb, 1e-12),
          "non-finite aero layout fields fall back even when marked valid");

    // ---- computePolar ----
    PolarResult pol = computePolar(st, a, prm);
    std::printf("[polar] Vs=%.3f  minP=%.1fW @%.1fm/s  L/Dmax=%.2f @%.1fm/s  Dp=%.2fN Di=%.2fN\n\n",
        pol.Vs, pol.mpY, pol.mpX, pol.bgY, pol.bgX, pol.Dp, pol.Di);
    check(pol.bgY > 20, "L/D max above 20");

    // ---- 決定的フライト(プラットフォーム発進, 270W, 乱流ゼロ) ----
    std::printf("[flight] platform launch, P=270W, no wind/turb (deterministic)\n");
    FlightState L = makeInitialState(c, prm, 0);
    std::printf("  t=0    x=%7.1f h=%6.2f V=%6.3f\n", L.x, L.h, L.V);
    double maxH = L.h, minV = L.V;
    int steps = 0;
    while (!L.done && L.t < 600) {
        stepSim(L, c, prm, 0.02);
        maxH = std::max(maxH, L.h); minV = std::min(minV, L.V);
        steps++;
        if (steps % 1500 == 0)  // 30秒毎
            std::printf("  t=%-5.0f x=%7.1f h=%6.2f V=%6.3f gam=%+.4f n=%.3f path=%.0f\n",
                L.t, L.x, L.h, L.V, L.gam, L.n, L.officialDist);
    }
    std::printf("  end: t=%.1f official=%.1fm pathAir=%.1fm splash=%d broken=%d %s\n",
        L.t, L.officialDist, L.pathAir, (int)L.splash, (int)L.sparBroken, L.failureMsg.c_str());
    check(L.officialDist > 3000, "default aircraft with 270W flies >3km");
    check(!L.sparBroken, "no structural failure in calm cruise");
    check(std::abs(L.officialDist - L.pathAir) / std::max(1.0, L.pathAir) < 0.02,
          "calm straight flight official distance remains within 2% of legacy air path");
    constexpr double PHASE3_STD_CRUISE_600M = 5542.6;
    check(std::abs(L.officialDist / PHASE3_STD_CRUISE_600M - 1.0) <= 0.02,
          "phase4: standard default cruise distance stays within 2% of Phase 3 baseline");
    check(minV > c.Vs * 0.8, "never deep-stalled");

    // ---- オート(性能計測)フライト ----
    SimParams prmA = prm; prmA.auto_ = true;
    AircraftConstants cA = aeroPack(st, a, prmA);
    FlightState LA = makeInitialState(cA, prmA, 0);
    while (!LA.done && LA.t < 3700) stepSim(LA, cA, prmA, 0.02);
    std::printf("\n[auto] path=%.0fm t=%.0fs done(broken=%d splash=%d)\n",
                LA.officialDist, LA.t, (int)LA.sparBroken, (int)LA.splash);
    check(LA.officialDist > 5000, "autopilot flies >5km");

    // ---- 天候システム ----
    SimParams ps; ps.summer = true; ps.weather = 4; // humid
    ps.tod = 12;
    applySummer(ps);
    std::printf("\n[weather humid @12:00] temp=%.0f wspd=%.2f turb=%.2f thermal=%.2f gust=%.2f wdir=%.0f\n",
        ps.temp, ps.wspd, ps.turb, ps.thermal, ps.gust, ps.wdir);
    check(near(biwaWeather()[4].therm, 0.6, 1e-12), "humid therm coefficient = 0.6 (fixed)");
    check(ps.thermal > 0.2, "midday humid thermal active");
    double pr = 0; for (auto& w : biwaWeather()) pr += w.prob;
    check(near(pr, 1.0, 1e-9), "weather probabilities sum to 1");

    // ---- 拡張物理(回転モデル) ----
    {
        SimParams p6 = prm; p6.sixdof = true;
        AircraftConstants c6 = aeroPack(st, a, p6);
        std::printf("\n[6DOF] Iyy=%.0f Ixx=%.0f  Cma=%.3f Cmq=%.2f Cmde=%.3f  Cnb=%.4f Clb=%.4f\n",
                    c6.Iyy, c6.Ixx, c6.Cma, c6.Cmq, c6.Cmde, c6.Cnb, c6.Clb);
        check(c6.Cma < 0, "6DOF: statically stable in pitch (Cma<0)");
        check(c6.Cmq < 0 && c6.Cnr < 0 && c6.Clp < 0, "6DOF: all dampings negative");
        // 巡航: 標準モデルと同等の飛行が成立するか
        FlightState L6 = makeInitialState(c6, p6, 0);
        double maxAl = 0, minV6 = 99;
        while (!L6.done && L6.t < 300) {
            stepSim6(L6, c6, p6, 0.02);
            if (L6.t > 20) { maxAl = std::max(maxAl, std::abs(L6.alpha)); minV6 = std::min(minV6, L6.V); }
        }
        std::printf("      cruise300s: path=%.0fm h=%.2f V=%.2f alpha=%.4f theta=%.4f (maxAl=%.3f)\n",
                    L6.officialDist, L6.h, L6.V, L6.alpha, L6.theta, maxAl);
        check(!L6.done, "6DOF: sustains flight for 300s at 270W");
        check(maxAl < 0.25, "6DOF: AoA stays bounded in cruise");
        check(minV6 > c6.Vs * 0.75, "6DOF: no deep stall in cruise");
        // 3DOFとの整合: 300秒の距離が±25%以内
        FlightState L3 = makeInitialState(c, prm, 0);
        while (!L3.done && L3.t < 300) stepSim(L3, c, prm, 0.02);
        std::printf("      dist 6DOF=%.0f vs 3DOF=%.0f (ratio %.2f)\n",
                    L6.officialDist, L3.officialDist, L6.officialDist / L3.officialDist);
        check(std::abs(L6.officialDist / L3.officialDist - 1) < 0.25,
              "6DOF cruise distance within 25%% of 3DOF");
        constexpr double PHASE3_ROT_CRUISE_300M = 2436.0;
        check(std::abs(L6.officialDist / PHASE3_ROT_CRUISE_300M - 1.0) <= 0.02,
              "phase4: expanded default cruise distance stays within 2% of Phase 3 baseline");
        // エレベーターパルス応答: 減衰して戻る(SM>0の動的発現)
        FlightState Lp = makeInitialState(c6, p6, 0);
        while (Lp.t < 60) stepSim6(Lp, c6, p6, 0.02);
        Lp.eTgt = 0.6;
        while (Lp.t < 61) stepSim6(Lp, c6, p6, 0.02);
        Lp.eTgt = 0;
        double alAfter = 0;
        while (Lp.t < 90 && !Lp.done) {
            stepSim6(Lp, c6, p6, 0.02);
            if (Lp.t > 85) alAfter = std::max(alAfter, std::abs(Lp.alpha));
        }
        std::printf("      elevator pulse: alpha 25s after release = %.4f rad\n", alAfter);
        check(!Lp.done && alAfter < 0.06, "6DOF: pitch oscillation damps out (dynamic stability)");
        // ラダー→上反角効果で旋回(エルロンなし機)
        FlightState Lr = makeInitialState(c6, p6, 0);
        while (Lr.t < 30) stepSim6(Lr, c6, p6, 0.02);
        Lr.rudTgt = 1.0;
        while (Lr.t < 40 && !Lr.done) stepSim6(Lr, c6, p6, 0.02);
        std::printf("      rudder 10s: phi=%.3f rad psi=%.3f rad beta=%.3f\n", Lr.phi, Lr.psi, Lr.beta);
        check(Lr.phi > 0.03, "6DOF: rudder induces right bank via dihedral effect");
        check(Lr.psi > 0.02, "6DOF: aircraft turns right with right rudder");
        // オートでも飛べる
        SimParams pa6 = p6; pa6.auto_ = true;
        AircraftConstants ca6 = aeroPack(st, a, pa6);
        FlightState La6 = makeInitialState(ca6, pa6, 0);
        while (!La6.done && La6.t < 3650) stepSim6(La6, ca6, pa6, 0.02);
        std::printf("      autopilot 6DOF: path=%.0fm t=%.0fs\n", La6.officialDist, La6.t);
        check(La6.officialDist > 5000, "6DOF: autopilot flies >5km");
    }

    // ---- 発進挙動マトリクス回帰(6DOF失敗事例の恒久チェック) ----
    // 小翼・高失速速度機(最適化が3DOFオート評価で吐いた設計)の発進:
    //   V0=4  (Vs=10.3, デッキ加速込みでも約5.6m/s) → 3DOF/6DOFとも着水
    //   V0=10                                        → 3DOF/6DOFとも飛行継続
    // (V0=7はデッキ滑走加速+失速後マージン導入で3DOFのみ地面効果滑走で
    //  生還する境界事例になったため、確実着水の代表をV0=4に変更)
    {
        AircraftParams sm;
        sm.span = 22.5; sm.rootChord = 0.60; sm.tipChord = 0.20; sm.washout = 4.0;
        sm.ribPitch = 0.30; sm.plankTop = 20; sm.plankBot = 65; sm.propDia = 2.9;
        sm.seatX = 1.80; sm.tailArm = 3.3; sm.hSpan = 2.7; sm.hChord = 0.40;
        sm.vHeight = 2.2; sm.vChord = 0.50; sm.rootDia = 105; sm.tipDia = 50; sm.wingX = 1.45;
        Analysis am = analyze(sm);
        auto fly = [&](bool sixdof, double V0) {
            SimParams p = prm;
            p.sixdof = sixdof; p.V0 = V0;
            AircraftConstants cc = aeroPack(sm, am, p);
            FlightState L2 = makeInitialState(cc, p, 0);
            while (!L2.done && L2.t < 120) {
                if (sixdof) stepSim6(L2, cc, p, 0.02); else stepSim(L2, cc, p, 0.02);
            }
            return L2;
        };
        FlightState r37 = fly(false, 4), r67 = fly(true, 4);
        FlightState r310 = fly(false, 10), r610 = fly(true, 10);
        std::printf("\n[launch matrix small-wing Vs=10.3]\n");
        std::printf("      V0=4 : 3DOF x=%.0f splash=%d / 6DOF x=%.0f splash=%d\n",
                    r37.officialDist, (int)r37.splash, r67.officialDist, (int)r67.splash);
        std::printf("      V0=10: 3DOF x=%.0f splash=%d / 6DOF x=%.0f splash=%d\n",
                    r310.officialDist, (int)r310.splash, r610.officialDist, (int)r610.splash);
        check(r37.splash && r67.splash, "underspeed launch ditches in BOTH models (consistent physics)");
        check(!r310.splash && !r610.splash, "V0=10 launch flies in BOTH models");
        check(r610.officialDist > 1000, "6DOF small-wing cruises after proper launch");
        // デフォルト機は手動・無入力でも6DOFで飛び続ける(暗黙パイロット)
        SimParams pd = prm; pd.sixdof = true;
        AircraftConstants cd = aeroPack(st, a, pd);
        FlightState Ld = makeInitialState(cd, pd, 0);
        double minH = 99;
        while (!Ld.done && Ld.t < 120) {
            stepSim6(Ld, cd, pd, 0.02);
            if (Ld.t > 1) minH = std::min(minH, Ld.h);
        }
        std::printf("      default 6DOF hands-off: x=%.0f minH=%.2f\n", Ld.officialDist, minH);
        check(!Ld.splash && minH > 3, "default aircraft flies hands-off in 6DOF (implicit pilot)");
    }

    // ---- プラットフォームのデッキ滑走(3.5°/10m/後端10.6m→前縁10.0m) ----
    {
        AircraftConstants cc = aeroPack(st, a, prm);
        FlightState L2 = makeInitialState(cc, prm, 0);
        std::printf("\n[deck] init x=%.1f h=%.2f V=%.2f ground=%d\n", L2.x, L2.h, L2.V, (int)L2.ground);
        check(L2.ground && std::abs(L2.x + 10) < 1e-9, "platform starts at deck rear (x=-10)");
        check(std::abs(L2.h - 10.61) < 0.02, "deck rear height ~10.61m");
        const double V0i = L2.V;
        int s = 0;
        while (L2.ground && !L2.done && s++ < 2000) stepSim(L2, cc, prm, 0.02);
        std::printf("      edge: V=%.2f (from %.2f) h=%.2f gam=%.3f\n", L2.V, V0i, L2.h, L2.gam);
        check(!L2.ground && L2.V > V0i + 0.8, "deck run gains speed (gravity+thrust)");
        check(std::abs(L2.h - 10.0) < 0.05, "leaves deck at edge height 10.0m");
        // ブレーキで発進中止(出力を止めて低速のうちなら前縁までに停止できる。
        // 高速+ペダル全開のままでは止まれない=中止判断は早く、が物理で出る)
        SimParams pb = prm; pb.P = 0; pb.V0 = 5;
        AircraftConstants cb = aeroPack(st, a, pb);
        FlightState Lb = makeInitialState(cb, pb, 0);
        s = 0;
        while (Lb.ground && !Lb.done && Lb.V > 0.05 && s++ < 2000) { Lb.brake = 1; stepSim(Lb, cb, pb, 0.02); }
        std::printf("      brake abort (P=0,V0=5): x=%.2f V=%.2f ground=%d\n", Lb.x, Lb.V, (int)Lb.ground);
        check(Lb.ground && Lb.x < 0, "brake+power-off aborts launch before the edge");
    }

    // ---- 夏の琵琶湖: 彦根アメダス校正(7-8月平年値 平均2.5m/s・最多風向NW) ----
    // 第4弾(文献再設計): 知見(4)の3系統湖陸風をlocalWind()に実装し、岸近傍の風増強の
    // 分担をapplySummer()の全域場からlocalWind()側へ移した。このためwspd/wspdMeanは
    // 「場所非依存の背景場」になり、実際にPF(彦根側の岸近傍)で体感される風は
    // 「背景場+localWindの局所加算」の合計になる。合計が彦根アメダスの実測レンジ
    // (日中2.3-4.1m/s)に収まることを新たに検証する(旧: 背景場単体を2.3-4.1で検証していた)。
    {
        auto windAt = [&](double tod, int wxIdx) {
            SimParams p = prm;
            p.summer = true; p.weather = wxIdx; p.tod = tod;
            p.dirJit = p.tempJit = p.wspdJit = 0;
            applySummer(p);
            return p;
        };
        const SimParams morn = windAt(7.0, 1), noon = windAt(14.0, 1);
        const SimParams noonSwkaze = windAt(14.0, 2), noonHira = windAt(14.0, 3);
        std::printf("\n[amedas] normal: 07h %.1fm/s dir%.0f / 14h %.1fm/s dir%.0f | swkaze14h %.1f hira14h %.1f\n",
                    morn.wspd, morn.wdir, noon.wspd, noon.wdir, noonSwkaze.wspd, noonHira.wspd);
        check(morn.wspd > 0.5 && morn.wspd < 1.6, "amedas: calm morning 0.6-1.7 m/s");
        check(noon.wspd > 1.5 && noon.wspd < 2.6,
              "amedas: midday background field reduced (shore boost moved to localWind, 3rd redesign)");
        check(std::abs(noon.wdir - 315) < 25, "amedas: midday wind from NW (315deg)");
        check(std::abs(morn.wdir - 115) < 25, "amedas: morning land breeze from E-SE");
        check(noonHira.wspd < 6.0, "amedas: even hira stays below HPA cruise speed");
        // どの天候でも日中の風速 < 巡航速度-2 (前進できないハマりを防ぐ)
        AircraftParams st2;
        const double Vcr = analyze(st2).V;
        check(noonHira.wspd < Vcr - 2, "amedas: forward progress always possible");

        // PF(東岸沖50m。x=0,yl=0)での合計風(背景場+localWindの3系統加算)を
        // 彦根アメダス実測レンジ(日中2.3-4.1m/s)で校正する(タスクAの校正値)
        SimParams pfNoon = noon;
        double aw, ax, av;
        localWind(pfNoon, 0, 0, 2.0, aw, ax, av);
        const double totWind = pfNoon.wind + aw, totXwind = pfNoon.xwind + ax;
        const double totSpd = std::sqrt(totWind * totWind + totXwind * totXwind);
        std::printf("      PF total(bg+local) 14h: %.2f m/s (bg=%.2f local-add=%.2f,%.2f)\n",
                    totSpd, noon.wspd, aw, ax);
        check(totSpd > 2.3 && totSpd < 4.1,
              "amedas: PF total (background+SE-system lake breeze) stays in 2.3-4.1 m/s");

        // 同地点の夜間(陸風): localWindの3系統が逆位相(0.4倍)で陸風成分を加える
        SimParams pfNight = windAt(22.0, 1);
        double awN, axN, avN;
        localWind(pfNight, 0, 0, 2.0, awN, axN, avN);
        const double totNight = std::sqrt(std::pow(pfNight.wind + awN, 2) + std::pow(pfNight.xwind + axN, 2));
        std::printf("      PF total(bg+local) 22h(land breeze): %.2f m/s (local-add=%.2f,%.2f)\n",
                    totNight, awN, axN);
        check(totNight > 0.5 && totNight < 2.2, "amedas: PF night land breeze modest (bg+local, knowledge(3))");
    }

    // ---- ガスト構造: 瞬間風速は平均の周りを揺らぐ(常時最大瞬間風速にしない) ----
    {
        SimParams p = prm;
        p.summer = true; p.weather = 1; p.tod = 13.0;
        p.dirJit = p.tempJit = p.wspdJit = p.gustJit = 0;
        applySummer(p);
        resetWeatherState(p, 0x12345678u, true);
        const double mean0 = p.wspdMean;
        double sum = 0, mx = 0, mn = 1e9;
        const int NF = 60 * 120;   // 2分間
        for (int i = 0; i < NF; i++) {
            updateWeatherJitter(p, true, 1.0 / 60.0);
            sum += p.wspd;
            mx = std::max(mx, p.wspd);
            mn = std::min(mn, p.wspd);
        }
        const double avg = sum / NF;
        std::printf("\n[gust] mean=%.2f avg(2min)=%.2f max=%.2f min=%.2f (gustFactor=%.2f)\n",
                    mean0, avg, mx, mn, mx / std::max(0.1, mean0));
        check(std::abs(avg - mean0) < mean0 * 0.30, "gust: 2-min average tracks the mean");
        check(mx / std::max(0.1, mean0) > 1.15 && mx / std::max(0.1, mean0) < 2.1,
              "gust: gust factor 1.15-2.1 (not constant, not extreme)");
        check(mn < mean0 * 0.85, "gust: lulls exist below the mean");
    }

    // ---- Phase 2: 気象時間はfps/再生速度ではなくsim時間で決まる ----
    {
        struct WxStats { double tod, mean, var, maxVGust, temp; SimParams final; };
        auto runWeather = [&](int fps, int speed) {
            SimParams p = prm;
            p.summer = true; p.weather = 1; p.tod = 10.0; p.speed = speed;
            applySummer(p);
            resetWeatherState(p, 0x51A7E123u, true);
            const double launchTemp = p.temp;
            const int targetSteps = 600 * 50;            // sim時間10分、固定刻み0.02s
            int steps = 0;
            double acc = 0, sum = 0, sum2 = 0, maxVG = 0;
            while (steps < targetSteps) {
                acc += speed / (double)fps;
                while (acc + 1e-12 >= 0.02 && steps < targetSteps) {
                    acc -= 0.02;
                    updateWeatherJitter(p, true, 0.02);
                    sum += p.wspd; sum2 += p.wspd * p.wspd;
                    maxVG = std::max(maxVG, std::abs(p.verticalGust));
                    steps++;
                }
            }
            const double mean = sum / targetSteps;
            WxStats out{p.tod, mean, sum2 / targetSteps - mean * mean, maxVG, p.temp, p};
            check(near(p.temp, launchTemp, 1e-12), "weather: launch atmosphere stays fixed in flight");
            return out;
        };

        const WxStats baseWx = runWeather(60, 1);
        bool allSame = true;
        for (int fps : {30, 60, 120}) for (int speed : {1, 10, 40}) {
            const WxStats qx = runWeather(fps, speed);
            allSame = allSame && near(qx.tod, baseWx.tod, 1e-10)
                      && near(qx.mean, baseWx.mean, 1e-10)
                      && near(qx.var, baseWx.var, 1e-10)
                      && near(qx.maxVGust, baseWx.maxVGust, 1e-10);
        }
        std::printf("\n[weather dt] tod=%.3f mean=%.3f var=%.4f max|vertical gust|=%.2f\n",
                    baseWx.tod, baseWx.mean, baseWx.var, baseWx.maxVGust);
        check(allSame, "weather: 30/60/120fps and x1/x10/x40 share the same sim-time history");
        check(near(baseWx.tod, 10.0 + 600.0 * 0.0048, 1e-9),
              "weather: time-of-day advances from simulation seconds");
        check(baseWx.var > 0.01 && baseWx.maxVGust > 0.25,
              "weather: horizontal variability and short vertical gust events remain active");

        // SimParamsのコピーは独立したRNG状態を持ち、更新順に左右されず同じ環境を再生する。
        SimParams player = baseWx.final, rival = baseWx.final;
        bool shared = true;
        for (int i = 0; i < 1000; i++) {
            updateWeatherJitter(player, true, 0.02);
            updateWeatherJitter(rival, true, 0.02);
            shared = shared && near(player.wspd, rival.wspd, 1e-12)
                     && near(player.verticalGust, rival.verticalGust, 1e-12);
        }
        check(shared, "weather: copied player/rival environments replay the same seeded samples");

        // Dryden乱流もFlightStateごとのweatherSeed系列を使い、グローバル乱数や
        // プレイヤー/ライバルの呼び出し順に影響されない。
        SimParams dry = prm;
        dry.summer = false; dry.turb = 0.8; dry.thermal = 0;
        dry.terrainWind = false; dry.pjit = false; dry.stamina = false;
        dry.weatherSeed = 0x2468ACE1u;
        FlightState dryA = makeInitialState(c, dry, 0);
        FlightState dryB = makeInitialState(c, dry, 0);
        for (FlightState* L : {&dryA, &dryB}) {
            L->ground = false; L->h = 100; L->V = c.Vt; L->gam = 0;
            L->t = 10; L->liftoffT = -1e9;
        }
        bool drySame = true;
        for (int i = 0; i < 500; i++) {
            stepSim(dryA, c, dry, 0.02);
            frand(); frand(); frand();                    // 外部の乱数消費を模擬
            stepSim(dryB, c, dry, 0.02);
            drySame = drySame && near(dryA.tz, dryB.tz, 1e-12)
                      && near(dryA.ty, dryB.ty, 1e-12)
                      && near(dryA.x, dryB.x, 1e-10)
                      && near(dryA.h, dryB.h, 1e-10);
        }
        check(drySame, "weather: seeded Dryden flight is independent of global RNG/call order");

        FlightState dryBase = makeInitialState(c, dry, 0);
        dryBase.V = c.Vt; dryBase.h = 100;
        stepTurbulence(dryBase, dry, 0.02);
        SimParams dryOther = dry; dryOther.weatherSeed++;
        FlightState dryC = makeInitialState(c, dryOther, 0);
        dryC.V = c.Vt; dryC.h = 100;
        stepTurbulence(dryC, dryOther, 0.02);
        check(!near(dryBase.tz, dryC.tz, 1e-6),
              "weather: a different seed changes the Dryden sequence");
    }

    // ---- Phase 2: 垂直突風荷重を3DOF/6DOFで共通化 ----
    {
        auto gustLoad = [&](bool sixdof, double gustVz, double nFail) {
            SimParams p = prm;
            p.summer = false; p.turb = 0; p.thermal = 0; p.terrainWind = false;
            p.verticalGust = gustVz; p.P = 0;
            AircraftConstants cg = c;
            cg.nFail = nFail; cg.nFailNeg = -100; cg.VNE = 40;
            FlightState Lg = makeInitialState(cg, p, 0);
            Lg.ground = false; Lg.h = 100; Lg.V = 12; Lg.gam = 0; Lg.t = 10;
            Lg.liftoffT = -1e9;
            if (sixdof) {
                const double q = 0.5 * cg.rho * Lg.V * Lg.V;
                const double clTrim = cg.W / (q * cg.S);
                Lg.theta = (clTrim - cg.CLcruise) / cg.CLa;
                Lg.init6 = true;
                stepSim6(Lg, cg, p, 0.02);
            } else {
                stepSim(Lg, cg, p, 0.02);
            }
            return Lg;
        };

        const FlightState g30 = gustLoad(false, 0, 100);
        const FlightState g31 = gustLoad(false, 1, 100);
        const FlightState g33 = gustLoad(false, 3, 100);
        const FlightState g60 = gustLoad(true, 0, 100);
        const FlightState g61 = gustLoad(true, 1, 100);
        const FlightState g63 = gustLoad(true, 3, 100);
        const double d31 = g31.n - g30.n, d33 = g33.n - g30.n;
        const double d61 = g61.n - g60.n, d63 = g63.n - g60.n;
        std::printf("[vertical gust load] 3DOF n=%.2f/%.2f/%.2f  6DOF n=%.2f/%.2f/%.2f\n",
                    g30.n, g31.n, g33.n, g60.n, g61.n, g63.n);
        check(g31.n > g30.n && g33.n > g31.n, "3DOF: load rises monotonically at vertical gust 0/1/3m/s");
        check(g61.n > g60.n && g63.n > g61.n, "6DOF: load rises monotonically at vertical gust 0/1/3m/s");
        check(std::abs(d31 - d61) < 0.08 && std::abs(d33 - d63) < 0.08,
              "vertical gust: 3DOF/6DOF initial load increments agree");
        check(gustLoad(false, 3, 2.0).sparBroken && !gustLoad(false, 3, 10.0).sparBroken,
              "vertical gust: weak spar breaks while strong spar survives");
    }

    // ---- 実機キャリブレーション: MIT Daedalus 88 相当の構成 ----
    // 公表値: 翼幅34.1m 翼面積30.8m² 総重量約104kg 巡航6.5-7.5m/s 所要パワー200-230W
    // 解析がこの実績値の±20%に入ることを回帰チェックする
    {
        AircraftParams dd;
        dd.span = 34.1; dd.planform = "taper"; dd.rootChord = 1.15; dd.tipChord = 0.55;
        dd.airfoil = "dae31"; dd.washout = 2.5;
        dd.posture = "recumbent"; dd.pilotW = 68; dd.powerMax = 230;
        dd.propDia = 3.4; dd.propPitch = 4.2; dd.propConfig = "pylon";
        dd.tailArm = 5.5; dd.hSpan = 3.6; dd.hChord = 0.62;
        dd.rootDia = 120, dd.tipDia = 60; dd.gear = "none"; dd.fairing = true;
        Analysis ad = analyze(dd);
        SimParams pd2 = prm;
        AircraftConstants cd2 = aeroPack(dd, ad, pd2);
        std::printf("\n[calib Daedalus] S=%.1f W=%.1fkg V=%.2f Preq=%.0fW Vs=%.2f VNE=%.1f e=%.3f\n",
                    ad.S, ad.W, ad.V, ad.Preq, cd2.Vs, cd2.VNE, cd2.e);
        check(ad.S > 26 && ad.S < 34, "calib: wing area ~30m^2");
        check(ad.W > 88 && ad.W < 120, "calib: gross weight ~104kg (+-15%)");
        check(ad.Preq > 160 && ad.Preq < 280, "calib: cruise power 200-230W (+-20%)");
        check(ad.V > 5.5 && ad.V < 8.5, "calib: cruise speed 6.5-7.5 m/s (+-20%)");
        check(cd2.e > 0.85, "calib: high-AR tapered wing has high span efficiency");
    }

    // ---- フラッターVNE: 桁剛性依存 ----
    {
        AircraftParams s1, s2, s3;
        s2.rootDia = 90; s2.tipDia = 50;       // 細い桁
        s3.sparMat = "m40j";                    // 高弾性材
        SimParams pv = prm;
        const double v1 = aeroPack(s1, analyze(s1), pv).VNE;
        const double v2 = aeroPack(s2, analyze(s2), pv).VNE;
        const double v3 = aeroPack(s3, analyze(s3), pv).VNE;
        std::printf("\n[flutter VNE] default=%.1f thin-spar=%.1f high-modulus=%.1f m/s\n", v1, v2, v3);
        check(v1 > 13 && v1 < 20, "VNE: default spar ~16 m/s (matches original balance)");
        check(v2 < v1, "VNE: thinner spar lowers flutter limit");
        check(v3 > v1, "VNE: higher modulus material raises flutter limit");
        check(v2 >= 1.24 * analyze(s2).V, "VNE: never below 1.25x cruise (floor)");
    }

    // ---- 琵琶湖地形・地形風 ----
    {
        std::printf("\n[terrain]\n");
        check(insideLake(11000, 0), "north pylon is on the lake");
        check(insideLake(9020, -6300), "south pylon is on the lake");
        check(insideLake(500, 0), "course ahead of PF is water");
        check(!insideLake(-1000, 0), "runway behind PF is land");
        check(!insideLake(5000, 3600), "east of east-shore is land");
        double aw, ax, av;
        SimParams tp; tp.weather = 3; tp.wspd = 5; tp.thermal = 0.5;
        localWind(tp, 8000, -9000, 10, aw, ax, av);
        std::printf("      hira west zone: addVz=%.2f addXwind=%.2f\n", av, ax);
        check(av < -0.3, "hira-oroshi downdraft on west side");
        check(ax > 0.2, "hira-oroshi pushes east");
        localWind(tp, 5000, 2500, 10, aw, ax, av);
        std::printf("      east shore: addVz=%.2f\n", av);
        check(av > 0.1, "shore thermal updraft near east shore");
        localWind(tp, 5000, 0, 10, aw, ax, av);
        std::printf("      mid-course (x=5000,yl=0): addWind=%.2f addXwind=%.2f addVz=%.2f\n", aw, ax, av);
        // 第4弾以前は「コース中心線(yl=0)は地形風ゼロ」だったが、知見(4)の3系統湖陸風・
        // 知見(1)の湖面熱容量差は湖上のどこでも(コース中心線上でも)働くため、
        // 比良おろし/岸サーマルのような閾値式の地物風とは異なり、ここも完全ゼロではなくなった。
        // 校正上は「岸際(SE系)や西岸(比良おろし)ほど強くない小さな値」であることのみ確認する。
        check(std::abs(aw) < 1.0 && std::abs(ax) < 1.0 && std::abs(av) < 0.5,
              "mid-course terrain wind is small (weaker than shore, but no longer exactly zero: 3-system + lake thermal apply lake-wide)");
    }

    // ---- 第4弾 検証: PF付近/湖心の実効風(6/10/14/19時)・チャネリング ----
    {
        std::printf("\n[biwa v4 verification]\n");
        auto effWind = [&](double tod, double x, double yl) {
            SimParams p = prm;
            p.summer = true; p.weather = 1; p.tod = tod;    // "湖風日和" normal
            p.dirJit = p.tempJit = p.wspdJit = 0;
            applySummer(p);
            double aw, ax, av;
            localWind(p, x, yl, 2.0, aw, ax, av);
            const double W = p.wind + aw, X = p.xwind + ax;
            const double spd = std::sqrt(W * W + X * X);
            const double fromDeg = std::fmod(std::atan2(-X, -W) * 180.0 / 3.14159265358979323846 + 360.0, 360.0);
            return std::make_tuple(spd, fromDeg, aw, ax, av);
        };
        std::printf("  地点            時刻   実効風速  風向(from)  local加算(w,x)  addVz\n");
        for (double tod : {6.0, 10.0, 14.0, 19.0}) {
            auto [spd, dir, aw, ax, av] = effWind(tod, 0, 0);
            std::printf("  PF(0,0)         %02.0f時  %5.2fm/s   %5.1f°     (%+.2f,%+.2f)      %+.2f\n",
                        tod, spd, dir, aw, ax, av);
        }
        for (double tod : {6.0, 10.0, 14.0, 19.0}) {
            auto [spd, dir, aw, ax, av] = effWind(tod, 8000, -3000);
            std::printf("  湖心(8000,-3000) %02.0f時  %5.2fm/s   %5.1f°     (%+.2f,%+.2f)      %+.2f\n",
                        tod, spd, dir, aw, ax, av);
        }
        // 知見(3)夜間湖上増幅の倍率そのものを検証する(背景風の日変化と交絡しないよう、
        // prm.wind/xwindを同一値に固定した上でtodだけを変えてlocalWindの加算量を比較する)
        {
            SimParams pDay = prm; pDay.summer = true; pDay.site = "biwa";
            pDay.wind = 2.0; pDay.xwind = 0.0; pDay.tod = 14.0;
            SimParams pNight = pDay; pNight.tod = 22.0;
            double awD, axD, avD, awN, axN, avN;
            localWind(pDay, 8000, -3000, 2.0, awD, axD, avD);
            localWind(pNight, 8000, -3000, 2.0, awN, axN, avN);
            std::printf("  湖心 夜間増幅チェック(背景風2.0m/s固定): 14時add=%.3f  22時add=%.3f\n", awD, awN);
            check(awN > awD, "knowledge(3): night amplification multiplier is larger than midday (isolated background wind)");
        }
        // 強風時の地形チャネリング(知見5)
        {
            SimParams ph = prm; ph.summer = true; ph.tod = 14.0; ph.weather = 3;  // hira
            ph.dirJit = ph.tempJit = ph.wspdJit = ph.gustJit = 0;
            applySummer(ph);
            std::printf("  wspdMean=%.1f hira時 wdir=%.0f (should channel toward 315)\n", ph.wspdMean, ph.wdir);
            check(std::abs(ph.wdir - 315) < 15, "channeling: hira wind directs toward NW terrain axis (315deg)");
            SimParams ps = prm; ps.summer = true; ps.tod = 14.0; ps.weather = 2;  // swkaze
            ps.dirJit = ps.tempJit = ps.wspdJit = ps.gustJit = 0;
            applySummer(ps);
            std::printf("  wspdMean=%.1f swkaze時 wdir=%.0f (should channel toward SSW 205)\n", ps.wspdMean, ps.wdir);
            check(std::abs(ps.wdir - 205) < 15, "channeling: swkaze wind directs toward SSW lake-axis (205deg)");

            // タスク指定の代表値 wspdMean=5 での理論ブレンド結果(applySummer内のチャネリング式
            // w=min(1,(wspdMean-3.5)/3), 最寄り軸ブレンドと同一の式で参照計算し、報告用に出力する)
            const double wspdMeanRef = 5.0, wdirStart = 250.0;   // 250°=どの軸からもやや離れた仮の風向
            const double wRef = clamp((wspdMeanRef - 3.5) / 3.0, 0.0, 1.0);
            static const double axesRef[3] = {25.0, 205.0, 315.0};
            double bestRef = 1e9, targetRef = 315.0;
            for (double axr : axesRef) {
                const double d = std::abs(std::fmod(wdirStart - axr + 540.0, 360.0) - 180.0);
                if (d < bestRef) { bestRef = d; targetRef = axr; }
            }
            const double diffRef = std::fmod(targetRef - wdirStart + 540.0, 360.0) - 180.0;
            const double wdirBlended = std::fmod(wdirStart + diffRef * wRef + 360.0, 360.0);
            std::printf("  [参考] wspdMean=5.0, 元wdir=250度 -> 最寄り軸=%.0f度, blend weight=%.2f -> 結果wdir=%.1f度\n",
                        targetRef, wRef, wdirBlended);
            check(near(wRef, 0.5, 1e-9), "channeling: blend weight formula gives w=0.5 at wspdMean=5");
        }
    }

    // ---- 富士川滑空場: 水域判定・風・河原サーマル ----
    {
        SimParams pf = prm;
        pf.site = "fujikawa";
        std::printf("\n[fujikawa]\n");
        check(insideWaterSite(pf, 3000, 0), "fujikawa: sea south of the coast");
        // 第8弾: 川を滑走路の東側近接(yl≈-50〜-160)へ移設したため水域判定点を更新
        check(insideWaterSite(pf, -2000, -100), "fujikawa: Fuji river ribbon is water");
        check(!insideWaterSite(pf, -500, -300), "fujikawa: runway riverbed is land");
        check(!insideWaterSite(pf, -500, 3000), "fujikawa: western plain is land");
        check(!insideWaterSite(pf, -120, -160), "fujikawa: visible upstream sandbar is land");
        check(insideWaterSite(pf, -500, -100), "fujikawa: river point outside sandbars is water");
        check(!insideWaterSite(pf, -400, -40), "fujikawa: near-bank inside is land");
        check(!insideWaterSite(pf, 950, -320), "fujikawa: mouth sandbar is land");
        check(insideWaterSite(pf, 1050, -320), "fujikawa: water around mouth sandbar stays water");
        check(site::insideFujikawaRunway(-860, 15) && site::insideFujikawaRunway(-10, -15),
              "fujikawa: shared 850x30m runway bounds include both edges");
        check(!site::insideFujikawaRunway(-861, 0) && !site::insideFujikawaRunway(-100, 15.1),
              "fujikawa: shared runway bounds reject outside points");
        SimParams pb = prm;
        check(insideWaterSite(pb, 500, 0) == insideLake(500, 0), "site=biwa keeps insideLake behavior");
        // 風向: 早朝=陸風(追い風≈185)、日中=海風(向かい風≈25)
        pf.summer = true; pf.weather = 1; pf.dirJit = pf.tempJit = pf.wspdJit = pf.gustJit = 0;
        pf.tod = 7.0; applySummer(pf);
        const double dMorn = pf.wdir;
        pf.tod = 14.0; applySummer(pf);
        const double dNoon = pf.wdir;
        std::printf("      wind: 07h dir=%.0f / 14h dir=%.0f\n", dMorn, dNoon);
        check(std::abs(dMorn - 185) < 25, "fujikawa: morning land breeze (tail, ~185deg)");
        check(std::abs(dNoon - 25) < 25, "fujikawa: midday sea breeze (head, ~25deg)");
        // 河原サーマル: 河川敷上空は上昇、川の上は付加なし
        pf.thermal = 0.5; pf.terrainWind = true; pf.wspd = 2;
        double aw, ax, av, aw2, ax2, av2;
        localWind(pf, -500, -300, 30, aw, ax, av);
        localWind(pf, -2000, -100, 30, aw2, ax2, av2);   // 第8弾: 川移設に伴い水上座標を更新
        std::printf("      riverbed thermal addVz=%.2f / over-river addVz=%.2f\n", av, av2);
        check(av > 0.1, "fujikawa: gravel riverbed generates strong thermals");
        check(av > av2, "fujikawa: weaker lift over the river than over land");
    }

    // ---- 公式距離: 発進点からの対地直線距離(pathAirとは独立) ----
    {
        FlightState d = makeInitialState(c, prm, 0);
        d.x = d.officialX0 + 600;
        d.yl = d.officialYl0 + 800;
        d.pathAir = 4200;
        refreshOfficialDistance(d);
        const double first = d.officialDist;
        d.pathAir = 9000;                 // 蛇行で対気経路だけ増えた想定
        refreshOfficialDistance(d);
        check(near(first, 1000.0, 1e-9) && near(d.officialDist, first, 1e-9),
              "official distance depends only on the common ground endpoint");
        check(near(d.pathAir, 9000.0, 1e-9), "air path remains available separately for analysis");

        AircraftConstants noGear = c;
        noGear.hasGear = false;
        SimParams rw = prm; rw.mode = "runway"; rw.site = "fujikawa";
        FlightState invalid = makeInitialState(noGear, rw, site::FUJI_START_Z_BASE);
        invalid.x = 500;
        refreshOfficialDistance(invalid);
        check(invalid.nogear && invalid.done && near(invalid.officialDist, 0.0, 1e-12),
              "nogear launch is an invalid zero official record");
    }

    // ---- 富士川の自由発進(第5弾): startPos/startHdg ----
    // 発進方位0°/90°/180°の3ケースで、機首方位・滑走方向・風との相対関係
    // (南の海風時: 0°=向かい風/180°=追い風)と滑走路逸脱・overrun新判定を検証
    {
        std::printf("\n[fujikawa free start]\n");
        SimParams pf = prm;
        pf.site = "fujikawa"; pf.mode = "runway";
        pf.startPos = 0;              // デフォルトが南エンド寄りに変わったため、
                                      // 従来値との比較テストは北端(startPos=0, z0=850。
                                      // 滑走路再設計850mに伴い旧z0=1000から比例更新)を明示指定
        pf.startHdg = 0;              // 第7弾でデフォルトが180(北向き)に変わったため、
                                      // 方位0/90/180の比較テストは基準の0(南向き)を明示指定
        pf.wind = -2; pf.xwind = 0;   // 南からの海風2m/s(コース前方=南 → wind負=向かい風)
        const double PI_T = 3.14159265358979323846;
        // ケース1: hdg=0(通常・南向き) → 向かい風で初期対気速度 = pushV+2
        {
            AircraftConstants c0 = aeroPack(st, a, pf);
            FlightState L0 = makeInitialState(c0, pf, site::FUJI_START_Z_BASE);
            std::printf("      hdg=0  : psi=%+.2f V0=%.2f (pushV=%.1f, 海風=向かい風)\n",
                        L0.psi, L0.V, pf.pushV);
            check(std::abs(L0.psi) < 1e-9, "free start: default heading 0 (backward compatible)");
            double sw0 = 0, sx0 = 0;
            horizontalWindAt(pf, 0, 0, 0, sw0, sx0);
            check(near(L0.V, std::abs(pf.pushV - sw0), 1e-9),
                  "free start: hdg=0 uses the shared surface-wind sample");
        }
        // ケース2: hdg=180(北向き=逆走) → 追い風で初期対気速度 = pushV-2、北(x減)へ滑走
        {
            SimParams p1 = pf; p1.startHdg = 180;
            AircraftConstants c1 = aeroPack(st, a, p1);
            FlightState L1 = makeInitialState(c1, p1, site::FUJI_START_Z_BASE);
            const double x0 = L1.x;
            std::printf("      hdg=180: psi=%+.2f V0=%.2f (海風=追い風)\n", L1.psi, L1.V);
            check(near(std::abs(L1.psi), PI_T, 1e-6), "free start: hdg=180 sets psi=pi");
            double sw1 = 0, sx1 = 0;
            horizontalWindAt(p1, 0, 0, 0, sw1, sx1);
            const double along1 = sw1 * std::cos(L1.psi) + sx1 * std::sin(L1.psi);
            check(near(L1.V, std::abs(p1.pushV - along1), 1e-9),
                  "free start: hdg=180 uses the same surface-wind sign convention");
            FlightState Lr = L1;
            for (int i = 0; i < 250 && Lr.ground && !Lr.done; i++) stepSim(Lr, c1, p1, 0.02);
            std::printf("      hdg=180 roll 5s: dx=%+.1f dyl=%+.1f rollDist=%.1f\n",
                        Lr.x - x0, Lr.yl, Lr.rollDist);
            check(Lr.x < x0 - 3, "free start: hdg=180 rolls north (course-x decreases)");
            check(Lr.rollDist > 3, "free start: rollDist accumulates during ground roll");
        }
        // ケース3: hdg=90(真横=川へ向く) → +ylへ滑走、滑走路を外れ砂利で加速が鈍る
        {
            SimParams p2 = pf; p2.startHdg = 90; p2.wind = 0;
            SimParams p3 = pf; p3.wind = 0;                        // 比較用 hdg=0(滑走路上)
            AircraftConstants c2 = aeroPack(st, a, p2);
            FlightState L2 = makeInitialState(c2, p2, site::FUJI_START_Z_BASE);
            FlightState L3 = makeInitialState(c2, p3, site::FUJI_START_Z_BASE);
            for (int i = 0; i < 250; i++) {
                if (L2.ground && !L2.done) stepSim(L2, c2, p2, 0.02);
                if (L3.ground && !L3.done) stepSim(L3, c2, p3, 0.02);
            }
            std::printf("      hdg=90 roll 5s: dyl=%+.1f dx=%+.1f V=%.2f / hdg=0(滑走路) V=%.2f\n",
                        L2.yl, L2.x, L2.V, L3.V);
            check(near(L2.psi, PI_T / 2, 1e-6), "free start: hdg=90 sets psi=pi/2");
            check(L2.yl > 3, "free start: hdg=90 rolls sideways toward the river (+yl)");
            check(std::abs(L2.x) < std::abs(L2.yl) * 0.2, "free start: hdg=90 course-x nearly constant");
            check(!L2.ground || !L3.ground || L2.V < L3.V - 0.02,
                  "free start: gravel off the runway slows acceleration (2.5x rolling resistance)");
        }
        // ケース4: 富士川のoverrun新判定=「滑走1200mで離陸速度未達」(方位自由化対応)
        {
            SimParams p4 = pf; p4.wind = 0;
            AircraftConstants c4 = aeroPack(st, a, p4);
            FlightState L4 = makeInitialState(c4, p4, site::FUJI_START_Z_BASE);
            L4.rollDist = site::FUJI_OVERRUN_DISTANCE + 50;
            stepSim(L4, c4, p4, 0.02);
            check(L4.overrun && L4.done, "free start: rollDist>1050 without liftoff -> overrun (fujikawa rule)");
            check(L4.officialInvalid && near(L4.officialDist, 0.0, 1e-12),
                  "free start: pre-liftoff overrun invalidates official record");
        }
    }

    // ---- 第6弾: 地上ステア/横風ドリフト/接地品質/南エンド発進 ----
    {
        std::printf("\n[ground handling v6]\n");
        const double PI_T = 3.14159265358979323846;
        SimParams pg = prm;
        pg.site = "fujikawa"; pg.mode = "runway"; pg.startPos = 0;
        pg.startHdg = 0;   // 本ブロックはFlightStateを手組みするが、意図を明示(第7弾デフォルトは180)
        pg.wind = 0; pg.xwind = 0;
        AircraftConstants cg = aeroPack(st, a, pg);

        // (1) 地上ステア: V=3/8 での最大回頭率(フルラダー1ステップ実測)と旧実装の比較
        {
            auto steerRate = [&](double V) {
                FlightState L{};
                L.ground = true; L.V = V; L.x = 500; L.z0 = 1000; L.yl = 0;
                L.rud = 1; L.rudTgt = 1; L.t = 5;
                const double psi0 = L.psi;
                stepSim(L, cg, pg, 0.02);
                return (L.psi - psi0) / 0.02 * 180.0 / PI_T;
            };
            auto oldRate = [&](double V) {   // 旧実装: 2·rudYaw·clamp(V/0.5Vt,0,1.2)
                return 2.0 * cg.rudYaw * clamp(V / std::max(0.5 * cg.Vt, 1.0), 0.0, 1.2) * 180.0 / PI_T;
            };
            const double n3 = steerRate(3), n8 = steerRate(8);
            std::printf("      steer max deg/s: V=3 new=%.1f (old=%.1f) / V=8 new=%.1f (old=%.1f) wheelBase=%.2f\n",
                        n3, oldRate(3), n8, oldRate(8), cg.wheelBase);
            check(n3 > oldRate(3) * 3, "v6 steer: low-speed turn rate far above old (~4x+)");
            check(n3 <= 30.0 + 1e-6 && n8 <= 30.0 + 1e-6, "v6 steer: capped at 30 deg/s (no ground spin)");
            check(n3 > 5 && n8 > 5, "v6 steer: usable turn rate at both speeds");
        }

        // (2) 横風3m/s・ラダー中立で60秒滑走: 横流れが旧比で大幅減+風見鶏でゆっくり風上へ
        //     (V=5に保持して定常滑走を再現する検証ハーネス。旧ドリフトの理論値=3m/s×60s=180m)
        {
            SimParams px = pg; px.xwind = 3;
            FlightState L{};
            L.ground = true; L.V = 5; L.x = 300; L.z0 = 1000; L.t = 5;
            double maxYl = 0;
            for (int i = 0; i < 3000 && !L.done; i++) {
                stepSim(L, cg, px, 0.02);
                L.V = 5;                            // 定常保持(離陸・失速判定を避ける)
                maxYl = std::max(maxYl, L.yl);
            }
            std::printf("      xwind3 60s roll: yl(max)=%.1fm final=%.1fm psi=%.1fdeg (旧理論値=180m)\n",
                        maxYl, L.yl, L.psi * 180 / PI_T);
            check(maxYl < 54.0, "v6 drift: downwind drift far below old (~180m -> <30%)");
            check(L.psi < -0.1, "v6 weathervane: nose slowly turns upwind with neutral rudder");
        }

        // (3) クラブ着陸(横風3m/s, 6DOF, 滑空): 接地フレーム前後で横速度ylRateが不連続ジャンプしない
        {
            SimParams px = pg; px.xwind = 3; px.P = 0;    // 滑空で自然な降下接地に
            FlightState L{};
            L.V = 8.5; L.gam = -0.05; L.h = 0.8; L.x = 500; L.z0 = 1000; L.yl = -5;
            L.theta = L.gam; L.init6 = true; L.t = 5;
            double prevYl = L.yl, prevRate = 0, maxJump = 0;
            bool first = true;
            for (int i = 0; i < 600 && !L.done; i++) {
                stepSim6(L, cg, px, 0.02);
                const double rate = (L.yl - prevYl) / 0.02;
                if (!first) maxJump = std::max(maxJump, std::abs(rate - prevRate));
                prevRate = rate; prevYl = L.yl; first = false;
            }
            std::printf("      crab landing (6DOF glide, xwind3): touchdowns=%d max|d(ylRate)|=%.2f m/s (旧実装≈数m/sのジャンプ)\n",
                        L.touchdowns, maxJump);
            check(L.touchdowns >= 1, "v6 crab: touches down");
            check(maxJump < 1.0, "v6 crab: no lateral velocity jump at touchdown (vLat decay)");
        }

        // (4) 沈下率による接地品質分岐: 1.5/3.0/4.0/5.5 m/s + 翼端接地の1段階悪化
        {
            // 指定沈下率の降下状態から接地するまで数ステップ回す(1ステップではhが0に届かない)
            auto touch = [&](double sink, double phi) {
                FlightState L{};
                L.V = 8; L.gam = -std::asin(sink / 8.0); L.h = 0.05;
                L.x = 500; L.z0 = 1000; L.yl = 0; L.phi = phi; L.t = 5;
                for (int i = 0; i < 10 && !L.done && !L.ground && L.touchdowns == 0; i++)
                    stepSim(L, cg, pg, 0.02);
                return L;
            };
            const FlightState n1 = touch(1.5, 0);
            const FlightState b1 = touch(3.0, 0);
            const FlightState g1 = touch(4.0, 0);
            const FlightState x1 = touch(5.5, 0);
            const FlightState w1 = touch(1.5, 0.30);   // バンク17°の翼端接地
            std::printf("      sink1.5: ground=%d / sink3.0: bounce(td=%d hard=%d V=%.2f) / "
                        "sink4.0: broken=%d / sink5.5: crash=%d / wingtip1.5: bounce=%d\n",
                        (int)n1.ground, b1.touchdowns, (int)b1.hardLanding, b1.V,
                        (int)g1.gearBroken, (int)(x1.done && x1.crashed), (int)(!w1.ground && w1.touchdowns == 1));
            check(n1.ground && !n1.gearBroken && !n1.sparBroken && !n1.crashed && !n1.done,
                  "v6 touchdown: sink 1.5 -> normal landing");
            check(!b1.ground && b1.touchdowns == 1 && b1.hardLanding && b1.V < 8 * 0.93,
                  "v6 touchdown: sink 3.0 -> hard landing bounce (V x0.92)");
            check(g1.ground && g1.gearBroken && !g1.sparBroken && !g1.crashed && !g1.done,
                  "v6 touchdown: sink 4.0 -> gear broken only, still rolling");
            check(x1.done && x1.crashed && !x1.splash && x1.impact == "crash",
                  "v6 touchdown: sink 5.5 -> land crash (not splash)");
            check(!w1.ground && w1.touchdowns == 1, "v6 touchdown: bank>12deg worsens one band (1.5 -> bounce)");

            // 気体の経路角だけでなく、下降流を含む実鉛直速度で接地区分が決まる。
            auto downdraftTouch = [&](bool sixdof) {
                FlightState D{};
                D.V = 8; D.gam = -std::asin(1.5 / 8.0); D.h = 0.01;
                D.x = -400; D.z0 = 0; D.yl = 0; D.t = 5; D.tz = -1.2;
                D.theta = D.gam; D.init6 = sixdof;
                for (int i = 0; i < 4 && !D.done && !D.ground && D.touchdowns == 0; i++)
                    sixdof ? stepSim6(D, cg, pg, 0.02) : stepSim(D, cg, pg, 0.02);
                return D;
            };
            const FlightState d3 = downdraftTouch(false);
            const FlightState d6 = downdraftTouch(true);
            check(d3.hardLanding && d3.touchdowns == 1,
                  "3DOF touchdown uses downdraft in actual sink rate");
            check(d6.hardLanding && d6.touchdowns == 1,
                  "6DOF touchdown uses downdraft in actual sink rate");

            auto touchAt = [&](double courseX, double yl, bool sixdof) {
                FlightState A{};
                A.V = 8; A.gam = -std::asin(1.0 / 8.0); A.h = 0.01;
                A.x = courseX; A.z0 = 0; A.yl = yl; A.t = 5;
                A.theta = A.gam; A.init6 = sixdof;
                for (int i = 0; i < 4 && !A.done && !A.ground; i++)
                    sixdof ? stepSim6(A, cg, pg, 0.02) : stepSim(A, cg, pg, 0.02);
                return A;
            };
            for (bool sixdof : {false, true}) {
                const FlightState sand = touchAt(-120, -160, sixdof);
                const FlightState water = touchAt(-500, -100, sixdof);
                check(sand.ground && !sand.splash,
                      sixdof ? "6DOF visible sandbar permits landing" : "3DOF visible sandbar permits landing");
                check(water.done && water.splash && !water.crashed,
                      sixdof ? "6DOF visible river causes splash" : "3DOF visible river causes splash");
            }
        }

        // (5) デフォルト発進(滑走路再設計850m: startPos=830 南エンド+startHdg=180 北向き):
        //     海風日和14時。南からの海風は追い風になるため初期対気速度は低く、
        //     滑走路全長(南端→北)を使って加速し、1050mの滑走以内に離陸できることを検証。
        {
            SimParams p5 = prm;
            p5.site = "fujikawa"; p5.mode = "runway";
            p5.summer = true; p5.weather = 0; p5.tod = 14.0;   // 海風日和(clear_sea)
            p5.dirJit = p5.tempJit = p5.wspdJit = p5.gustJit = 0;
            applySummer(p5);
            p5.turb = 0; p5.thermal = 0;                       // 決定的にするため乱流・サーマルは切る
            check(near(p5.startPos, site::FUJI_START_POS_DEFAULT, 1e-12),
                  "v7 default: startPos is 830 (south end)");
            check(near(p5.startHdg, 180.0, 1e-12), "v7 default: startHdg is 180 (facing north)");
            AircraftConstants c5 = aeroPack(st, a, p5);
            const double z0f = site::fujikawaStartWorldZ(p5.startPos);
            FlightState L5 = makeInitialState(c5, p5, z0f);
            check(near(std::abs(L5.psi), PI_T, 1e-6), "v7 default: initial heading is north (psi=pi)");
            double tLift = -1, rollAtLift = -1;
            double maxH = 0;
            while (!L5.done && L5.t < 300) {
                // 簡易パイロット: 滑走中はラダーでセンターライン維持(横風下の追い風滑走は
                // 実プレイでも操舵が必要。方位偏差+横ずれの比例制御で機首を北へ保つ)
                if (L5.ground) {
                    const double tgtPsi = PI_T + clamp(0.03 * L5.yl, -0.35, 0.35);
                    const double err = std::atan2(std::sin(L5.psi - tgtPsi), std::cos(L5.psi - tgtPsi));
                    L5.rudTgt = clamp(-4.0 * err, -1.0, 1.0);
                } else {
                    L5.rudTgt = 0;
                    // 離陸後は軽い引き起こしで上昇(3DOF手動モデルは操舵なしだと高度維持のため)
                    L5.eTgt = 0.35;
                }
                const bool wasG = L5.ground;
                stepSim(L5, c5, p5, 0.02);
                if (wasG && !L5.ground && tLift < 0) { tLift = L5.t; rollAtLift = L5.rollDist; }
                if (tLift >= 0) maxH = std::max(maxH, L5.h);
                if (tLift >= 0 && L5.t > tLift + 30) break;    // 離陸後30秒の飛行継続を確認
            }
            std::printf("      default launch (南端・北向き, 海風日和14時 wind=%.1f=追い風): 離陸滑走 %.0fm t=%.1fs maxH=%.1fm\n",
                        p5.wind, rollAtLift, tLift, maxH);
            check(tLift >= 0 && rollAtLift < site::FUJI_OVERRUN_DISTANCE,
                  "v7 default: lifts off within the 1050m roll budget (downwind full-runway run)");
            check(!L5.done && maxH > 3, "v7 default: keeps flying after downwind takeoff");
        }

        // (6) 迎角(主翼取付角)と自然浮上: CLg0が取付角に応じて増え、
        //     揚力≧重量で台本なしに浮くこと。取付角が大きいほど離陸滑走が短く、
        //     低取付角では自然浮上せず引き起こし(ローテーション)が必要になること
        {
            auto rollToLiftoff = [&](double inc, bool rotate) {
                AircraftParams si = st; si.incidence = inc;
                Analysis ai = analyze(si);
                SimParams pi = prm;
                pi.site = "fujikawa"; pi.mode = "runway"; pi.summer = false;
                pi.wind = 0; pi.xwind = 0; pi.turb = 0; pi.thermal = 0;
                AircraftConstants ci = aeroPack(si, ai, pi);
                // Game側と同じ発進原点変換(デフォルト: 南エンドstartPos=830 → z0=20)
                FlightState Li = makeInitialState(ci, pi, site::fujikawaStartWorldZ(pi.startPos));
                while (Li.ground && !Li.done && Li.t < 200) {
                    if (rotate && Li.V >= ci.Vs) Li.eTgt = 0.5;   // 引き起こし操作
                    stepSim(Li, ci, pi, 0.02);
                }
                return std::make_pair(Li.ground || Li.done ? -1.0 : Li.rollDist, ci.CLg0);
            };
            const auto [rollHi, clHi] = rollToLiftoff(6.0, false);
            const auto [rollMid, clMid] = rollToLiftoff(3.0, false);
            const auto [rollLoN, clLo] = rollToLiftoff(1.0, false);
            const auto [rollLoR, clLo2] = rollToLiftoff(1.0, true);
            (void)clLo2;
            std::printf("      incidence vs liftoff: inc=6 CLg0=%.2f roll=%.0fm / inc=3 CLg0=%.2f roll=%.0fm / "
                        "inc=1 CLg0=%.2f natural=%.0fm rotate=%.0fm\n",
                        clHi, rollHi, clMid, rollMid, clLo, rollLoN, rollLoR);
            check(clHi > clMid && clMid > clLo, "aoA: ground CL increases with incidence");
            check(rollHi > 0 && rollMid > 0, "aoA: inc 3-6deg lifts off naturally (lift >= W, no script)");
            check(rollHi < rollMid - 10, "aoA: higher incidence -> meaningfully shorter takeoff roll");
            check(rollLoN < 0 && rollLoR > 0, "aoA: low incidence needs rotation (no auto-liftoff script)");
        }
    }

    // ---- 体力モデル(手動適用) ----
    {
        SimParams pst = prm; pst.stamina = true; pst.P = 700;
        AircraftConstants cs = aeroPack(st, a, pst);
        FlightState Ls = makeInitialState(cs, pst, 0);
        double maxP = 0;
        while (Ls.t < 120 && !Ls.done) {
            stepSim(Ls, cs, pst, 0.02);
            maxP = std::max(maxP, Ls.Pnow);
        }
        std::printf("\n[stamina] demand=700W: maxP=%.0f endP=%.0f W'=%.1fkJ gly=%.2f\n",
                    maxP, Ls.Pnow, Ls.wbal / 1000, Ls.gly);
        check(maxP <= st.powerMax + 220 + 1, "stamina caps burst at CP+220W");
        check(Ls.wbal < 3000, "W' drains under sustained overdemand");
        check(Ls.Pnow < st.powerMax + 5, "output falls to sustainable after W' depleted");
        check(Ls.gly < 1.0, "glycogen depletes under load");

        // グリコーゲンは低出力でも減り、飛行中は回復しない(絞って無限回復できない)
        SimParams pl = prm; pl.stamina = true; pl.P = 60;   // ずっと低出力
        AircraftConstants cl = aeroPack(st, a, pl);
        FlightState Ll = makeInitialState(cl, pl, 0);
        double gly30 = 1;
        while (Ll.t < 120 && !Ll.done) {
            stepSim(Ll, cl, pl, 0.02);
            if (Ll.t >= 30 && gly30 == 1) gly30 = Ll.gly;
        }
        std::printf("[stamina] low-power(60W): gly@30s=%.3f gly@end=%.3f (monotonic drain)\n", gly30, Ll.gly);
        check(gly30 < 1.0, "glycogen drains even at low power (basal metabolism)");
        check(Ll.gly < gly30, "glycogen never recovers in flight (no infinite recovery exploit)");

        // 枯渇でCPeが低下: 満タン時より枯渇時の持続出力が明確に低い
        AircraftConstants cf = aeroPack(st, a, pst);
        FlightState Lfull = makeInitialState(cf, pst, 0);
        FlightState Lempty = Lfull; Lempty.gly = 0.0; Lempty.wbal = pst.wcap * 1000;
        Lfull.wbal = pst.wcap * 1000;
        // 30秒巡航要求での持続出力を比較(直接CPe相当を測る: 低出力要求で頭打ち)
        SimParams pc = pst; pc.P = 1000;
        double pFull = 0, pEmpty = 0;
        for (int i = 0; i < 60 * 40; i++) stepSim(Lfull, cf, pc, 0.02), pFull = Lfull.Pnow;
        for (int i = 0; i < 60 * 40; i++) stepSim(Lempty, cf, pc, 0.02), pEmpty = Lempty.Pnow;
        std::printf("[stamina] sustained P: full-gly=%.0fW empty-gly=%.0fW\n", pFull, pEmpty);
        check(pEmpty < pFull - 20, "depleted glycogen lowers sustainable power");
    }

    // ---- フラッター疲労破断 ----
    {
        AircraftConstants cf = aeroPack(st, a, prm);
        FlightState Lf = makeInitialState(cf, prm, 0);
        int steps = 0;
        while (!Lf.sparBroken && steps++ < 6000) {
            Lf.V = cf.VNE * 0.95;   // VNE手前を維持(振動域)
            Lf.h = 100;
            stepSim(Lf, cf, prm, 0.02);
        }
        std::printf("\n[flutter] broken after %.1fs at 0.95VNE: %s (fatigue=%.2f)\n",
                    steps * 0.02, Lf.failureMsg.c_str(), Lf.fatigue);
        check(Lf.sparBroken, "sustained near-VNE flight breaks the spar by fatigue");
        check(steps * 0.02 > 3, "but not instantly (warning window exists)");
    }

    // ---- 動的サーマル場 ----
    {
        SimParams pt = prm; pt.thermal = 1.0; pt.realThermal = true;
        pt.site = "biwa"; pt.wind = pt.xwind = pt.wspdMean = 0;
        const double v1 = thermalFieldVz(pt, 5000, 4000, 100);
        const double v2 = thermalFieldVz(pt, 5000, 4000, 100);
        check(near(v1, v2, 1e-12), "thermal field is deterministic");
        double landMax = -9, waterMax = -9, vmin = 9;
        for (double x = 0; x < 6000; x += 60)
            for (double y = 3300; y < 6000; y += 60) {
                const double v = thermalFieldVz(pt, x, y, 200);
                landMax = std::max(landMax, v); vmin = std::min(vmin, v);
            }
        for (double x = 0; x < 6000; x += 60)
            for (double y = -1500; y < 1500; y += 60)
                waterMax = std::max(waterMax, thermalFieldVz(pt, x, y, 200));
        std::printf("\n[realThermal] landMax=%.2f waterMax=%.2f min=%.2f m/s\n",
                    landMax, waterMax, vmin);
        check(landMax > 0.4, "active thermal cores exist over land");
        check(landMax > waterMax * 2.0 + 0.1, "strong thermal cores are suppressed over water");
        check(vmin > -0.2, "gentle sink between cells");
        // 時間経過でセルが入れ替わる
        double diff = 0;
        for (double x = 0; x < 4000; x += 200)
            diff += std::abs(thermalFieldVz(pt, x, 4000, 100) - thermalFieldVz(pt, x, 4000, 700));
        check(diff > 0.3, "cells evolve over time");

        // 同じ発生セルが平均風下へ移動することを可視化APIでも確認する。
        double sx0[64], sy0[64], ss0[64], sr0[64];
        double sx1[64], sy1[64], ss1[64], sr1[64];
        SimParams adv = pt; adv.wspdMean = 2.0; adv.wdir = 180; // +x方向へ2m/s
        const int n0 = thermalSitesNear(pt, 3000, 4000, 200, 64, sx0, sy0, ss0, sr0);
        const int n1 = thermalSitesNear(adv, 3000, 4000, 200, 64, sx1, sy1, ss1, sr1);
        bool advected = n0 > 0 && n0 == n1;
        for (int i = 0; i < std::min(n0, n1); i++) advected = advected && sx1[i] > sx0[i] + 5.0;
        check(advected, "thermal cells advect downwind at a deterministic rate");

        // 最強風ではセルが約2.2km移流する。可視化APIが返すコア位置を物理場でも
        // 必ず拾えることを確認し、固定7x7探索への後戻りを防ぐ。
        SimParams strong = pt; strong.wspdMean = 9.3; strong.wdir = 180;
        double sx[256], sy[256], ss[256], sr[256];
        const int ns = thermalSitesNear(strong, 3000, 4000, 400, 256, sx, sy, ss, sr);
        int strongCores = 0;
        bool coresFound = true;
        for (int i = 0; i < ns; i++) if (ss[i] > 0.2) {
            strongCores++;
            coresFound = coresFound && thermalFieldVz(strong, sx[i], sy[i], 400) > 0.03;
        }
        check(strongCores > 0 && coresFound,
              "thermal: strong-wind advected cores remain inside the physics search");
    }

    // ---- Phase 3: CFRP材料・局所強度・質量・操舵接続 ----
    {
        const auto& mats = materialDB();
        check(mats.size() == 3 && near(materialOf("t700").young, 115e9, 1.0)
              && near(materialOf("t800").compressive, 700e6, 1.0)
              && near(materialOf("m40j").density, 1600, 1e-12),
              "phase3: CFRP grade table matches the approved SI values");

        AircraftParams t700 = st, t800 = st, m40j = st;
        t700.sparMat = "t700"; t800.sparMat = "t800"; m40j.sparMat = "m40j";
        const Analysis a700 = analyze(t700), a800 = analyze(t800), a40 = analyze(m40j);
        const AircraftConstants c700 = aeroPack(t700, a700, prm);
        const AircraftConstants c800 = aeroPack(t800, a800, prm);
        const AircraftConstants c40 = aeroPack(m40j, a40, prm);
        std::printf("\n[phase3 material] t700 n=%.2f VNE=%.1f m=%.2f / t800 n=%.2f VNE=%.1f / m40j n=%.2f VNE=%.1f\n",
                    c700.nFail, c700.VNE, a700.wEmpty, c800.nFail, c800.VNE,
                    c40.nFail, c40.VNE);
        check(c800.nFail > c700.nFail, "phase3: higher compressive allowable raises nFail");
        check(c800.VNE > c700.VNE && c40.VNE > c800.VNE,
              "phase3: higher Young's modulus raises VNE monotonically");
        check(c40.nFail < c700.nFail, "phase3: high modulus does not masquerade as high strength");
        check(near(a700.sparSF, c700.nFail, 1e-12),
              "phase3: analysis spar SF exactly equals flight nFail");
        check(near(c700.nFailNeg, -std::max(0.5, 0.6 * c700.nFail), 1e-12),
              "phase3: negative structural limit derives from positive nFail");
        check(a700.wEmpty >= 27.8 && a700.wEmpty <= 29.0
              && c700.nFail >= 2.11 * 0.9 && c700.nFail <= 2.11 * 1.1
              && c700.VNE >= 16.0 * 0.9 && c700.VNE <= 16.0 * 1.1,
              "phase3: default mass/nFail/VNE remain inside Claude calibration bands");

        AircraftParams thick = t700;
        thick.rootDia *= 1.15; thick.tipDia *= 1.15;
        const Analysis aThick = analyze(thick);
        check(aThick.wSpar > a700.wSpar && aThick.sparSF > a700.sparSF,
              "phase3: larger tube diameter raises both spar mass and strength");

        AircraftParams tapered = t700;
        tapered.rootDia = 135; tapered.tipDia = 30; tapered.segments = 2;
        const Analysis aTaper = analyze(tapered);
        std::printf("[phase3 local] tapered SF=%.2f station=%.2f mode=%s\n",
                    aTaper.sparSF, aTaper.failStation, aTaper.failMode.c_str());
        check(aTaper.failStation > 0.4,
              "phase3: strong taper is governed by an outboard local station");

        AircraftParams bare = st; bare.gear = "none"; bare.fairing = false; bare.drive = "chain";
        AircraftParams geared = bare; geared.gear = "tandem";
        AircraftParams faired = bare; faired.fairing = true;
        AircraftParams shaft = bare; shaft.drive = "shaft";
        const Analysis aBare = analyze(bare), aGear = analyze(geared);
        const Analysis aFair = analyze(faired), aShaft = analyze(shaft);
        check(near(aGear.wEmpty - aBare.wEmpty, 0.9, 1e-9)
              && near(aFair.wEmpty - aBare.wEmpty, 1.1, 1e-9)
              && near(aShaft.wEmpty - aBare.wEmpty, 0.25 * aShaft.driveDist, 1e-9),
              "phase3: gear, fairing and shaft masses enter total mass once");
        AircraftParams boomSmall = bare, boomLarge = bare;
        boomSmall.boomDia = 50; boomLarge.boomDia = 120;
        check(analyze(boomLarge).wBoom > analyze(boomSmall).wBoom * 4.0,
              "phase3: boom diameter controls boom tube mass");

        AircraftParams ailSmall = st, ailLarge = st;
        ailSmall.ailMode = ailLarge.ailMode = "large";
        ailSmall.ailSpanFrac = 0.15; ailSmall.ailChordFrac = 0.20;
        ailLarge.ailSpanFrac = 0.40; ailLarge.ailChordFrac = 0.40;
        const auto cAilSmall = aeroPack(ailSmall, analyze(ailSmall), prm);
        const auto cAilLarge = aeroPack(ailLarge, analyze(ailLarge), prm);
        AircraftParams tailSmall = st;
        tailSmall.elevRatio = 0.3; tailSmall.rudRatio = 0.35;
        const auto cTailSmall = aeroPack(tailSmall, analyze(tailSmall), prm);
        check(cAilLarge.ailRate > cAilSmall.ailRate && cAilLarge.Clda > cAilSmall.Clda
              && cTailSmall.elevAuth < c700.elevAuth && cTailSmall.Cmde < c700.Cmde
              && cTailSmall.rudYaw < c700.rudYaw && cTailSmall.Cndr < c700.Cndr,
              "phase3: control-surface dimensions scale 3DOF/6DOF authority coefficients");

        AircraftParams old180, old230, old300;
        aircraftFromJson("{\"sparMod\":180}", old180);
        aircraftFromJson("{\"sparMod\":230}", old230);
        aircraftFromJson("{\"sparMod\":300}", old300);
        check(old180.sparMat == "t700" && old230.sparMat == "t800" && old300.sparMat == "m40j",
              "phase3: legacy sparMod JSON migrates to approved material grades");
        check(legacyModForMaterial("t700") == 180 && legacyModForMaterial("t800") == 230
              && legacyModForMaterial("m40j") == 320
              && std::string(materialFromLegacyMod(legacyModForMaterial("m40j"))) == "m40j",
              "phase3: legacy URL export representatives preserve every material grade");
        AircraftParams noMaterial;
        aircraftFromJson("{\"span\":28}", noMaterial);
        check(noMaterial.sparMat == "t700", "phase3: JSON without either material key keeps T700 default");
        AircraftParams saved = st; saved.sparMat = "m40j"; saved.boomDia = 95;
        AircraftParams loaded;
        aircraftFromJson(aircraftToJson(saved), loaded);
        check(loaded.sparMat == "m40j" && near(loaded.boomDia, 95, 1e-12),
              "phase3: sparMat and boomDia survive JSON round-trip");

        AircraftParams seamless = t700;
        seamless.rootDia = 115; seamless.tipDia = 45; seamless.segments = 1;
        AircraftParams sleeved = seamless; sleeved.segments = 5;
        const Analysis aSeamless = analyze(seamless), aSleeved = analyze(sleeved);
        std::printf("[phase3 joint] seamless SF=%.2f / 5-piece SF=%.2f station=%.2f\n",
                    aSeamless.sparSF, aSleeved.sparSF, aSleeved.failStation);
        check(aSleeved.sparSF < aSeamless.sparSF
              && near(aSleeved.failStation, 0.20, 1e-12),
              "phase3: sleeve 0.80 knockdown lowers SF at the first strong-taper joint");

        AircraftConstants breakC = c700;
        breakC.nFail = 0.5;
        FlightState broken = makeInitialState(breakC, prm, 0);
        broken.ground = false; broken.h = 100; broken.V = breakC.Vt;
        broken.t = 10; broken.liftoffT = -1e9;
        stepSim(broken, breakC, prm, 0.02);
        check(broken.sparBroken && near(broken.failStation, breakC.failStation, 1e-12)
              && broken.failMode == breakC.failMode
              && broken.failureMsg.find(u8"半翼") != std::string::npos,
              "phase3: actual structural failure records station and mode in its message");
    }

    // ---- Phase 4: 負揚力・回転モデル操縦補助・双方向構造破壊 ----
    {
        std::printf("\n[phase4 flight dynamics]\n");
        const double smCases[3] = {10.0, 0.0, -5.0};
        bool sameElevatorSign = true, independentAuthority = true;
        for (double smCase : smCases) {
            Analysis ax = a; ax.SM = smCase;
            AircraftConstants cx = aeroPack(st, ax, prm);
            independentAuthority = independentAuthority
                                && near(cx.Cmde, 0.15 * cx.elevAuth, 1e-12);
            SimParams px = prm; px.assist = false;
            FlightState lx = makeInitialState(cx, px, 0);
            lx.ground = false; lx.h = 100; lx.V = cx.Vdesign;
            lx.gam = lx.theta = lx.alpha = 0; lx.init6 = true;
            lx.e = 0.4;
            stepSim6(lx, cx, px, 0.02);
            sameElevatorSign = sameElevatorSign && lx.qRate > 0;
        }
        check(independentAuthority, "phase4: Cmde is 0.15*elevAuth and independent of SM");
        check(sameElevatorSign, "phase4: elevator keeps the same pitch direction at SM +10/0/-5");

        SimParams assistOn = prm, assistOff = prm;
        assistOn.assist = true; assistOff.assist = false;
        auto bankDisturbance = [&](const SimParams& p) {
            FlightState x = makeInitialState(c, p, 0);
            x.ground = false; x.h = 200; x.V = c.Vdesign;
            x.gam = x.theta = x.alpha = 0; x.init6 = true;
            x.phi = 8.0 * 3.14159265358979323846 / 180.0;
            for (int i = 0; i < 250 && !x.done; i++) stepSim6(x, c, p, 0.02);
            return x;
        };
        const FlightState rollOn = bankDisturbance(assistOn);
        const FlightState rollOff = bankDisturbance(assistOff);
        std::printf("  bank 5s: assist ON %.2fdeg / OFF %.2fdeg\n",
                    rollOn.phi * 180 / 3.14159265358979323846,
                    rollOff.phi * 180 / 3.14159265358979323846);
        check(std::abs(rollOn.phi) < 3.0 * 3.14159265358979323846 / 180.0,
              "phase4: assist ON levels an 8deg bank disturbance below 3deg");
        check(std::abs(rollOff.phi) > std::abs(rollOn.phi) + 1.0 * 3.14159265358979323846 / 180.0
              && rollOff.ailApplied == 0.0,
              "phase4: assist OFF has no active wing-level command or assisted settling");

        FlightState exact = makeInitialState(c, assistOff, 0);
        exact.ground = false; exact.h = 100; exact.V = c.Vdesign;
        exact.gam = exact.theta = exact.alpha = 0; exact.init6 = true;
        exact.e = 0.17; exact.ail = -0.23; exact.rud = 0.31;
        stepSim6(exact, c, assistOff, 0.02);
        check(exact.eApplied == exact.e && exact.ailApplied == exact.ail && exact.rudApplied == exact.rud,
              "phase4: assist OFF passes all three user controls through exactly");

        // 同じ動圧と要求CLで、両モデルが同符号・概ね同じ負荷重を出す。
        AircraftConstants cn = c; cn.nFailNeg = -99; cn.VNE = 99;
        const double negV = 1.8 * cn.Vdesign;
        const double targetCL = -0.35;
        const double qn = 0.5 * cn.rho * negV * negV;
        const double cl1g = cn.W / (qn * cn.S);
        FlightState n3 = makeInitialState(cn, assistOff, 0);
        n3.ground = false; n3.h = 200; n3.V = negV; n3.gam = 0;
        n3.t = 10; n3.liftoffT = -1e9; n3.e = (targetCL - cl1g) / cn.elevAuth; n3.eTgt = n3.e;
        FlightState n6 = n3;
        n6.e = 0; n6.theta = n6.gam + (targetCL - cn.CLcruise) / cn.CLa;
        n6.alpha = n6.theta - n6.gam; n6.init6 = true;
        stepSim(n3, cn, assistOff, 0.02);
        stepSim6(n6, cn, assistOff, 0.02);
        std::printf("  matched negative load: 3DOF %.3f / rotation %.3f\n", n3.n, n6.n);
        check(n3.n < 0 && n6.n < 0, "phase4: both physics models produce negative load with the same sign");
        check(std::abs(n3.n - n6.n) / std::max(0.05, std::abs(n3.n)) <= 0.35,
              "phase4: negative-load magnitude differs by no more than 35 percent");

        // フラップ展開時も、実際のCL需要がCLminへ達するまでは負側失速させない。
        SimParams flapPrm = assistOff; flapPrm.funPlane = true;
        AircraftConstants flapC = funPlaneConstants(flapPrm);
        flapC.nFailNeg = -99; flapC.VNE = 99;
        FlightState flapL = makeInitialState(flapC, flapPrm, 0);
        flapL.ground = false; flapL.h = 200; flapL.V = flapC.Vdesign;
        flapL.flap = flapL.flapTgt = 1.0; flapL.init6 = true;
        const double flapCLmin = std::min(-0.15, -0.50 * flapC.CLmax + 0.60 * flapC.flapDCL);
        const double flapDemand = flapCLmin + 0.25 * flapC.flapDCL;
        flapL.gam = 0;
        flapL.theta = (flapDemand - flapC.CLcruise - flapC.flapDCL) / flapC.CLa;
        flapL.alpha = flapL.theta;
        stepSim6(flapL, flapC, flapPrm, 0.02);
        const double flapExpectedN = 0.5 * flapC.rho * flapC.Vdesign * flapC.Vdesign
                                   * flapC.S * flapDemand / flapC.W;
        check(near(flapL.n, flapExpectedN, 1e-9),
              "phase4: deployed flap does not trigger negative stall before CL demand reaches CLmin");

        auto crossingN = [&](bool six, double target) {
            FlightState x = makeInitialState(cn, assistOff, 0);
            x.ground = false; x.h = 200; x.V = 1.5 * cn.Vdesign;
            x.gam = 0; x.t = 10; x.liftoffT = -1e9;
            const double qx = 0.5 * cn.rho * x.V * x.V;
            if (six) {
                x.theta = (target - cn.CLcruise) / cn.CLa;
                x.alpha = x.theta; x.init6 = true;
                stepSim6(x, cn, assistOff, 0.02);
            } else {
                const double base = cn.W / (qx * cn.S);
                x.e = (target - base) / cn.elevAuth; x.eTgt = x.e;
                stepSim(x, cn, assistOff, 0.02);
            }
            return x.n;
        };
        const double dN3 = std::abs(crossingN(false, 0.01) - crossingN(false, -0.01));
        const double dN6 = std::abs(crossingN(true, 0.01) - crossingN(true, -0.01));
        check(dN3 < 0.15 && dN6 < 0.15, "phase4: load remains continuous while crossing 0G");

        // 限界の1.05倍を繰り返し与え、0.14秒では耐え0.16秒で折れることを確認。
        AircraftConstants cs = cn; cs.nFailNeg = -0.20;
        FlightState ns = makeInitialState(cs, assistOff, 0);
        ns.ground = false; ns.h = 200; ns.t = 10; ns.liftoffT = -1e9;
        const double persistV = 1.5 * cs.Vdesign;
        const double persistCL = -0.21 * cs.W / (0.5 * cs.rho * persistV * persistV * cs.S);
        for (int i = 0; i < 7; i++) {
            ns.ground = false; ns.h = 200; ns.V = persistV; ns.gam = 0;
            const double base = cs.W / (0.5 * cs.rho * persistV * persistV * cs.S);
            ns.e = (persistCL - base) / cs.elevAuth; ns.eTgt = ns.e;
            stepSim(ns, cs, assistOff, 0.02);
        }
        check(!ns.sparBroken && near(ns.overNT, 0.14, 1e-9),
              "phase4: negative limit exceedance survives the first 0.14 seconds");
        ns.ground = false; ns.h = 200; ns.V = persistV; ns.gam = 0;
        stepSim(ns, cs, assistOff, 0.02);
        check(ns.sparBroken, "phase4: sustained negative limit exceedance breaks at 0.15 seconds");

        SimParams holdOff = assistOff; holdOff.hold = true; holdOff.hTgt = 10;
        FlightState hh = makeInitialState(c, holdOff, 0);
        hh.ground = false; hh.h = 10; hh.V = c.Vdesign;
        hh.gam = hh.theta = hh.alpha = 0; hh.init6 = true;
        for (int i = 0; i < 500 && !hh.done; i++) stepSim6(hh, c, holdOff, 0.02);
        check(std::abs(hh.h - holdOff.hTgt) < 1.0,
              "phase4: altitude hold remains active with general assist OFF");

        FlightState recover = makeInitialState(cn, assistOn, 0);
        recover.ground = false; recover.h = 200; recover.V = cn.Vdesign;
        recover.gam = 0;
        const double negStallAlpha = (-0.50 * cn.CLmax - cn.CLcruise) / cn.CLa;
        recover.theta = negStallAlpha - 0.08;
        recover.alpha = recover.theta; recover.init6 = true;
        for (int i = 0; i < 250 && !recover.done; i++) stepSim6(recover, cn, assistOn, 0.02);
        check(recover.n >= 0.7 && recover.n <= 1.3,
              "phase4: assisted negative stall recovers to 0.7-1.3G within 5 seconds");
    }

    // ---- Phase 5: 地上風・解析値の実飛行整合 ----
    {
        std::printf("\n[phase5 environment consistency]\n");
        SimParams surface = prm;
        surface.site = "fujikawa"; surface.mode = "runway";
        surface.terrainWind = false; surface.wind = 3.0; surface.xwind = 0;
        surface.pushV = 5.0; surface.P = 0; surface.startHdg = 0;
        double w0 = 0, xw0 = 0, w1 = 0, xw1 = 0;
        horizontalWindAt(surface, 0, 0, 0, w0, xw0);
        horizontalWindAt(surface, 0, 0, 0.02, w1, xw1);
        check(std::abs(w1 - w0) < 0.05 && near(xw0, xw1, 1e-12),
              "phase5: surface-to-air wind profile is continuous");

        SimParams tail = surface; tail.wind = 12.0; tail.pushV = 4.0;
        AircraftConstants ct = aeroPack(st, a, tail);
        FlightState lt = makeInitialState(ct, tail, 0);
        check(near(lt.groundSpeed, tail.pushV, 1e-12),
              "phase5: a tailwind stronger than push speed does not change the commanded ground start");
        stepSim(lt, ct, tail, 0.02);
        check(lt.groundSpeed < tail.pushV + 0.1,
              "phase5: strong tailwind produces force, not an instantaneous ground-speed jump");

        SimParams cold = prm, hot = prm;
        cold.temp = cold.launchTemp = 0; hot.temp = hot.launchTemp = 35;
        const PolarResult pcold = computePolar(st, a, cold);
        const PolarResult phot = computePolar(st, a, hot);
        check(phot.Vs > pcold.Vs, "phase5: polar uses flight air density (hot air raises stall speed)");
        check(analyze(st, &hot).V > analyze(st, &cold).V,
              "phase5: summary analysis uses the same atmospheric density");

        AircraftParams geared = st, clean = st;
        geared.gear = "tri"; clean.gear = "none";
        const PolarResult pg = computePolar(geared, analyze(geared), prm);
        const PolarResult pn = computePolar(clean, analyze(clean), prm);
        check(pg.Dp > pn.Dp, "phase5: polar profile drag includes landing gear");

        AircraftParams efficient = st, lossy = st;
        efficient.driveEffPct = 100; lossy.driveEffPct = 60;
        const PolarResult pe = computePolar(efficient, analyze(efficient), prm);
        const PolarResult pl = computePolar(lossy, analyze(lossy), prm);
        check(pl.mpY > pe.mpY * 1.5, "phase5: polar required pilot power includes drivetrain efficiency");
        check(analyze(lossy, &prm).Preq > analyze(efficient, &prm).Preq * 1.5,
              "phase5: summary required power includes drivetrain efficiency");

        SimParams north = surface, south = surface;
        north.startHdg = 0; south.startHdg = 180;
        const GustResult gn = gustCalc(st, a, north);
        const GustResult gs = gustCalc(st, a, south);
        check(gs.launchVa > gn.launchVa,
              "phase5: Fujikawa launch analysis uses pushV, heading and surface wind");

        SimParams locked = hot;
        locked.atmosphereLocked = true; locked.launchTemp = 12; locked.temp = 35;
        check(near(airDensity(locked), airDensity([] { SimParams p; p.temp = 12; return p; }()), 1e-12),
              "phase5: in-flight analysis displays launch-locked atmospheric density");
    }

    // ---- 破壊テスト: 桁の弱い機体は高G旋回で折れる ----
    AircraftParams weak = st; weak.rootDia = 70; weak.tipDia = 40;
    Analysis aw = analyze(weak);
    AircraftConstants cw = aeroPack(weak, aw, prm);
    std::printf("\n[weak spar] nFail=%.3f (should be low)\n", cw.nFail);
    check(cw.nFail < c.nFail, "thinner spar has lower failure load");

    // ==== リアリズム更新(LLT / BEMT / Re極曲線 / たわみ連成 / Dryden / 暑熱) ====
    std::printf("\n---- realism upgrades ----\n");
    // LLT: 楕円翼はe~1、平面形・ねじりが効く
    {
        AircraftParams el = st; el.planform = "ellipse"; el.washout = 0; el.tipChord = 0.02;
        check(computeLLT(el, 1.0).e > 0.97, "LLT: elliptic wing e ~= 1");
        AircraftParams rc = st; rc.planform = "rect"; rc.washout = 0;
        check(computeLLT(rc, 1.0).e < computeLLT(el, 1.0).e, "LLT: rect wing worse e than ellipse");
        AircraftParams tp = st; tp.washout = 0; tp.tipChord = 0.30;
        AircraftParams tw = tp; tw.washout = 4;
        check(computeLLT(tw, 1.0).tStall < computeLLT(tp, 1.0).tStall,
              "LLT: washout moves stall onset inboard");
    }
    // BEMT: 巡航効率・ピッチ依存・風車状態
    {
        const PropTables pt2 = computeBEMT(st);
        double n1;
        const double T1 = propThrust(pt2.J, pt2.Ct, pt2.Cp, st.propDia, c.rho, 7.5, 240, n1);
        std::printf("[bemt] cruise eta=%.3f n=%.0frpm\n", T1 * 7.5 / 240, n1 * 60);
        check(T1 * 7.5 / 240 > 0.72 && T1 * 7.5 / 240 < 0.93, "BEMT: cruise efficiency plausible");
        double nw;
        check(propThrust(pt2.J, pt2.Ct, pt2.Cp, st.propDia, c.rho, 8.0, 0, nw) <= 0.5,
              "BEMT: no free thrust at zero power (windmill)");
    }
    // Re極曲線: 低Reで断面抗力が増える(全翼型)
    {
        for (const auto& af : airfoilDB())
            if (afCd0AtRe(af, 1.2e5) <= afCd0AtRe(af, 5e5)) {
                check(false, "polar: cd0 rises at low Re for all airfoils");
                break;
            }
        const AirfoilData& thick = airfoilOf([] { AircraftParams t; t.airfoil = "fx76"; return t; }());
        const AirfoilData& thin = airfoilOf([] { AircraftParams t; t.airfoil = "ag18"; return t; }());
        check(afCd0AtRe(thick, 1e5) / thick.cdT[2] > afCd0AtRe(thin, 1e5) / thin.cdT[2],
              "polar: thick airfoil suffers more at low Re");
    }
    // たわみ連成: 曲げ上反角が正で、フラット治具でも荷重で横安定が出る
    {
        check(c.dihBend > 0, "flex: bending adds effective dihedral");
        AircraftParams fl2 = st; fl2.jig = "flat";
        AircraftConstants cf2 = aeroPack(fl2, analyze(fl2), prm);
        check(cf2.dihBase == 0 && cf2.dihBend > 0.3, "flex: flat jig gains dihedral from load");
    }
    // Dryden乱流: seedに対して決定的で、RMSが強度に比例し高空で弱まる
    {
        SimParams pt3 = prm; pt3.turb = 1.0;
        FlightState Lt; Lt.V = 7.5;
        double s2lo = 0, s2hi = 0;
        Lt.h = 2;  for (int i = 0; i < 4000; i++) { stepTurbulence(Lt, pt3, 0.02); s2lo += Lt.tz * Lt.tz; }
        Lt.tz = 0; Lt.h = 60;
        for (int i = 0; i < 4000; i++) { stepTurbulence(Lt, pt3, 0.02); s2hi += Lt.tz * Lt.tz; }
        std::printf("[dryden] rms(h=2)=%.2f rms(h=60)=%.2f m/s\n",
                    std::sqrt(s2lo / 4000), std::sqrt(s2hi / 4000));
        check(s2lo > s2hi * 1.5, "dryden: turbulence stronger near ground");
        check(std::sqrt(s2lo / 4000) > 0.3 && std::sqrt(s2lo / 4000) < 1.5,
              "dryden: low-alt RMS in sane range");
    }
    // 暑熱: 気温が高いとCPが落ちる
    {
        SimParams hot = prm; hot.temp = 34;
        AircraftConstants ch = aeroPack(st, a, hot);
        std::printf("[heat] heatFac(34degC)=%.3f\n", ch.heatFac);
        check(ch.heatFac < 1.0 && ch.heatFac >= 0.88, "heat: hot day derates CP up to 12%");
        check(near(aeroPack(st, a, prm).heatFac, 1.0, 1e-9) || prm.temp > 24,
              "heat: no derating at or below 24degC");
    }

    std::printf("\n=== %s (%d failure(s)) ===\n", failures ? "TEST FAILED" : "ALL TESTS PASSED", failures);
    return failures ? 1 : 0;
}
