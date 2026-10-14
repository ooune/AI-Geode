#include "StatusPanel.hpp"

#include "AIController.hpp"

#include <Geode/Geode.hpp>

using namespace geode::prelude;

namespace {
constexpr float kScale = 0.42f;
constexpr float kPad = 4.f;

std::string percent(float p) {
    return fmt::format("{:.1f}%", p * 100.f);
}
} // namespace

StatusPanel* StatusPanel::create() {
    auto* ret = new StatusPanel();
    if (ret->init()) {
        ret->autorelease();
        return ret;
    }
    delete ret;
    return nullptr;
}

bool StatusPanel::init() {
    if (!CCNode::init()) return false;
    auto win = CCDirector::get()->getWinSize();
    this->setAnchorPoint({0.f, 1.f});
    this->setPosition({6.f, win.height - 24.f});

    m_background = CCLayerColor::create({0, 0, 0, 130}, 10.f, 10.f);
    m_background->setAnchorPoint({0.f, 1.f});
    this->addChild(m_background);

    m_label = CCLabelBMFont::create("", "chatFont.fnt");
    m_label->setAnchorPoint({0.f, 1.f});
    m_label->setScale(kScale);
    m_label->setAlignment(kCCTextAlignmentLeft);
    this->addChild(m_label, 1);
    return true;
}

void StatusPanel::refresh() {
    auto& ai = AIController::get();
    auto const& set = ai.settings();
    this->setVisible(set.showPanel);
    if (!set.showPanel) return;

    std::string text;
    if (!ai.supported()) {
        text = "AI: unavailable here\n" + ai.unsupportedReason();
    } else if (!set.aiEnabled) {
        text = "AI: off  (toggle with the keybind, default F8)";
        if (auto const* mem = ai.levelMemory(); mem && mem->totalAttempts > 0) {
            text += fmt::format("\nRemembered: {} attempts, best {}, {} completions", mem->totalAttempts, percent(mem->bestProgress), mem->completions);
        }
    } else {
        auto const& st = ai.status();
        auto const* mem = ai.levelMemory();
        text = fmt::format(
            "AI: {}{}{}  |  Attempt {}  |  Deaths {}\n"
            "Progress {}  Best {}  Completions {} ({:.0f}% of recent)",
            aigd::agentPhaseName(st.phase), set.learning ? "" : " (learning off)", st.stuck ? " - stuck, widening search" : "",
            st.attempt, st.deaths,
            percent(st.progress), percent(st.bestProgress), st.completions, st.completionRate * 100.f
        );
        if (st.lastFailureProgress >= 0.f) text += "\nLast failure: " + st.lastFailure;
        if (!st.lastChange.empty()) text += "\nChange: " + st.lastChange;
        if (st.lastImproved >= 0) text += st.lastImproved ? "\nResult: went further than the previous attempt" : "\nResult: did not get further";
        if (!st.lastCredit.empty()) text += "\nCause: " + st.lastCredit;
        text += fmt::format("\nDecision: {}", aigd::decisionSourceName(st.source));
        if (!st.reason.empty()) text += "\nLast notable: " + st.reason;
        if (!ai.notice().empty()) text += "\n" + ai.notice();

        if (set.prediction) {
            auto const& pr = ai.predictor();
            auto outcome = [](aigd::Certainty c, int32_t ttc) {
                return ttc >= 0 ? fmt::format("{} in {}", aigd::certaintyName(c), ttc + 1) : std::string(aigd::certaintyName(c));
            };
            if (!aigd::Predictor::supports(ai.lastObservation().mode)) {
                text += fmt::format("\nPrediction: not modelled for {} (learning from experience)", aigd::gameModeName(ai.lastObservation().mode));
            } else if (!st.predictionValid) {
                text += "\nPrediction: nothing to avoid ahead";
            } else {
                // The decision, explained from the candidate evaluations that
                // were actually made for it.
                auto const& sf = st.safety;
                bool const steered = aigd::isFlyingMode(sf.mode);
                auto clearOr = [&](aigd::Certainty c, int32_t ttc) {
                    if (c == aigd::Certainty::Collision) return fmt::format("collision in {} steps", ttc + 1);
                    return std::string(c == aigd::Certainty::Safe ? "predicted clear" : "uncertain");
                };
                std::string obstacle = sf.hazardAhead ? fmt::format("hazard {:.1f} blocks ahead ({:+.0f} high)", sf.hazardDx / aigd::kBlock, sf.hazardDy)
                                                      : std::string("no hazard ahead");
                char const* pressName = steered ? "Hold" : "Jump";
                char const* releaseName = steered ? "Release" : "No jump";
                text += fmt::format("\n{} | {}: {} | {}: {} | Selected: {} | Executed: {}", obstacle, pressName,
                                    clearOr(sf.outcome[1], sf.ttc[1]), releaseName,
                                    clearOr(steered ? sf.outcome[0] : sf.never, steered ? sf.ttc[0] : sf.neverTTC),
                                    sf.proposed ? pressName : releaseName, sf.executed ? pressName : releaseName);
                bool learned = sf.proposedSource == aigd::DecisionSource::AvoidRepeat || sf.proposedSource == aigd::DecisionSource::KnownGood ||
                               sf.proposedSource == aigd::DecisionSource::Experiment;
                text += fmt::format("\nProposed by {}{} | {} | {}{}", aigd::decisionSourceName(sf.proposedSource),
                                    learned ? " (learned from earlier attempts)" : "",
                                    sf.trusted ? fmt::format("prediction trusted, drift {:.3f}/step, latency {}", pr.model().driftPerTick(sf.mode), pr.inputLatency())
                                               : std::string("PREDICTION NOT USED"),
                                    aigd::safetyVerdictName(sf.verdict), sf.predictionCurrent || !sf.trusted ? "" : " (stale prediction)");
                if (!sf.reason.empty()) text += "\nWhy: " + sf.reason;
                text += fmt::format("\nThis attempt: {} presses, {} unnecessary presses skipped, {} safety overrides, {} with no safe option",
                                    st.pressesThisAttempt, st.pressesAvoidedThisAttempt, st.overridesThisAttempt, st.noSafeOptionThisAttempt);
                // The live plan and how its timing was found (all from the
                // actual plan / search, nothing assumed).
                auto const& plans = ai.plans();
                auto const& lp = plans.plan();
                if (lp.active) {
                    int32_t const now = ai.lastTick();
                    std::string params = lp.params.pressAt < 0   ? std::string("no press")
                                       : lp.params.pressAt == 0 ? std::string("press now")
                                                                : fmt::format("press in {} steps", lp.params.pressAt);
                    if (lp.params.holdFor > 0) params += fmt::format(", hold {}", lp.params.holdFor);
                    else if (lp.params.holdFor < 0) params += ", keep holding";
                    if (lp.params.secondAt >= 0) params += fmt::format(", second press {} steps after landing", lp.params.secondAt);
                    text += fmt::format("\nPlan: {} (made {} step(s) ago), {} pending input change(s) | {} | {} | {} candidates, stage {}, score {:.1f}{}",
                                        st.safety.proposedSource == aigd::DecisionSource::Plan ? "following" : "made this step",
                                        plans.age(now), lp.pendingChanges(now), params, aigd::planSourceName(lp.source), lp.candidates,
                                        aigd::searchStageName(lp.stage), lp.score,
                                        lp.landed ? fmt::format(", lands at x={:.0f}", lp.landX) : std::string());
                    auto const& adv = ai.predictor().lastAdvice();
                    text += fmt::format("\nSearch: resolution {} step(s){}{}{}", adv.resolution,
                                        adv.seeded ? (adv.fromSeed ? ", learned timing tried first and kept" : ", learned timing tried first, search found better")
                                                   : "",
                                        adv.budgetHit ? ", stopped by the computation budget" : "",
                                        adv.firstSafeSim > 0 ? fmt::format(", first safe candidate after {} simulations", adv.firstSafeSim) : std::string());
                } else {
                    text += "\nPlan: none (no trusted prediction - the learned policy decides each step)";
                }
                if (plans.lastCheck.hadPlan)
                    text += fmt::format("\nActual vs planned: height {:+.1f}, position {:+.1f}, {} (planned {})", plans.lastCheck.dy, plans.lastCheck.dx,
                                        plans.lastCheck.actualGrounded ? "on the ground" : "in the air",
                                        plans.lastCheck.expectedGrounded ? "on the ground" : "in the air");
                if (!st.planEvent.empty()) text += "\nReplanned: " + st.planEvent;
                text += fmt::format("\nThis attempt: {} plans made, {} invalidated, {} pending actions cancelled, {} decisions followed a plan",
                                    plans.adopted, st.plansInvalidatedThisAttempt, st.actionsCancelledThisAttempt, st.decisionsFollowingPlan);
                (void)outcome;
            }
        }

        // Cross-level transfer: what is ahead, and what the strategy library did about it.
        if (!st.situation.empty()) text += "\nAhead: " + st.situation;
        if (!st.transferEvent.empty()) text += "\nTransfer: " + st.transferEvent;
        {
            auto const& T = st.transfer;
            if (T.situations > 0)
                text += fmt::format("\nStrategies this attempt: {} groups ({} known, {} similar, {} new) | {} executed as learned, {} replaced "
                                    "by the search, {} played directly, {} rejected after failing here",
                                    T.situations, T.known, T.similar, T.unknown, T.confirmed, T.rejectedBySim, T.played, T.rejectedHere);
        }

        if (set.showDebug) {
            auto const& o = ai.lastObservation();
            auto feat = [](char const* name, aigd::Feature const& f) {
                return f.present ? fmt::format(" {} dx={:.0f} dy={:.0f}", name, f.dx, f.dy) : std::string();
            };
            text += fmt::format(
                "\n[obs] {} {}{}{} x={:.1f} y={:.1f} vy={:.2f} speed={:.1f}{}{}{}",
                aigd::gameModeName(o.mode), o.onGround ? "ground" : "air", o.upsideDown ? " flipped" : "", o.mini ? " mini" : "",
                o.x, o.y, o.vy, o.playerSpeed, feat("hazard", o.hazard), feat("solid", o.solid), feat("orb", o.orb)
            );
            text += fmt::format(
                "\n[decide] Q(release)={:.2f} Q(hold)={:.2f} -> {} ({})  eps={:.2f}",
                st.q[0], st.q[1], st.action ? "HOLD" : "release", aigd::decisionSourceName(st.source), st.epsilon
            );
            text += fmt::format(
                "\n[evidence here] release: failed {} ({} in a row), passed {} | hold: failed {} ({} in a row), passed {}",
                st.fails[0], st.streak[0], st.passes[0], st.fails[1], st.streak[1], st.passes[1]
            );
            auto const& m = st.metrics;
            text += fmt::format(
                "\n[learning] repeated failures {} (alternatives tried {}, helped {}, vetoed by prediction {}) | known mistakes repeated {}"
                "\n[credit] causes found by prediction {}, by analysis {}, unknown {} | doomed decisions not blamed {} | model errors {}"
                " | reference replays {} of {} decisions",
                m.repeatedFailuresDetected, m.alternativesTried, m.alternativesThatHelped, m.alternativesVetoed, m.sameMistakeRepeated,
                m.pivotsByPrediction, m.pivotsByAnalysis, m.pivotsUnknown, m.doomedSkipped, m.mispredictedDeaths, m.referenceSteps,
                m.referenceSteps + m.independentSteps
            );
            text += fmt::format(
                "\n[memory] {} level states, {} pattern states, {} hotspots, {} objects indexed",
                st.tableSize, st.patternSize, mem ? mem->hotspots.size() : 0, ai.indexedObjects()
            );
            if (set.prediction) {
                auto const& pr = ai.predictor();
                auto const& adv = pr.lastAdvice();
                text += fmt::format(
                    "\n[physics] {} | step error {:.2f}u (pred y {:.2f} / real {:.2f}) | 32-step error {:.2f}u | estimates replaced {}",
                    pr.model().describe(ai.lastObservation().mode), pr.lastStepError(), pr.lastPredictedY(), pr.lastActualY(), pr.model().lastResimError(),
                    pr.model().regimeChanges()
                );
                text += fmt::format(
                    "\n[planner] {} sims in {:.0f}us | {} obstacles | best: {} | mispredictions {} (caution {:.2f}) | {}{}",
                    adv.simulations, adv.micros, ai.obstacleCount(),
                    adv.bestPresses ? std::string("press now") : (adv.bestPressAt >= 0 ? fmt::format("press in {} steps", adv.bestPressAt) : std::string("no press")),
                    pr.mispredictions(), pr.caution(), st.planned ? "planned " : "", st.shielded ? "SHIELDED" : ""
                );
                if (!st.predictionNote.empty()) text += "\n[last death] " + st.predictionNote;
            }
        }
    }

    m_label->setString(text.c_str());
    auto size = m_label->getScaledContentSize();
    m_background->setContentSize({size.width + kPad * 2.f, size.height + kPad * 2.f});
    m_background->setPosition({-kPad, -size.height - kPad});
    m_label->setPosition({0.f, 0.f});
}
