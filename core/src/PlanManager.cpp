#include <aigd/PlanManager.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace aigd {

namespace {
constexpr size_t kHistoryMax = 32;
} // namespace

char const* planSourceName(PlanSource s) {
    switch (s) {
        case PlanSource::Search: return "search";
        case PlanSource::Learned: return "learned timing";
        case PlanSource::Fallback: return "fallback (policy)";
        default: return "-";
    }
}

int LivePlan::indexOf(int32_t tick) const {
    int32_t i = tick - createdTick - 1;
    if (i < 0) return -1;
    return static_cast<int>(std::min<int32_t>(i, static_cast<int32_t>(steps.size())));
}

int LivePlan::pendingChanges(int32_t tick) const {
    int n = 0;
    int from = std::max(1, indexOf(tick) + 1);
    for (size_t j = static_cast<size_t>(from); j < steps.size(); ++j) n += steps[j].hold != steps[j - 1].hold ? 1 : 0;
    return n;
}

void PlanManager::clear() {
    m_plan = LivePlan{};
    adopted = invalidated = cancelledActions = 0;
    lastCheck = PlanCheck{};
    lastInvalidation = PlanCheck{};
    history.clear();
}

void PlanManager::archive(int32_t tick, bool cancelled, int cancelledActionsHere, std::string reason) {
    if (!m_plan.active) return;
    PlanRecord r;
    r.createdTick = m_plan.createdTick;
    r.endedTick = tick;
    r.params = m_plan.params;
    r.source = m_plan.source;
    r.cancelled = cancelled;
    r.cancelledActions = cancelledActionsHere;
    r.reason = std::move(reason);
    history.push_back(std::move(r));
    while (history.size() > kHistoryMax) history.pop_front();
    m_plan.active = false;
}

void PlanManager::adopt(PredictedPath const& path, Observation const& from, int32_t tick, PlanSource source,
                        Predictor::Advice const& adv, bool followable) {
    archive(tick, false, 0, "replaced by a new plan");
    m_plan = LivePlan{};
    if (path.points.empty() || path.holds.size() < path.points.size()) return;
    m_plan.active = true;
    m_plan.createdTick = tick;
    m_plan.mode = from.mode;
    m_plan.speedClass = from.speedClass();
    m_plan.upsideDown = from.upsideDown;
    m_plan.mini = from.mini;
    m_plan.steps.reserve(path.points.size());
    for (size_t i = 0; i < path.points.size(); ++i) {
        PlannedStep st;
        st.tick = tick + 1 + static_cast<int32_t>(i);
        st.hold = path.holds[i];
        st.x = path.points[i].x;
        st.y = path.points[i].y;
        st.grounded = i < path.ground.size() && path.ground[i] != 0;
        m_plan.steps.push_back(st);
    }
    m_plan.params = path.plan;
    m_plan.outcome = path.outcome;
    m_plan.score = adv.bestScore;
    m_plan.candidates = adv.simulations;
    m_plan.stage = adv.stage;
    m_plan.source = source;
    m_plan.landed = path.landed;
    m_plan.landX = path.landX;
    m_plan.startX = from.x;
    m_plan.startY = from.y;
    m_plan.startGrounded = from.onGround;
    m_plan.followable = followable;
    ++adopted;
}

void PlanManager::release(int32_t tick, char const* why) { archive(tick, false, 0, why); }

int PlanManager::holdFor(int32_t tick) const {
    if (!m_plan.active) return -1;
    int32_t i = tick - m_plan.createdTick; // the step tick+1 is index tick - createdTick
    if (i < 0 || i >= static_cast<int32_t>(m_plan.steps.size())) return -1;
    return m_plan.steps[static_cast<size_t>(i)].hold;
}

PlanCheck PlanManager::validate(Observation const& obs, float tolerance) {
    PlanCheck c;
    c.hadPlan = m_plan.active;
    c.actualGrounded = obs.onGround;
    if (!m_plan.active) {
        lastCheck = c;
        return c;
    }
    char buf[200];
    std::string reason;
    // Physics changed: everything planned after this point assumed the old physics.
    if (obs.mode != m_plan.mode) reason = "Plan invalidated by a game mode change";
    else if (obs.speedClass() != m_plan.speedClass) reason = "Plan invalidated by a speed change";
    else if (obs.upsideDown != m_plan.upsideDown) reason = "Plan invalidated by a gravity change";
    else if (obs.mini != m_plan.mini) reason = "Plan invalidated by a size change";

    int const i = m_plan.indexOf(obs.tick);
    if (reason.empty()) {
        if (i < 0) {
            // Still the step the plan was made in: valid only for the state it was made from.
            c.dx = obs.x - m_plan.startX;
            c.dy = obs.y - m_plan.startY;
            if (std::fabs(c.dx) <= 0.01f && std::fabs(c.dy) <= 0.01f && obs.onGround == m_plan.startGrounded) {
                c.valid = true;
                lastCheck = c;
                return c;
            }
            reason = "The state changed within the step the plan was made in";
        } else if (i >= static_cast<int>(m_plan.steps.size())) {
            c.finished = true;
            archive(obs.tick, false, 0, "completed");
            lastCheck = c;
            return c;
        }
    }
    if (reason.empty()) {
        PlannedStep const& e = m_plan.steps[static_cast<size_t>(i)];
        c.dx = obs.x - e.x;
        c.dy = obs.y - e.y;
        c.expectedGrounded = e.grounded;
        // Grounded / airborne: compared over a step either side, so a landing
        // that is exactly one step off does not count as a different plan.
        bool groundMismatch = true;
        for (int j = std::max(0, i - 1); j <= std::min<int>(i + 1, static_cast<int>(m_plan.steps.size()) - 1); ++j)
            if ((m_plan.steps[static_cast<size_t>(j)].grounded != 0) == obs.onGround) groundMismatch = false;
        if (groundMismatch && obs.onGround) {
            std::snprintf(buf, sizeof(buf), "Landing detected earlier than expected (landed at y=%.1f, planned airborne at y=%.1f)", obs.y, e.y);
            reason = buf;
        } else if (groundMismatch) {
            std::snprintf(buf, sizeof(buf), "Still airborne where the plan expected the ground (y=%.1f, planned y=%.1f)", obs.y, e.y);
            reason = buf;
        } else if (std::fabs(c.dy) > tolerance) {
            std::snprintf(buf, sizeof(buf), "Current state differs from planned state (height off by %+.1f units)", c.dy);
            reason = buf;
        } else if (std::fabs(c.dx) > std::max(2.f, tolerance)) {
            std::snprintf(buf, sizeof(buf), "Current state differs from planned state (position off by %+.1f units)", c.dx);
            reason = buf;
        }
    }
    if (reason.empty()) {
        c.valid = true;
        lastCheck = c;
        return c;
    }
    c.valid = false;
    c.reason = reason;
    c.cancelled = m_plan.pendingChanges(obs.tick);
    ++invalidated;
    cancelledActions += static_cast<uint32_t>(c.cancelled);
    archive(obs.tick, true, c.cancelled, reason);
    lastCheck = c;
    lastInvalidation = c;
    return c;
}

} // namespace aigd
