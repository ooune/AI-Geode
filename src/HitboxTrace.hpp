#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

class PlayLayer;
class PlayerObject;
class GameObject;

// Optional raw trace of the game's own collision geometry ("Hitbox Trace"
// setting, off by default), written to hitbox-trace.csv in the mod's save
// folder. It exists to check the AI's collision rules against what the game
// actually does, offline:
//  P rows: the player's rectangle, the rectangle GD reports at 0.3 scale (inner
//          hitbox), the rotation and the player's oriented box, every 30 steps;
//  N rows: every step a round or rotated hazard is within 8 units of the
//          player's rectangle (passes the player survived);
//  D rows: the killing object at each death, in the same format.
class HitboxTrace {
public:
    void begin(std::string const& path);
    void end();
    bool active() const { return m_file != nullptr; }
    void step(PlayLayer* layer, PlayerObject* player, int32_t tick, std::vector<GameObject*> const& nearby);
    void death(PlayLayer* layer, PlayerObject* player, GameObject* killer, int32_t tick);

private:
    void player(char kind, PlayerObject* p, int32_t tick);
    void object(GameObject* o);
    std::FILE* m_file = nullptr;
    size_t m_rows = 0;
};
