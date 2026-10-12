# AI Geode

An AI player that learns Geometry Dash levels **from its own deaths**.

Turn it on in a level (default key **F8**, or the **AI** button in the pause menu) and it plays by itself. After every death it works out what went wrong (jumped too early, too late, held too long...), changes that part of its play, and tries again. What it learns is saved per level, so it continues where it left off next time.

- Real reinforcement learning (Q-learning) plus targeted timing experiments - not a recorded macro
- Status panel showing what failed, what the AI changed and whether it helped
- Learning memory can be reset, exported and imported
- Optional practice-mode learning with AI-placed checkpoints

**Limitations:** classic (non-platformer) levels only; in 2-player levels only player 1 is controlled. Hard levels can take hundreds or thousands of attempts, and some may never be beaten.
