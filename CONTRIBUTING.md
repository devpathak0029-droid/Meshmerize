# CONTRIBUTING.md - Meshmerize Robotics Development

Welcome to the **Meshmerize** autonomous robotics development repository.

## Contribution Guidelines
1. **Branching Strategy:** Create feature branches (`feat/sensor-enhancement`, `fix/turn-debounce`) off `main`.
2. **Commit Hygiene:** Use Conventional Commits (`feat: ...`, `fix: ...`, `docs: ...`, `test: ...`).
3. **Build Requirement:** Every PR must compile cleanly with `pio run` and pass host-side tests in `python test/maze_sim.py`.
4. **Hardware Safety:** Never commit aggressive PID or speed values as defaults without bench testing on actual motors. Ensure safety bounds exist on motor PWM and turn timeouts.
