#ifndef MAZESOLVER_H
#define MAZESOLVER_H

#include <Arduino.h>
#include "Config.h"

// Actions taken at intersections:
// 'L' = Left turn
// 'R' = Right turn
// 'S' = Straight through
// 'B' = Back (U-turn / dead end)
// 'E' = End zone reached

class MazeSolver {
public:
    MazeSolver();

    void reset();
    void recordTurn(char turn);
    
    // Simplifies raw turns using standard turn-reduction rules
    // (LBL->S, LBR->B, LBS->R, SBL->R, SBS->B, RBL->B)
    void simplifyPath();

    // Loop detection & prune:
    // If the bot revisits the same junction orientation, trims cyclical dead paths
    void optimizeAll();

    const char* getPath() const { return _path; }
    uint8_t getPathLength() const { return _pathLength; }
    char getTurnAt(uint8_t index) const;

    void printPath() const;

private:
    char _path[MAX_PATH_LENGTH];
    uint8_t _pathLength;
};

#endif // MAZESOLVER_H
