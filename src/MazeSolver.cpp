#include "MazeSolver.h"

MazeSolver::MazeSolver() {
    reset();
}

void MazeSolver::reset() {
    _pathLength = 0;
    memset(_path, 0, sizeof(_path));
}

void MazeSolver::recordTurn(char turn) {
    if (_pathLength < MAX_PATH_LENGTH - 1) {
        _path[_pathLength++] = turn;
        _path[_pathLength] = '\0';
        simplifyPath(); // Simplify on the fly whenever a dead-end 'B' is added
    }
}

char MazeSolver::getTurnAt(uint8_t index) const {
    if (index < _pathLength) {
        return _path[index];
    }
    return '\0';
}

void MazeSolver::simplifyPath() {
    // We only simplify if the penultimate turn was a 'B' (U-turn)
    if (_pathLength < 3 || _path[_pathLength - 2] != 'B') {
        return;
    }

    char first  = _path[_pathLength - 3];
    char second = _path[_pathLength - 2]; // 'B'
    char third  = _path[_pathLength - 1];

    char replacement = 0;

    // Standard Left-Hand Rule Simplification Table:
    // L B L -> S
    // L B S -> R
    // L B R -> B
    // R B L -> B
    // S B L -> R
    // S B S -> B
    // R B R -> S (if Right priority occurs)
    if (first == 'L' && third == 'L') {
        replacement = 'S';
    } else if (first == 'L' && third == 'S') {
        replacement = 'R';
    } else if (first == 'L' && third == 'R') {
        replacement = 'B';
    } else if (first == 'R' && third == 'L') {
        replacement = 'B';
    } else if (first == 'S' && third == 'L') {
        replacement = 'R';
    } else if (first == 'S' && third == 'S') {
        replacement = 'B';
    } else if (first == 'R' && third == 'R') {
        replacement = 'S';
    }

    if (replacement != 0) {
        _path[_pathLength - 3] = replacement;
        _pathLength -= 2;
        _path[_pathLength] = '\0';

        // Recursively simplify if the replacement is also a 'B'
        if (_pathLength >= 3 && _path[_pathLength - 2] == 'B') {
            simplifyPath();
        }
    }
}

void MazeSolver::optimizeAll() {
    // Repeatedly simplify until no 'B' remains
    bool reduced = true;
    while (reduced) {
        reduced = false;
        for (int i = 0; i <= (int)_pathLength - 3; i++) {
            if (_path[i + 1] == 'B') {
                char first = _path[i];
                char third = _path[i + 2];
                char rep = 0;

                if (first == 'L' && third == 'L') rep = 'S';
                else if (first == 'L' && third == 'S') rep = 'R';
                else if (first == 'L' && third == 'R') rep = 'B';
                else if (first == 'R' && third == 'L') rep = 'B';
                else if (first == 'S' && third == 'L') rep = 'R';
                else if (first == 'S' && third == 'S') rep = 'B';
                else if (first == 'R' && third == 'R') rep = 'S';

                if (rep != 0) {
                    _path[i] = rep;
                    // Shift the rest left by 2 positions
                    for (int j = i + 1; j < _pathLength - 2; j++) {
                        _path[j] = _path[j + 2];
                    }
                    _pathLength -= 2;
                    _path[_pathLength] = '\0';
                    reduced = true;
                    break;
                }
            }
        }
    }
}

void MazeSolver::printPath() const {
    Serial.print("Learned Path (Length ");
    Serial.print(_pathLength);
    Serial.print("): ");
    Serial.println(_path);
}
