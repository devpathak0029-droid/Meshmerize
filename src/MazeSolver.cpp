#include "MazeSolver.h"

MazeSolver::MazeSolver() {
    reset();
}

void MazeSolver::reset() {
    _pathLength = 0;
    memset(_path, 0, sizeof(_path));
}

bool MazeSolver::recordTurn(char turn) {
    if (_pathLength < MAX_PATH_LENGTH - 1) {
        _path[_pathLength++] = turn;
        _path[_pathLength] = '\0';
        simplifyPath(); // Simplify on the fly whenever a dead-end 'B' is added
        return true;
    }
    Serial.println("ERROR: Path buffer overflow!");
    return false;
}

char MazeSolver::getTurnAt(uint8_t index) const {
    if (index < _pathLength) {
        return _path[index];
    }
    return '\0';
}

void MazeSolver::simplifyPath() {
    // Only simplify if the penultimate turn was a 'B' (U-turn at dead end)
    if (_pathLength < 3 || _path[_pathLength - 2] != 'B') {
        return;
    }

    char first  = _path[_pathLength - 3];
    char third  = _path[_pathLength - 1];
    char replacement = 0;

    // Complete canonical Left-Hand Rule turn-reduction algebra:
    // 1. L B L -> S
    // 2. L B S -> R
    // 3. L B R -> B
    // 4. R B L -> B
    // 5. S B L -> R
    // 6. S B S -> B
    // 7. R B R -> S
    // 8. S B R -> L  (Canonical: heading straight, hitting dead end, exiting original right = Left turn)
    // 9. R B S -> L  (Canonical: turning right, hitting dead end, continuing straight = Left turn)
    if (first == 'L' && third == 'L') replacement = 'S';
    else if (first == 'L' && third == 'S') replacement = 'R';
    else if (first == 'L' && third == 'R') replacement = 'B';
    else if (first == 'R' && third == 'L') replacement = 'B';
    else if (first == 'S' && third == 'L') replacement = 'R';
    else if (first == 'S' && third == 'S') replacement = 'B';
    else if (first == 'R' && third == 'R') replacement = 'S';
    else if (first == 'S' && third == 'R') replacement = 'L';
    else if (first == 'R' && third == 'S') replacement = 'L';

    if (replacement != 0) {
        _path[_pathLength - 3] = replacement;
        _pathLength -= 2;
        _path[_pathLength] = '\0';

        // Recursively simplify if cascading reduction placed another 'B'
        if (_pathLength >= 3 && _path[_pathLength - 2] == 'B') {
            simplifyPath();
        }
    }
}

void MazeSolver::optimizeAll() {
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
                else if (first == 'S' && third == 'R') rep = 'L';
                else if (first == 'R' && third == 'S') rep = 'L';

                if (rep != 0) {
                    _path[i] = rep;
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
