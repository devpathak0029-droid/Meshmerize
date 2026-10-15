# ==============================================================================
# Techfest Meshmerize 2026-27: Maze Path Optimizer & Simulator
# Verifies turn reduction, dead-end elimination, and loop optimizations
# ==============================================================================

REDUCTION_RULES = {
    ("L", "B", "L"): "S",
    ("L", "B", "S"): "R",
    ("L", "B", "R"): "B",
    ("R", "B", "L"): "B",
    ("S", "B", "L"): "R",
    ("S", "B", "S"): "B",
    ("R", "B", "R"): "S",
}

def simplify_path(path_turns):
    """
    Simulates recursive turn simplification matching the C++ MazeSolver engine.
    """
    path = list(path_turns)
    i = 0
    while i < len(path) - 2:
        triplet = (path[i], path[i+1], path[i+2])
        if triplet in REDUCTION_RULES:
            replacement = REDUCTION_RULES[triplet]
            path = path[:i] + [replacement] + path[i+3:]
            # Backtrack to catch cascading reductions (e.g. L + (LBL->S) + B...)
            i = max(0, i - 2)
        else:
            i += 1
    return "".join(path)

def test_simplifications():
    print("--------------------------------------------------")
    print("Running Maze Turn Simplification Test Suite")
    print("--------------------------------------------------")
    
    test_cases = [
        # Basic Dead Ends
        ("LBL", "S"),
        ("LBS", "R"),
        ("LBR", "B"),
        ("RBL", "B"),
        ("SBL", "R"),
        ("SBS", "B"),
        
        # Cascading dead end reductions
        # "L B L B S" -> (LBL->S) -> "S B S" -> B
        ("LBLBS", "B"),
        # "L B L B L L" -> (LBL->S) -> "S B L L" -> (SBL->R) -> "RL"
        ("LBLBLL", "RL"),
        
        # Simulated Dry Run through a maze section with 2 dead ends:
        # "L B L L B R" -> (LBL->S) -> "S L B R" -> (LBR->B) -> "S B"
        ("LBLLBR", "SB"),
    ]
    
    all_passed = True
    for raw, expected in test_cases:
        res = simplify_path(raw)
        passed = (res == expected)
        print(f"Raw: {raw:12} -> Optimized: {res:6} | Expected: {expected:6} [{'PASS' if passed else 'FAIL'}]")
        if not passed:
            all_passed = False
            
    print("--------------------------------------------------")
    if all_passed:
        print(">> ALL TEST CASES PASSED SUCCESSFULLY! <<")
    else:
        print(">> SOME TESTS FAILED. CHECK RULES! <<")
    print("--------------------------------------------------")

if __name__ == "__main__":
    test_simplifications()
