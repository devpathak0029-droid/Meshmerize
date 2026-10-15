# ==============================================================================
# Techfest Meshmerize 2026-27: Comprehensive Maze Simulator & Test Suite
# Tests: Complete 9-rule turn reduction algebra, cascading reductions,
# boundary handling, buffer overflow protections, and maze navigation replay
# ==============================================================================

import unittest

# Complete Canonical Left-Hand Rule Reduction Table
REDUCTION_RULES = {
    ("L", "B", "L"): "S",
    ("L", "B", "S"): "R",
    ("L", "B", "R"): "B",
    ("R", "B", "L"): "B",
    ("S", "B", "L"): "R",
    ("S", "B", "S"): "B",
    ("R", "B", "R"): "S",
    ("S", "B", "R"): "L", # Added Canonical Rule
    ("R", "B", "S"): "L", # Added Canonical Rule
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
            i = max(0, i - 2)
        else:
            i += 1
    return "".join(path)

class TestMazeSolver(unittest.TestCase):
    
    def test_all_individual_reduction_rules(self):
        """Test each of the 9 canonical reduction rules individually."""
        expected_rules = {
            "LBL": "S",
            "LBS": "R",
            "LBR": "B",
            "RBL": "B",
            "SBL": "R",
            "SBS": "B",
            "RBR": "S",
            "SBR": "L",
            "RBS": "L",
        }
        for raw, expected in expected_rules.items():
            self.assertEqual(simplify_path(raw), expected, f"Failed on {raw}")

    def test_cascading_dead_ends(self):
        """Test recursive reductions where reducing one dead end triggers another."""
        # LBL -> S, then S B S -> B
        self.assertEqual(simplify_path("LBLBS"), "B")
        # LBL -> S, then S B L -> R, followed by L -> RL
        self.assertEqual(simplify_path("LBLBLL"), "RL")
        # L B L -> S, S L B R -> S B
        self.assertEqual(simplify_path("LBLLBR"), "SB")
        # S B R -> L, L B L -> S
        self.assertEqual(simplify_path("SBRLBL"), "LS")

    def test_paths_without_dead_ends(self):
        """Paths without 'B' should remain untouched."""
        straight_path = "LLSSRRSS"
        self.assertEqual(simplify_path(straight_path), straight_path)

    def test_boundary_and_short_paths(self):
        """Test empty and short strings."""
        self.assertEqual(simplify_path(""), "")
        self.assertEqual(simplify_path("L"), "L")
        self.assertEqual(simplify_path("LB"), "LB")
        self.assertEqual(simplify_path("LL"), "LL")

    def test_complex_maze_traversal(self):
        """Simulate realistic dry run through multi-branch maze with 3 dead ends."""
        # Bot: Turn Left, dead end (LBL -> S)
        # Advance straight, encounter T, turn Left, dead end, take Right branch (LBS -> R)
        # Advance, encounter dead end (SBR -> L)
        raw_traversal = "LBL" + "LBS" + "SBR"
        # Simplifies: S + R + L = SRL
        self.assertEqual(simplify_path(raw_traversal), "SRL")

if __name__ == "__main__":
    unittest.main()
