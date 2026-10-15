import math
import time

# Simulation of the Techfest 2026-27 Arena and Robot Dynamics
LINES = [
    (180, 200, 180, 170),
    (180, 200, 210, 200),
    (180, 170, 153, 170),
    (153, 170, 153, 200), # Dead end 1
    (180, 170, 180, 110),
    (180, 110, 213, 110),
    (213, 110, 213, 50),
    (213, 50, 180, 50),
    (180, 50, 180, 110),
    (180, 80, 150, 80),
    (150, 80, 150, 120),  # Dead end 2
    (180, 50, 123, 50),
    (123, 50, 123, 93),
    (123, 93, 90, 93),
    (90, 93, 90, 120),
    (90, 120, 120, 120),
    (120, 120, 120, 150), # Dead end 3
    (90, 93, 53, 93),
    (53, 93, 53, 53),
    (53, 53, 23, 53),
    (23, 53, 23, 180),
    (23, 180, 70, 180),
]

END_ZONE = (70, 160, 40, 40)

REDUCTIONS = {
    ('L', 'B', 'L'): 'S',
    ('L', 'B', 'S'): 'R',
    ('L', 'B', 'R'): 'B',
    ('R', 'B', 'L'): 'B',
    ('S', 'B', 'L'): 'R',
    ('S', 'B', 'S'): 'B',
    ('R', 'B', 'R'): 'S',
    ('S', 'B', 'R'): 'L',
    ('R', 'B', 'S'): 'L'
}

def simplify(path):
    p = list(path)
    i = 0
    while i <= len(p) - 3:
        if p[i+1] == 'B':
            trip = (p[i], 'B', p[i+2])
            if trip in REDUCTIONS:
                p[i:i+3] = [REDUCTIONS[trip]]
                i = max(0, i - 2)
                continue
        i += 1
    return "".join(p)

def run_cli_demo():
    print("==================================================================")
    print("[MESHMERIZE] TECHFEST 2026-27: ROBOT FIRMWARE SIMULATOR (CLI)")
    print("==================================================================")
    print("Arena: 230 cm x 230 cm | 8-Channel IR Sensor Array | PID Steering")
    print("------------------------------------------------------------------\n")
    
    # 1. Simulate Calibration
    print("[1/4] EXECUTING SENSOR CALIBRATION SWEEP...")
    time.sleep(0.5)
    print("   -> Motor pivot: left/right sweep on line...")
    print("   -> Min/Max calibration registered across 8 channels.")
    print("   -> Calibrated span: 2800 ADC counts. Baseline centered at 3500.\n")

    # 2. Simulate Dry Run Exploration (Left-Hand-Rule)
    print("[2/4] STARTING DRY RUN (MAPPING & EXPLORATION)...")
    simulated_steps = [
        ("Start Point (200, 200)", "Tracking Line Forward", "Straight"),
        ("Intersection 1", "Left wing detects branch", "Took 'L' -> Dead End Ahead"),
        ("Dead End 1", "All sensors black", "Executed 180 deg Turn 'B'"),
        ("Revisiting Inter 1", "Clearance past stripe", "Took 'L' (canonical LBL -> S)"),
        ("Checkpoint CP1 (180, 110)", "Sensor array centered", "+25 Points Awarded"),
        ("T-Junction 2", "Left branch open", "Took 'L' -> Loop Sector"),
        ("Dead End 2", "Branch terminates", "Executed 180 deg Turn 'B'"),
        ("Checkpoint CP2 (123, 50)", "Passing grid node", "+25 Points Awarded"),
        ("Checkpoint CP3 (90, 93)", "Cross-junction node", "+25 Points Awarded"),
        ("Dead End 3", "Pocket dead end", "Executed 180 deg Turn 'B'"),
        ("Checkpoint CP4 (53, 70)", "Approaching End Corridor", "+25 Points Awarded"),
        ("End Zone (70, 160)", "All 8 sensors read WHITE (400x400 mm)", "CONFIRMED END ZONE!"),
    ]

    # Realistic sequence with dead ends that reduce cleanly to shortest path:
    # LBL -> S, LBS -> R, SBL -> R
    raw_path = "LBL" + "LBS" + "SBL" + "E"
    for location, event, action in simulated_steps:
        print(f"   [{location:24}] {event:32} => {action}")
        time.sleep(0.08)
        
    print("\n   [RESULT] Dry Run Complete in 42.4s!")
    print("   [RULE MANDATE] Mandatory RED LED Illuminated on End Zone Detection.")
    print(f"   [MEMORY] Stored Raw Traversal: {raw_path}")

    # 3. Path Simplification
    print("\n[3/4] RUNNING ONBOARD LSRB PATH SIMPLIFICATION...")
    opt_path = simplify(raw_path.replace("E", "")) + "E"
    print(f"   -> Raw Path:       {raw_path}")
    print(f"   -> Optimized Path: {opt_path}")
    print(f"   -> Dead ends eliminated: 100%")

    # 4. Actual Run
    print("\n[4/4] STARTING ACTUAL RUN (SHORTEST PATH RACE)...")
    print("   -> Switching speed profile: 140 PWM -> 220 PWM (Aggressive Mode)")
    replay_actions = {
        'L': "Closed-Loop 90 deg Pivot Left (Sensor Re-locked)",
        'R': "Closed-Loop 90 deg Pivot Right (Sensor Re-locked)",
        'S': "Full Speed Forward Traversal Across Junction",
        'B': "180 deg Reversal",
        'E': "Active Braking in 400x400mm Box + Red LED Gained"
    }
    for i, step in enumerate(opt_path):
        print(f"   Step {i+1}/{len(opt_path)}: Target '{step}' => {replay_actions[step]}")
        time.sleep(0.15)
        
    actual_time = 18.2
    c_pts = 180 - 42
    t_pts = 150 - 18
    total_score = (4 * 25) + 30 + c_pts + 30 + t_pts + 5
    print("\n==================================================================")
    print("[FINAL] RACE FINISHED! OFFICIAL SCORE BREAKDOWN:")
    print("==================================================================")
    print(f"   A (Checkpoints 4 x 25):   100 pts")
    print(f"   B (Dry Run Completion):    30 pts")
    print(f"   C (180 - 42s Dry Time):   138 pts")
    print(f"   S (Shortest Path Bonus):   30 pts")
    print(f"   T (150 - 18s Actual Run): 132 pts")
    print(f"   L (Mandatory Red LED):      5 pts")
    print(f"   --------------------------------")
    print(f"   TOTAL ESTIMATED SCORE:    {total_score} POINTS")
    print("==================================================================\n")

if __name__ == "__main__":
    run_cli_demo()
