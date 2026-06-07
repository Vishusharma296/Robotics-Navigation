import os
import glob
import re
import sys
import random
from datetime import datetime
import config
from simulation import RobotNavigationSimulation
from visualizer import SimulationVisualizer, save_gif


def generate_filename(output_dir):
    os.makedirs(output_dir, exist_ok=True)
    existing = glob.glob(os.path.join(output_dir, "RS-*.gif"))
    serial = 1
    for fpath in existing:
        m = re.search(r'RS-(\d+)-\d{8}_\d{6}\.gif', os.path.basename(fpath))
        if m:
            s = int(m.group(1))
            if s >= serial:
                serial = s + 1
    now = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(output_dir, f"RS-{serial}-{now}.gif"), serial


def main():
    sim_seed = random.randint(0, 999999)

    print("Generating pathable map...")
    try:
        sim = RobotNavigationSimulation(config, seed=sim_seed)
    except RuntimeError as e:
        print(f"Error: {e}")
        sys.exit(1)

    viz = SimulationVisualizer(config.M_L, config.M_B, scale=5,
                               panel_width=config.PANEL_WIDTH)
    viz.setup_figure()
    frames = []

    print(f"Running simulation (max {config.MAX_STEPS} ticks, seed={sim.seed})...")
    while not sim.done:
        done = sim.step()

        if sim.tick % config.FRAME_SKIP == 0 or done:
            img = viz.render(sim.get_state())
            frames.append(img)

        if done:
            break
        if sim.tick >= config.MAX_STEPS:
            break

    viz.close()

    out_path, serial = generate_filename(config.OUTPUT_DIR)
    save_gif(frames, out_path)
    status = "arrived" if sim.arrived else "timeout"
    print(f"Simulation finished: {sim.tick} ticks, {status}")
    print(f"  Re-plans: {sim.replan_count}, Seed: {sim.seed}")
    print(f"Frames captured: {len(frames)}")
    print(f"Saved to: {out_path}")


if __name__ == "__main__":
    main()
