import json
import matplotlib.pyplot as plt
import numpy as np
import sys
import matplotlib.ticker as ticker

def load_data(filename):
    with open(filename, 'r') as f:
        data = json.load(f)
    return data

def group_results(results, experiment_type):
    """
    Group experiments by # of rescue robots deployed.
    experiment_type: "vr_rr" for experiments with vine + rescue robots,
                     "rr_only" for experiments with only rescue robots.
    """
    groups = {}
    for result in results:
        active = result.get("active_robots", {})
        has_vr = active.get("vr", False)
        # Filter by experiment type.
        if experiment_type == "vr_rr" and not has_vr:
            continue
        if experiment_type == "rr_only" and has_vr:
            continue
        
        num_rr = result.get("num_rr_deployed", 0)
        coverage = result.get("total_coverage", None)
        if coverage is None:
            continue
        groups.setdefault(num_rr, []).append(coverage)
    return groups

def compute_statistics(groups):
    xs = sorted(groups.keys())
    means = []
    stds = []
    for x in xs:
        arr = np.array(groups[x])
        means.append(np.mean(arr))
        stds.append(np.std(arr))
    return xs, means, stds
from matplotlib.widgets import Button

def plot_seed_results_by_seed(results):
    """
    Group the results by seed. We assume that every three consecutive
    results in 'results' correspond to one seed run:
       - One with both vine and rescue (VR+RR),
       - One with only vine (VR Only),
       - One with only rescue (RR Only).

    Then create an interactive plot showing the coverage over time curves
    for each type for a given seed. Two buttons (Prev, Next) allow moving
    through the seeds.
    """
    seeds = []
    n = len(results)
    if n % 3 != 0:
        print("Warning: number of results is not a multiple of 3!")
    n_seeds = n // 3
    for i in range(n_seeds):
        seed_res = {}
        # We assume that each group of 3 results corresponds to the three types.
        for res in results[3*i:3*i+3]:
            active = res.get("active_robots", {})
            vr = active.get("vr", False)
            rr = active.get("rr", False)
            if vr and rr:
                seed_res["VR+RR"] = res
            elif vr and not rr:
                seed_res["VR Only"] = res
            elif not vr and rr:
                seed_res["RR Only"] = res
        seeds.append(seed_res)

    current_seed = 0

    # Create the initial figure and axis.
    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.2)

    def plot_seed(seed_idx):
        ax.clear()
        seed_data = seeds[seed_idx]
        # For each type in this seed, sort its coverage data and plot.
        for key, res in seed_data.items():
            cov = res.get("coverage_over_time", [])
            if cov:
                cov_sorted = sorted(cov, key=lambda pt: pt[0])
                times = [pt[0] for pt in cov_sorted]
                coverages = [pt[1] for pt in cov_sorted]
                ax.plot(times, coverages, label=key)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Coverage (%)")
        ax.set_title(f"Seed {seed_idx}")
        ax.legend()
        fig.canvas.draw_idle()

    plot_seed(current_seed)

    # Create axes for the Prev and Next buttons.
    axprev = plt.axes([0.3, 0.05, 0.1, 0.075])
    axnext = plt.axes([0.6, 0.05, 0.1, 0.075])
    bprev = Button(axprev, 'Prev')
    bnext = Button(axnext, 'Next')

    def prev(event):
        nonlocal current_seed
        current_seed = (current_seed - 1) % len(seeds)
        plot_seed(current_seed)

    def next(event):
        nonlocal current_seed
        current_seed = (current_seed + 1) % len(seeds)
        plot_seed(current_seed)

    bprev.on_clicked(prev)
    bnext.on_clicked(next)

    plt.show()
def plot_aggregated_coverage_over_time(results):
    """
    For each experiment type, plot the individual coverage–time curves
    (light blue for VR+RR and orange for RR Only) along with the aggregated
    mean curve and standard deviation error band.
    """
    groups = {"VR+RR": [], "VR Only": [], "RR Only": []}
    for res in results:
        active = res.get("active_robots", {})
        has_vr = active.get("vr", False)
        has_rr = active.get("rr", False)
        cov = res.get("coverage_over_time", [])
        if cov:
            cov_sorted = sorted(cov, key=lambda pt: pt[0])
            times = [pt[0] for pt in cov_sorted]
            coverages = [pt[1] for pt in cov_sorted]
            if has_vr and has_rr:
                groups["VR+RR"].append((times, coverages))
            elif has_vr and not has_rr:
                groups["VR Only"].append((times, coverages))
            elif not has_vr and has_rr:
                groups["RR Only"].append((times, coverages))

    
    plt.figure(figsize=(10,6))
    for label, series_list in groups.items():
        if not series_list:
            continue
        
        if label == "VR+RR":
            ind_color = "lightblue"
            agg_color = "blue"
        elif label == "VR Only":
            ind_color = "tomato"
            agg_color = "red"
        elif label == "RR Only":
            ind_color = "palevgreen"
            agg_color = "green"
        
        # Plot the individual time series.
        for times, coverages in series_list:
            plt.plot(times, coverages, color=ind_color, alpha=0.5, linewidth=1, label="_nolegend_")
        
        # Compute aggregated (mean and std) values.
        # We assume all experiments share the same time stamps.
        times = series_list[0][0]
        coverage_matrix = np.array([s[1] for s in series_list])
        mean_coverage = np.mean(coverage_matrix, axis=0)
        std_coverage = np.std(coverage_matrix, axis=0)
        
        # Plot the aggregated mean and its error band.
        plt.plot(times, mean_coverage, color=agg_color, label=f"{label} (n={len(series_list)})", linewidth=2)
        plt.fill_between(times, mean_coverage - std_coverage, mean_coverage + std_coverage,
                         color=agg_color, alpha=0.3)
    
    ax = plt.gca()
    ax.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, pos: f"{x:.2f} s"))
    
    plt.xlabel("Time (s)")
    plt.ylabel("Coverage (%)")
    plt.title("Aggregated Coverage Over Time")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


def main():
    if len(sys.argv) > 1:
        filename = sys.argv[1]

    data = load_data(filename)
    results = data.get("results", [])
        
    plot_seed_results_by_seed(results)

if __name__ == "__main__":
    main()