import json
import matplotlib.pyplot as plt
import numpy as np
import sys
import matplotlib.ticker as ticker
from matplotlib.widgets import Button

# Set matplotlib parameters for IEEE-style formatting (Helvetica, appropriate font sizes)
plt.rcParams.update({
    "font.family": "Helvetica",
    "font.size": 8,
    "axes.labelsize": 8,
    "axes.titlesize": 8,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "legend.fontsize": 8,
})

# Updated color map using the specified colors.
COLOR_MAP = {
    "Vine Robot Only": "#89023e",
    "RESCUE Rollers Only (one per 1 min)": "#7eb77f",
    "RESCUE Rollers Only (all per 5 sec)": "#ef8354",
    "Vine and RESCUE Rollers (one per 1 min)": "#4f5d75",
    "Vine and RESCUE Rollers (on retraction)": "#d6d5b3"
}

def load_data(filename):
    with open(filename, 'r') as f:
        content = f.read()
    lines = content.splitlines()
    new_lines = []
    for line in lines:
        if '"final_heat_map":' in line and not line.rstrip().endswith(','):
            new_lines.append(line.rstrip() + ',')
        else:
            new_lines.append(line)
    fixed_content = "\n".join(new_lines)
    try:
        data = json.loads(fixed_content)
        return data
    except json.JSONDecodeError as e:
        print("Error decoding JSON file:", e)
        sys.exit(1)

def get_experiment_type(res):
    active = res.get("active_robots", {})
    vr = active.get("vr", False)
    rr = active.get("rr", False)
    dt = res.get("delta_drop_time", None)
    if vr and not rr:
        return "Vine Robot Only"
    elif not vr and rr:
        if dt == 60:
            return "RESCUE Rollers Only (one per 1 min)"
        elif dt == 5:
            return "RESCUE Rollers Only (all per 5 sec)"
        else: return "Error? 1"
    elif vr and rr:
        if dt == 0: return "Vine and RESCUE Rollers (on retraction)"
        elif dt == 60: return "Vine and RESCUE Rollers (one per 1 min)"
        else: return "Error"
    return "Unknown"

def group_by_experiment(results):
    groups = {}
    for res in results:
        etype = get_experiment_type(res)
        groups.setdefault(etype, []).append(res)
    return groups

def plot_seed(results):
    n_results = len(results)
    n_exp = 0
    if n_results % 4 == 0:
        n_exp = 4
    elif n_results % 5 == 0:
        n_exp = 5
    else: n_exp = None

    if n_exp is not None:
        seeds = []
        n_seeds = n_results // n_exp
        for i in range(n_seeds):
            seed_res = results[i*n_exp:(i+1)*n_exp]
            seeds.append(seed_res)
        current_seed = 0
        fig, ax = plt.subplots()
        plt.subplots_adjust(bottom=0.2)
        def plot_seed_inner(seed_idx):
            ax.clear()
            seed_data = seeds[seed_idx]
            for res in seed_data:
                etype = get_experiment_type(res)
                cov = res.get("coverage_over_time", [])
                if cov:
                    cov_sorted = sorted(cov, key=lambda pt: pt[0])
                    times = [pt[0] for pt in cov_sorted]
                    times_minutes = [t/60 for t in times]
                    coverages = [pt[1] for pt in cov_sorted]
                    ax.plot(times_minutes, coverages, label=etype, color=COLOR_MAP.get(etype, None))
            ax.set_xlabel("Time (min)")
            ax.set_ylabel("Coverage (%)")
            ax.set_title(f"Seed {seed_idx}")
            ax.legend()
            ax.axvspan(0,10,facecolor='lightpink',alpha=0.5)
            fig.canvas.draw_idle()
        plot_seed_inner(current_seed)
        axprev = plt.axes([0.3, 0.05, 0.1, 0.075])
        axnext = plt.axes([0.6, 0.05, 0.1, 0.075])
        bprev = Button(axprev, 'Prev')
        bnext = Button(axnext, 'Next')
        def prev(event):
            nonlocal current_seed
            current_seed = (current_seed - 1) % len(seeds)
            plot_seed_inner(current_seed)
        def next(event):
            nonlocal current_seed
            current_seed = (current_seed + 1) % len(seeds)
            plot_seed_inner(current_seed)
        bprev.on_clicked(prev)
        bnext.on_clicked(next)
        plt.show()
    else:
        fig, ax = plt.subplots()
        for res in results:
            etype = get_experiment_type(res)
            cov = res.get("coverage_over_time", [])
            if cov:
                cov_sorted = sorted(cov, key=lambda pt: pt[0])
                times = [pt[0] for pt in cov_sorted]
                times_minutes = [t/60 for t in times]
                coverages = [pt[1] for pt in cov_sorted]
                ax.plot(times_minutes, coverages, label=etype, color=COLOR_MAP.get(etype, None))
        ax.set_xlabel("Time (min)")
        ax.set_ylabel("Coverage (%)")
        ax.set_title("Coverage by Experiment Type")
        ax.legend()
        ax.axvspan(0,10,facecolor='lightpink',alpha=0.5)
        plt.show()

def plot_coverage(results):
    groups = {}
    for res in results:
        etype = get_experiment_type(res)
        groups.setdefault(etype, []).append(res)
    plt.figure(figsize=(10,6))
    for etype, res_list in groups.items():
        color = COLOR_MAP.get(etype, None)
        series_list = []
        for res in res_list:
            cov = res.get("coverage_over_time", [])
            if cov:
                cov_sorted = sorted(cov, key=lambda pt: pt[0])
                times = [pt[0] for pt in cov_sorted]
                coverages = [pt[1] for pt in cov_sorted]
                series_list.append((times, coverages))
        if series_list:
            for times, coverages in series_list:
                times_minutes = [t/60 for t in times]
            times = series_list[0][0]
            times_minutes = [t/60 for t in times]
            coverage_matrix = np.array([s[1] for s in series_list])
            mean_coverage = np.mean(coverage_matrix, axis=0)
            std_coverage = np.std(coverage_matrix, axis=0)
            plt.plot(times_minutes, mean_coverage, color=color, label=f"{etype}", linewidth=2)
            plt.fill_between(times_minutes, mean_coverage-std_coverage, mean_coverage+std_coverage, color=color, alpha=0.2)

    ax = plt.gca()
    ax.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x,pos: f"{x:.2f} min"))
    plt.xlabel("Time (min)")
    plt.ylabel("Coverage (%)")
    plt.title("Aggregated Coverage Over Time")
    plt.grid(True)
    plt.legend()
    ax.axvspan(0,10,facecolor='lightpink',alpha=0.3)
    plt.tight_layout()
    plt.show()

def plot_found_times_table(results): # Group runs by experiment type 
    groups = {} 
    for res in results: 
        etype = get_experiment_type(res) 
        if "person_found_times" in res: 
            groups.setdefault(etype, []).append(res["person_found_times"])

    table_data = []
    # For each experiment type, average the survivors percentage
    for etype, runs in groups.items():
        if len(runs) == 0:
            avg_percent = "N/A"
        else:
            percentages = []
            for run in runs:
                count = sum(1 for t in run if t > 0)
                percentage = (count / 4) * 100  
                percentages.append(percentage)
            avg_percent = np.mean(percentages)
            avg_percent = f"{avg_percent:.2f}%"
        table_data.append([etype, avg_percent])

    fig, ax = plt.subplots(figsize=(6,2))
    ax.axis('tight')
    ax.axis('off')
    col_labels = ["Experiment Type", "Survivors Found (%)"]
    table = ax.table(cellText=table_data, colLabels=col_labels, loc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(12)
    table.scale(1, 2)
    plt.title("Average Survivors Found by Experiment Type")
    plt.show()

def main():
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        sys.exit(1)
    data = load_data(filename)
    results = data.get("results", [])
    plot_seed(results)
    plot_coverage(results)
    plot_found_times_table(results)

if __name__ == "__main__":
    main()
