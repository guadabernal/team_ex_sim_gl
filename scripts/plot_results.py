import json
import matplotlib.pyplot as plt
import numpy as np

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

def main():
    filename = "merged_res_1.json"
    data = load_data(filename)
    results = data.get("results", [])
    
    # Group experiments by type.
    groups_vr_rr = group_results(results, "vr_rr")
    groups_rr_only = group_results(results, "rr_only")
    
    xs_vr_rr, means_vr_rr, stds_vr_rr = compute_statistics(groups_vr_rr)
    xs_rr_only, means_rr_only, stds_rr_only = compute_statistics(groups_rr_only)
    
    # Create the plot.
    plt.figure(figsize=(10,6))
    
    if xs_vr_rr:
        plt.errorbar(xs_vr_rr, means_vr_rr, yerr=stds_vr_rr, fmt='-o', capsize=5,
                     label="VR + RR")
    if xs_rr_only:
        plt.errorbar(xs_rr_only, means_rr_only, yerr=stds_rr_only, fmt='-s', capsize=5,
                     label="RR Only")
    
    plt.xlabel("# of Rescue Robots Deployed")
    plt.ylabel("% Coverage")
    plt.title("Coverage vs. Number of Rescue Robots Deployed")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
