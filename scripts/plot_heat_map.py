import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

# Set all default font sizes to 8 pt.
mpl.rcParams.update({
    'font.family': 'sans-serif',
    'font.sans-serif': ['Helvetica'],
    'font.size': 8,
    'axes.labelsize': 8,
    'axes.titlesize': 8,
    'xtick.labelsize': 8,
    'ytick.labelsize': 8,
    'legend.fontsize': 8,
    'figure.titlesize': 8
})

def plot_heat_map(heat_csv, occupancy_csv, fixed_min=20, fixed_max=37):
    # Load the heat map and occupancy grid from CSV files.
    heat = np.loadtxt(heat_csv, delimiter=',')
    occupancy = np.loadtxt(occupancy_csv, delimiter=',')

    # Create a custom colormap from white to dark red.
    # White corresponds to room temperature (20°C) and dark red to the hottest (37°C).
    cmap = LinearSegmentedColormap.from_list("custom_heat", ["#faf0ff", "#4a001e"], N=256)

    # Create a figure sized for one column (3.5" wide by 2.5" high).
    fig, ax = plt.subplots(figsize=(3.5, 2.5))
    
    # Display the heat map with extent such that [left, right, bottom, top] = [0,20,20,0].
    # This mapping ensures that the first row of data (index 0) is drawn at y=0 (top left).
    im = ax.imshow(heat, cmap=cmap, vmin=fixed_min, vmax=fixed_max,
                   extent=[0, 20, 20, 0], origin="upper")
    
    # Overlay walls: assume occupancy==0 are walls.
    wall_mask = np.where(occupancy == 0, 1.0, np.nan)
    wall_cmap = LinearSegmentedColormap.from_list("walls", ["black", "black"])
    ax.imshow(wall_mask, cmap=wall_cmap, extent=[0, 20, 20, 0],
              origin="upper", interpolation="none")

    # Set white background.
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')

    # Set axis labels and ticks with font size 8.
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_xticks(np.linspace(0, 20, 5))
    ax.set_yticks(np.linspace(0, 20, 5))
    ax.tick_params(labelsize=8)
    ax.grid(color='gray', linestyle='--', linewidth=0.5)

    # Add a vertical colorbar with label.
    cbar = fig.colorbar(im, ax=ax)
    cbar.set_label("Temperature (°C)")
    cbar.ax.tick_params(labelsize=8)

    plt.tight_layout()
    plt.savefig("heat_final.png", dpi=300, bbox_inches="tight")
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python plot_heat_map.py <heat_csv> <occupancy_csv>")
        sys.exit(1)
    heat_csv = sys.argv[1]
    occupancy_csv = sys.argv[2]
    plot_heat_map(heat_csv, occupancy_csv)
