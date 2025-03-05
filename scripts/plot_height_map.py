import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from mpl_toolkits.axes_grid1 import make_axes_locatable

# Force Helvetica as the font and all font sizes to be 8 pt.
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


def plot_height_map(height_csv, occupancy_csv, fixed_min=-1, fixed_max=1):
    # Load height map and occupancy grid from CSV files.
    height = np.loadtxt(height_csv, delimiter=',')
    occupancy = np.loadtxt(occupancy_csv, delimiter=',')

    # Define a continuous colormap from 7 stops.
    # Color stops (hex):
    # dark orange: "#580000", "#9c4511", "#dd8629", yellow: "#ffffe0",
    # "#3ea8a6", "#076769", blue: "#002c2d"
    # colors = ["#580000", "#9c4511", "#dd8629", "#ffffe0", "#3ea8a6", "#076769", "#002c2d"]
    # cmap = LinearSegmentedColormap.from_list("custom_height", colors, N=256)
    # Option 1: Reverse the list inline.
    colors = ["#580000", "#9c4511", "#dd8629", "#ffffe0", "#3ea8a6", "#076769", "#002c2d"]
    cmap = LinearSegmentedColormap.from_list("custom_height", colors[::-1], N=256)


    # Create a figure sized for one column (3.5" x 2.5") to match the heat map.
    fig, ax = plt.subplots(figsize=(3.5, 2.5))
    
    # Display the height map.
    # Using extent=[0,20,20,0] with origin="upper" makes (0,0) appear at the top left.
    im = ax.imshow(height, cmap=cmap, vmin=fixed_min, vmax=fixed_max,
                   extent=[0, 20, 20, 0], origin="upper")
    
    # Overlay walls: assume occupancy==0 indicates walls.
    wall_mask = np.where(occupancy == 0, 1.0, np.nan)
    wall_cmap = LinearSegmentedColormap.from_list("walls", ["black", "black"])
    ax.imshow(wall_mask, cmap=wall_cmap, extent=[0, 20, 20, 0],
              origin="upper", interpolation="none")
    
    # Set background to white.
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    
    # Set axis labels and ticks from 0 to 20 m.
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_xticks(np.linspace(0, 20, 5))
    ax.set_yticks(np.linspace(0, 20, 5))
    ax.tick_params(labelsize=8)
    ax.grid(color='gray', linestyle='--', linewidth=0.5)
    
    # Attach a vertical colorbar whose height exactly matches the image.
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.04)
    cbar = fig.colorbar(im, cax=cax)
    # Force the colorbar to show exactly 7 ticks.
    tick_locs = np.linspace(fixed_min, fixed_max, 7)
    cbar.set_ticks(tick_locs)
    cbar.set_label("Height (m)")
    cbar.ax.tick_params(labelsize=8)
    
    plt.tight_layout()
    plt.savefig("height_final.png", dpi=300, bbox_inches="tight")
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python plot_height_map.py <height_csv> <occupancy_csv>")
        sys.exit(1)
    height_csv = sys.argv[1]
    occupancy_csv = sys.argv[2]
    plot_height_map(height_csv, occupancy_csv)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python plot_height_map.py <height_csv> <occupancy_csv>")
        sys.exit(1)
    height_csv = sys.argv[1]
    occupancy_csv = sys.argv[2]
    plot_height_map(height_csv, occupancy_csv)
