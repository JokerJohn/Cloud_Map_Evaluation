import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.colors import LinearSegmentedColormap
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.ticker as ticker
from matplotlib.patches import Ellipse
import matplotlib.colors as colors
import matplotlib.cm as cmx
from scipy.stats import multivariate_normal


import matplotlib
matplotlib.use('TkAgg')  # or another interactive backend
# Set up matplotlib to use LaTeX
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Times"],
    "font.size": 20,
    "axes.grid": True,
    "grid.linestyle": "-",
    "grid.linewidth": 0.2,
    "axes.grid.axis": "both",
    "legend.fontsize": 22,
    "legend.frameon": True,
    "legend.framealpha": 0.8,
    "legend.facecolor": "white",
    "legend.edgecolor": "black",
    "xtick.labelsize": 22,
    "ytick.labelsize": 22,
    "axes.labelsize": 22,
    "axes.linewidth": 1.0,
    # "axes.labelweight": "normal",
})


def read_data(filename):
    data = np.loadtxt(filename, delimiter=' ')
    return data

def plot_error_histogram(errors, output_file):
    plt.figure(figsize=(10, 8))
    plt.hist(errors, bins=50, edgecolor='black')
    plt.xlabel('Wasserstein Distance Error')
    plt.ylabel('Number of Voxels')
    plt.title('Histogram of Wasserstein Distance Errors',  pad=10)
    mean_error = np.mean(errors)
    std_error = np.std(errors)
    three_sigma = 3 * std_error
    plt.axvline(mean_error + three_sigma, color='r', linestyle='dashed', linewidth=2)
    mask = errors <= (mean_error + three_sigma)
    avg_error_within_bound = np.mean(errors[mask])
    plt.text(0.7, 0.95, f'(3$\sigma$): {mean_error + three_sigma:.2f}',
             transform=plt.gca().transAxes, fontsize=20,
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output_file, format='pdf', dpi=600, bbox_inches='tight')
    plt.show()
    plt.close()
    print(f"Average WD error within 3-sigma: {avg_error_within_bound:.4f}")
    print(f"3-sigma value: {mean_error + three_sigma:.4f}")

def plot_planar_error(data, output_file, plane='xy'):
    # Extract data
    if plane == 'xy':
        voxel_min_x, voxel_min_y = data[:, 0], data[:, 1]
        voxel_max_x, voxel_max_y = data[:, 3], data[:, 4]
        mean_x, mean_y = data[:, 6], data[:, 7]
    elif plane == 'xz':
        voxel_min_x, voxel_min_y = data[:, 0], data[:, 2]
        voxel_max_x, voxel_max_y = data[:, 3], data[:, 5]
        mean_x, mean_y = data[:, 6], data[:, 8]
    errors = data[:, 9]  # Wasserstein distance error
    est_points = data[:, 11]
    gt_points = data[:, 10]
    fig, ax = plt.subplots(figsize=(10, 8))
    # Create custom colormap: blue (low error) -> red (high error)
    cmap = LinearSegmentedColormap.from_list("custom", ["#4575B4", "#FFFFBF", "#D73027"])
    norm = plt.Normalize(vmin=errors.min(), vmax=errors.max())
    # Create custom colormap for point count ratio: red -> green -> blue
    ratio_cmap = LinearSegmentedColormap.from_list("ratio", ["red", "green", "blue"])
    ratios = est_points / gt_points
    ratio_norm = plt.Normalize(vmin=ratios.min(), vmax=ratios.max())
    # Use a set to keep track of unique voxels
    unique_voxels = set()
    for i in range(len(mean_x)):
        voxel_key = (voxel_min_x[i], voxel_min_y[i])
        if voxel_key not in unique_voxels:
            unique_voxels.add(voxel_key)
            rect = Rectangle((voxel_min_x[i], voxel_min_y[i]),
                             voxel_max_x[i] - voxel_min_x[i],
                             voxel_max_y[i] - voxel_min_y[i],
                             facecolor=cmap(norm(errors[i])), edgecolor='black', linewidth=0)
            ax.add_patch(rect)
            # Plot mean point with color based on point count ratio
            # ax.plot(mean_x[i], mean_y[i], 'o', color=ratio_cmap(ratio_norm(ratios[i])),
            #         markeredgecolor='white', markeredgewidth=0.5, markersize=1, alpha=0.8)
    plt.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap), label='Wasserstein Distance Error')
    if plane == 'xy':
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Voxel Error Visualization (XY)', pad=10)
    elif plane == 'xz':
        plt.xlabel('X (m)')
        plt.ylabel('Z (m)')
        plt.title('Voxel Error Visualization (XZ)', pad=10)
    # Set the extent of the plot to match the voxel grid
    ax.set_xlim(voxel_min_x.min(), voxel_max_x.max())
    ax.set_ylim(voxel_min_y.min(), voxel_max_y.max())
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output_file, format='pdf', dpi=600, bbox_inches='tight')
    plt.show()
    plt.close()


def plot_planar_error_combined(data, output_file):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 10))  # Increased figure height
    # Create custom colormap: blue (low error) -> red (high error)
    cmap = LinearSegmentedColormap.from_list("custom", ["#4575B4", "#FFFFBF", "#D73027"])
    norm = plt.Normalize(vmin=data[:, 9].min(), vmax=data[:, 9].max())
    # Create custom colormap for point count ratio: red -> green -> blue
    ratio_cmap = LinearSegmentedColormap.from_list("ratio", ["red", "green", "blue"])
    ratios = data[:, 10] / data[:, 11]
    ratio_norm = plt.Normalize(vmin=ratios.min(), vmax=ratios.max())
    for idx, (ax, plane) in enumerate([(ax1, 'xy'), (ax2, 'xz')]):
        if plane == 'xy':
            voxel_min_x, voxel_min_y = data[:, 0], data[:, 1]
            voxel_max_x, voxel_max_y = data[:, 3], data[:, 4]
            mean_x, mean_y = data[:, 6], data[:, 7]
        elif plane == 'xz':
            voxel_min_x, voxel_min_y = data[:, 0], data[:, 2]
            voxel_max_x, voxel_max_y = data[:, 3], data[:, 5]
            mean_x, mean_y = data[:, 6], data[:, 8]
        errors = data[:, 9]  # Wasserstein distance error
        # Use a set to keep track of unique voxels
        unique_voxels = set()
        for i in range(len(mean_x)):
            voxel_key = (voxel_min_x[i], voxel_min_y[i])
            if voxel_key not in unique_voxels:
                unique_voxels.add(voxel_key)
                rect = Rectangle((voxel_min_x[i], voxel_min_y[i]),
                                 voxel_max_x[i] - voxel_min_x[i],
                                 voxel_max_y[i] - voxel_min_y[i],
                                 facecolor=cmap(norm(errors[i])), edgecolor='black', linewidth=0.5)
                ax.add_patch(rect)
                # Plot mean point with color based on point count ratio
                ax.plot(mean_x[i], mean_y[i], 'o', color=ratio_cmap(ratio_norm(ratios[i])),
                        markeredgecolor='white', markeredgewidth=0.5, markersize=4, alpha=0.8)
        ax.set_xlim(voxel_min_x.min(), voxel_max_x.max())
        ax.set_ylim(voxel_min_y.min(), voxel_max_y.max())
        ax.grid(True)
        if plane == 'xy':
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_title('Voxel Error Visualization (XY)', pad=20)
        elif plane == 'xz':
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Z (m)')
            ax.set_title('Voxel Error Visualization (XZ)', pad=20)
    # Adjust the layout to make room for the colorbar
    plt.tight_layout(rect=[0, 0.1, 1, 1])  # Adjust the bottom value to create space for colorbar
    # Add a colorbar below the subplots
    cbar_ax = fig.add_axes([0.1, 0.08, 0.8, 0.04])  # [left, bottom, width, height]
    cbar = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap), cax=cbar_ax, orientation='horizontal')
    cbar.set_label('Wasserstein Distance Error')
    plt.savefig(output_file, format='pdf', dpi=600, bbox_inches='tight')
    plt.show()
    plt.close()



# Modified function to create a combined 2x2 plot
def plot_combined_2x2(data, output_file):
    fig = plt.figure(figsize=(16, 12))  # Create a figure
    # Custom colormap
    cmap = LinearSegmentedColormap.from_list("custom", ["#4575B4", "#FFFFBF", "#D73027"])
    norm = plt.Normalize(vmin=data[:, 9].min(), vmax=data[:, 9].max())
    ratio_cmap = LinearSegmentedColormap.from_list("ratio", ["red", "green", "blue"])
    ratios = data[:, 10] / data[:, 11]
    ratio_norm = plt.Normalize(vmin=ratios.min(), vmax=ratios.max())

    # XY Plane Plot
    ax_xy = fig.add_subplot(2, 2, 1)
    voxel_min_x, voxel_min_y = data[:, 0], data[:, 1]
    voxel_max_x, voxel_max_y = data[:, 3], data[:, 4]
    mean_x, mean_y = data[:, 6], data[:, 7]
    errors = data[:, 9]
    unique_voxels = set()
    for i in range(len(mean_x)):
        voxel_key = (voxel_min_x[i], voxel_min_y[i])
        if voxel_key not in unique_voxels:
            unique_voxels.add(voxel_key)
            rect = Rectangle((voxel_min_x[i], voxel_min_y[i]),
                             voxel_max_x[i] - voxel_min_x[i],
                             voxel_max_y[i] - voxel_min_y[i],
                             facecolor=cmap(norm(errors[i])), edgecolor='black', linewidth=0.5)
            ax_xy.add_patch(rect)
            # Plot mean point with color based on point count ratio
            ax_xy.plot(mean_x[i], mean_y[i], 'o', color=ratio_cmap(ratio_norm(ratios[i])),
                       markeredgecolor='white', markeredgewidth=0.5, markersize=1, alpha=0.8)
    ax_xy.set_xlim(voxel_min_x.min(), voxel_max_x.max())
    ax_xy.set_ylim(voxel_min_y.min(), voxel_max_y.max())
    ax_xy.grid(True)
    ax_xy.set_xlabel('X (m)')
    ax_xy.set_ylabel('Y (m)')

    # XZ Plane Plot
    ax_xz = fig.add_subplot(2, 2, 2)
    voxel_min_x, voxel_min_y = data[:, 0], data[:, 2]
    voxel_max_x, voxel_max_y = data[:, 3], data[:, 5]
    mean_x, mean_y = data[:, 6], data[:, 8]
    errors = data[:, 9]
    unique_voxels = set()
    for i in range(len(mean_x)):
        voxel_key = (voxel_min_x[i], voxel_min_y[i])
        if voxel_key not in unique_voxels:
            unique_voxels.add(voxel_key)
            rect = Rectangle((voxel_min_x[i], voxel_min_y[i]),
                             voxel_max_x[i] - voxel_min_x[i],
                             voxel_max_y[i] - voxel_min_y[i],
                             facecolor=cmap(norm(errors[i])), edgecolor='black', linewidth=0.5)
            ax_xz.add_patch(rect)
            # Plot mean point with color based on point count ratio
            ax_xz.plot(mean_x[i], mean_y[i], 'o', color=ratio_cmap(ratio_norm(ratios[i])),
                       markeredgecolor='white', markeredgewidth=0.5, markersize=1, alpha=0.8)
    ax_xz.set_xlim(voxel_min_x.min(), voxel_max_x.max())
    ax_xz.set_ylim(voxel_min_y.min(), voxel_max_y.max())
    ax_xz.grid(True)
    ax_xz.set_xlabel('X (m)')
    ax_xz.set_ylabel('Z (m)')

    # YZ Plane Plot
    ax_yz = fig.add_subplot(2, 2, 3)
    voxel_min_x, voxel_min_y = data[:, 1], data[:, 2]
    voxel_max_x, voxel_max_y = data[:, 4], data[:, 5]
    mean_x, mean_y = data[:, 7], data[:, 8]
    errors = data[:, 9]
    unique_voxels = set()
    for i in range(len(mean_x)):
        voxel_key = (voxel_min_x[i], voxel_min_y[i])
        if voxel_key not in unique_voxels:
            unique_voxels.add(voxel_key)
            rect = Rectangle((voxel_min_x[i], voxel_min_y[i]),
                             voxel_max_x[i] - voxel_min_x[i],
                             voxel_max_y[i] - voxel_min_y[i],
                             facecolor=cmap(norm(errors[i])), edgecolor='black', linewidth=0.5)
            ax_yz.add_patch(rect)
            # Plot mean point with color based on point count ratio
            ax_yz.plot(mean_x[i], mean_y[i], 'o', color=ratio_cmap(ratio_norm(ratios[i])),
                       markeredgecolor='white', markeredgewidth=0.5, markersize=1, alpha=0.8)
    ax_yz.set_xlim(voxel_min_x.min(), voxel_max_x.max())
    ax_yz.set_ylim(voxel_min_y.min(), voxel_max_y.max())
    ax_yz.grid(True)
    ax_yz.set_xlabel('Y (m)')
    ax_yz.set_ylabel('Z (m)')
    # 3D plot in the lower right corner
    ax3d = fig.add_subplot(2, 2, 4, projection='3d')
    # ax3d.set_box_aspect([1, 1, 1])  # Set equal aspect ratio for 3D plot
    voxel_min_x, voxel_min_y, voxel_min_z = data[:, 0], data[:, 1], data[:, 2]
    voxel_max_x, voxel_max_y, voxel_max_z = data[:, 3], data[:, 4], data[:, 5]
    mean_x, mean_y, mean_z = data[:, 6], data[:, 7], data[:, 8]
    errors = data[:, 9]
    est_points = data[:, 10]
    gt_points = data[:, 11]
    ratios = est_points / gt_points
    unique_voxels = set()
    for i in range(len(mean_x)):
        voxel_key = (voxel_min_x[i], voxel_min_y[i], voxel_min_z[i])
        if voxel_key not in unique_voxels:
            unique_voxels.add(voxel_key)
            # Plot voxel
            dx = voxel_max_x[i] - voxel_min_x[i]
            dy = voxel_max_y[i] - voxel_min_y[i]
            dz = voxel_max_z[i] - voxel_min_z[i]
            ax3d.bar3d(voxel_min_x[i], voxel_min_y[i], voxel_min_z[i],
                       dx, dy, dz,
                       color=cmap(norm(errors[i])),  # Color based on Wasserstein distance error
                       alpha=0.8, shade=True)
            # Plot mean point
            ax3d.scatter(mean_x[i], mean_y[i], mean_z[i],
                         c=ratio_cmap(ratio_norm(ratios[i])),  # Color based on point count ratio
                         s=20, edgecolors='white', linewidths=0.5)
    ax3d.set_xlabel('X (m)', labelpad=20)
    ax3d.set_ylabel('Y (m)', labelpad=20)
    ax3d.set_zlabel('Z (m)', labelpad=20)
    ax3d.set_frame_on(False)  # Remove 2D frame around 3D plot
    # Adjust the layout to make room for the colorbar
    plt.tight_layout(rect=[0, 0.1, 1, 1])
    # Add a colorbar below the subplots
    cbar_ax = fig.add_axes([0.1, 0.08, 0.8, 0.02])  # [left, bottom, width, height]
    cbar = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap), cax=cbar_ax, orientation='horizontal')
    cbar.set_label('Wasserstein Distance Error')
    plt.savefig(output_file, format='pdf', dpi=600, bbox_inches='tight')
    plt.show()
    plt.close()

def plot_point_count_ratio(data, output_file):
    voxel_min_x, voxel_min_y = data[:, 0], data[:, 1]
    voxel_max_x, voxel_max_y = data[:, 3], data[:, 4]
    est_points = data[:, 11]
    # gt_points = data[:, 11]
    ratios = est_points
    fig, ax = plt.subplots(figsize=(10, 8))
    # Create custom colormap: red -> green -> blue
    cmap = LinearSegmentedColormap.from_list("custom", ["red", "green", "blue"])
    norm = plt.Normalize(vmin=ratios.min(), vmax=ratios.max())
    # Use a set to keep track of unique voxels
    unique_voxels = set()
    for i in range(len(voxel_min_x)):
        voxel_key = (voxel_min_x[i], voxel_min_y[i])
        if voxel_key not in unique_voxels:
            unique_voxels.add(voxel_key)
            rect = Rectangle((voxel_min_x[i], voxel_min_y[i]),
                             voxel_max_x[i] - voxel_min_x[i],
                             voxel_max_y[i] - voxel_min_y[i],
                             facecolor=cmap(norm(ratios[i])), edgecolor='black', linewidth=0.5)
            ax.add_patch(rect)
    # Add colorbar with formatted tick labels
    cbar = plt.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap), label='Est/GT Point Count Ratio')
    cbar.ax.yaxis.set_major_formatter(ticker.FuncFormatter(lambda x, p: f'{x:.1e}' if x >= 1000 else f'{x:.0f}'))
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Voxel Point Count Visualization (XY)', pad=10)
    # Set the extent of the plot to match the voxel grid
    ax.set_xlim(voxel_min_x.min(), voxel_max_x.max())
    ax.set_ylim(voxel_min_y.min(), voxel_max_y.max())
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output_file, format='pdf', dpi=600, bbox_inches='tight')
    plt.show()
    plt.close()

def plot_3d_error(data, output_file):
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    voxel_min_x, voxel_min_y, voxel_min_z = data[:, 0], data[:, 1], data[:, 2]
    voxel_max_x, voxel_max_y, voxel_max_z = data[:, 3], data[:, 4], data[:, 5]
    mean_x, mean_y, mean_z = data[:, 6], data[:, 7], data[:, 8]
    errors = data[:, 9]  # Wasserstein distance error
    est_points = data[:, 10]
    gt_points = data[:, 11]
    # Create custom colormap: blue (low error) -> red (high error)
    cmap = LinearSegmentedColormap.from_list("custom", ["#4575B4", "#FFFFBF", "#D73027"])
    norm = plt.Normalize(vmin=errors.min(), vmax=errors.max())
    # Create custom colormap for point count ratio: red -> green -> blue
    ratio_cmap = LinearSegmentedColormap.from_list("ratio", ["red", "green", "blue"])
    ratios = est_points / gt_points
    ratio_norm = plt.Normalize(vmin=ratios.min(), vmax=ratios.max())

    # Use a set to keep track of unique voxels
    unique_voxels = set()
    for i in range(len(mean_x)):
        voxel_key = (voxel_min_x[i], voxel_min_y[i], voxel_min_z[i])
        if voxel_key not in unique_voxels:
            unique_voxels.add(voxel_key)
            # Plot voxel
            dx = voxel_max_x[i] - voxel_min_x[i]
            dy = voxel_max_y[i] - voxel_min_y[i]
            dz = voxel_max_z[i] - voxel_min_z[i]
            ax.bar3d(voxel_min_x[i], voxel_min_y[i], voxel_min_z[i],
                     dx, dy, dz,
                     color=cmap(norm(errors[i])),  # Color based on Wasserstein distance error
                     alpha=0.8, shade=True)
            # Plot mean point
            ax.scatter(mean_x[i], mean_y[i], mean_z[i],
                       c=ratio_cmap(ratio_norm(ratios[i])),  # Color based on point count ratio
                       s=20, edgecolors='white', linewidths=0.5)
    ax.set_xlabel('X (m)', labelpad=20)
    ax.set_ylabel('Y (m)', labelpad=20)
    ax.set_zlabel('Z (m)', labelpad=20)
    ax.set_title('3D Voxel Error Visualization', pad=20)

    # for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
    #     axis.line.set_linewidth(2)
    #     axis.line.set_color('black')
    #     axis.set_pane_color((0.8, 0.8, 0.8, 1.0))

    # Add colorbar for Wasserstein distance error
    # cbar = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap),
    #                     ax=ax, pad=0.1, aspect=20, label='Wasserstein Distance Error')
    
    cax = fig.add_axes([ax.get_position().x1 + 0.02, ax.get_position().y0, 0.02, ax.get_position().height * 1.0])
    cbar = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap),pad=10, 
                        cax=cax, label='Wasserstein Distance Error')

    plt.tight_layout()
    plt.savefig(output_file, format='pdf', dpi=600, bbox_inches='tight')
    plt.show()
    plt.close()


def read_data_cov(filename):
    data = np.loadtxt(filename, delimiter=' ')
    means = data[:, 6:9]
    cov_00, cov_01, cov_02 = data[:, 12], data[:, 13], data[:, 14]
    cov_11, cov_12, cov_22 = data[:, 15], data[:, 16], data[:, 17]
    cov_matrices = np.array([[cov_00, cov_01, cov_02],
                             [cov_01, cov_11, cov_12],
                             [cov_02, cov_12, cov_22]]).transpose(2, 0, 1)
    weights = data[:, 11] / np.sum(data[:, 11])
    return means, cov_matrices, weights

def gmm_pdf(x, means, covs, weights):
    pdf = np.zeros(x.shape[:-1])
    for mean, cov, weight in zip(means, covs, weights):
        mvn = multivariate_normal(mean=mean, cov=cov)
        pdf += weight * mvn.pdf(x)
    return pdf
def safe_norm(vmin, vmax):
    if vmin == vmax:
        return colors.Normalize(vmin, vmax + 1)
    elif vmin > vmax:
        return colors.Normalize(vmax, vmin)
    else:
        return colors.Normalize(vmin, vmax)
def plot_gmm_2d(means, cov_matrices, weights, output_file, plane='xy'):
    fig, ax = plt.subplots(figsize=(10, 8))
    idx1, idx2 = (0, 1) if plane == 'xy' else (0, 2)
    x = np.linspace(means[:, idx1].min() - 1, means[:, idx1].max() + 1, 200)
    y = np.linspace(means[:, idx2].min() - 1, means[:, idx2].max() + 1, 200)
    X, Y = np.meshgrid(x, y)
    pos = np.dstack((X, Y))
    Z = gmm_pdf(pos, means[:, [idx1, idx2]], cov_matrices[:, [idx1, idx2]][:, :, [idx1, idx2]], weights)
    # Handle zero or negative values
    Z = np.maximum(Z, np.finfo(float).eps)  # Replace zeros or negatives with a small positive value
    # Use logarithmic normalization for smoother visualization
    norm = colors.LogNorm(vmin=Z.min(), vmax=Z.max())
    contour = ax.pcolormesh(X, Y, Z, norm=norm, cmap='viridis', shading='auto')
    # Add contour lines for better visualization of the distribution
    levels = np.logspace(np.log10(Z.min()), np.log10(Z.max()), 20)
    ax.contour(X, Y, Z, levels=levels, colors='k', alpha=0.3, linewidths=0.5)
    ax.set_xlabel(f'{"X" if idx1 == 0 else "Z"} (m)')
    ax.set_ylabel(f'{"Y" if idx2 == 1 else "Z"} (m)')
    ax.set_title(f'GMM Probability Density ({plane.upper()} plane)')
    cbar = plt.colorbar(contour)
    cbar.set_label('Probability Density')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output_file, format='pdf', dpi=600, bbox_inches='tight')
    plt.show()
    plt.close()

def plot_gmm_3d(means, cov_matrices, weights, output_file):
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    x = np.linspace(means[:, 0].min() - 1, means[:, 0].max() + 1, 30)
    y = np.linspace(means[:, 1].min() - 1, means[:, 1].max() + 1, 30)
    z = np.linspace(means[:, 2].min() - 1, means[:, 2].max() + 1, 30)
    X, Y, Z = np.meshgrid(x, y, z)
    pos = np.stack((X, Y, Z), axis=-1)
    PDF = gmm_pdf(pos, means, cov_matrices, weights)
    # Use scatter plot with alpha based on PDF for 3D visualization
    points = pos.reshape(-1, 3)
    norm = safe_norm(PDF.min(), PDF.max())
    colors_rgba = plt.cm.viridis(norm(PDF.flatten()))
    colors_rgba[:, 3] = norm(PDF.flatten())  # Set alpha channel
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors_rgba)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D GMM Probability Density')
    # Add colorbar
    sm = plt.cm.ScalarMappable(cmap='viridis', norm=norm)
    sm.set_array([])
    cbar = plt.colorbar(sm)
    cbar.set_label('Probability Density')
    plt.tight_layout()
    plt.savefig(output_file, format='pdf', dpi=600, bbox_inches='tight')
    plt.show()  
    plt.close()

def visualize_gmm(filename, output_prefix):
    means, cov_matrices, weights = read_data_cov(filename)
    plot_gmm_2d(means, cov_matrices, weights, f'{output_prefix}_xy.pdf', plane='xy')
    plot_gmm_2d(means, cov_matrices, weights, f'{output_prefix}_xz.pdf', plane='xz')
    plot_gmm_3d(means, cov_matrices, weights, f'{output_prefix}_3d.pdf')

def plot_gaussian_comparison(data, output_file):
    # Find the voxel with the highest total point count (GT + EST)
    total_points = data[:, 10] + data[:, 11]
    max_index = np.argmax(total_points)
    # Extract data for the selected voxel
    est_mean = data[max_index, 6:9]
    gt_mean = data[max_index, 18:21]
    est_cov = np.array([[data[max_index, 12], data[max_index, 13], data[max_index, 14]],
                        [data[max_index, 13], data[max_index, 15], data[max_index, 16]],
                        [data[max_index, 14], data[max_index, 16], data[max_index, 17]]])
    gt_cov = np.array([[data[max_index, 21], data[max_index, 22], data[max_index, 23]],
                       [data[max_index, 22], data[max_index, 24], data[max_index, 25]],
                       [data[max_index, 23], data[max_index, 25], data[max_index, 26]]])
    # Create 2D plots for XY, XZ, and YZ planes
    planes = ['xy', 'xz', 'yz']
    for plane in planes:
        fig, ax = plt.subplots(figsize=(10, 8))
        if plane == 'xy':
            est_mean_2d, gt_mean_2d = est_mean[:2], gt_mean[:2]
            est_cov_2d, gt_cov_2d = est_cov[:2, :2], gt_cov[:2, :2]
            xlabel, ylabel = 'X', 'Y'
        elif plane == 'xz':
            est_mean_2d, gt_mean_2d = est_mean[::2], gt_mean[::2]
            est_cov_2d, gt_cov_2d = est_cov[::2, ::2], gt_cov[::2, ::2]
            xlabel, ylabel = 'X', 'Z'
        else:  # yz
            est_mean_2d, gt_mean_2d = est_mean[1:], gt_mean[1:]
            est_cov_2d, gt_cov_2d = est_cov[1:, 1:], gt_cov[1:, 1:]
            xlabel, ylabel = 'Y', 'Z'
        # Create a meshgrid
        x = np.linspace(min(est_mean_2d[0], gt_mean_2d[0]) - 1, max(est_mean_2d[0], gt_mean_2d[0]) + 1, 100)
        y = np.linspace(min(est_mean_2d[1], gt_mean_2d[1]) - 1, max(est_mean_2d[1], gt_mean_2d[1]) + 1, 100)
        X, Y = np.meshgrid(x, y)
        pos = np.dstack((X, Y))
        # Calculate PDF for EST and GT
        rv_est = multivariate_normal(est_mean_2d, est_cov_2d)
        rv_gt = multivariate_normal(gt_mean_2d, gt_cov_2d)
        Z_est = rv_est.pdf(pos)
        Z_gt = rv_gt.pdf(pos)
        # Plot contours
        levels = np.linspace(0, max(Z_est.max(), Z_gt.max()), 20)
        ax.contour(X, Y, Z_est, levels=levels, colors='r', alpha=0.5, linestyles='solid')
        ax.contour(X, Y, Z_gt, levels=levels, colors='b', alpha=0.5, linestyles='dashed')
        # Plot mean points
        ax.plot(est_mean_2d[0], est_mean_2d[1], 'ro', markersize=10, label='EST Mean')
        ax.plot(gt_mean_2d[0], gt_mean_2d[1], 'bo', markersize=10, label='GT Mean')
        ax.set_xlabel(f'{xlabel} (m)')
        ax.set_ylabel(f'{ylabel} (m)')
        ax.set_title(f'Gaussian Comparison ({plane.upper()} plane)')
        ax.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(f'{output_file}_{plane}.pdf', format='pdf', dpi=600, bbox_inches='tight')
        plt.show()  
        plt.close()


def main():
    data_file = 'voxel_errors.txt'  # Replace with your actual file name
    data = read_data(data_file)
    errors = data[:, 9]  # Assuming the error is in the 10th column now
    plot_error_histogram(errors, 'error_histogram.pdf')
    plot_planar_error(data, 'xy_planar_error.pdf', plane='xy')
    # plot_planar_error(data, 'xz_planar_error.pdf', plane='xz')

    # New combined plot
    # plot_planar_error_combined(data, 'combined_planar_error.pdf')
    # Example usage
    data2 = np.loadtxt(data_file, delimiter=' ')
    plot_combined_2x2(data2, 'combined_2x2_plot.pdf')

    plot_point_count_ratio(data, 'point_count_ratio.pdf')
    # Add the new 3D plot
    # plot_3d_error(data, '3d_voxel_error.pdf')

      # Add the new Gaussian comparison plot
    # plot_gaussian_comparison(data, 'gaussian_comparison')

        # Example usage
    # visualize_gmm('voxel_errors.txt', 'gmm_visualization')

if __name__ == "__main__":
    main()