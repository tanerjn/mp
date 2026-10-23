#  Shared plots
import matplotlib.pyplot as plt


def plot_avoidance(times, nominal, maneuvered, nominal_distance, maneuvered_distance, required_separation, t_burn):
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
        
    axes[0].plot(times, nominal_distance, label="No burn")
    axes[0].plot(times, maneuvered_distance, label="Optimized burn")
    axes[0].axhline(required_separation, color="red", linestyle="--", label="Required separation")
    axes[0].axvline(t_burn, color="grey", linestyle=":", label="Burn time")
    axes[0].set_xlabel("Time [s]")
    axes[0].set_ylabel("Separation [m]")
    axes[0].legend()

    axes[1].plot(nominal[:, 0], nominal[:, 1], label="No burn")
    axes[1].plot(maneuvered[:, 0], maneuvered[:, 1], label="Optimized burn")
    axes[1].scatter(0, 0, color="red", label="Threat")
    axes[1].set_xlabel("Radial position [m]")
    axes[1].set_ylabel("Along-track position [m]")
    axes[1].set_title("In-plane projection")
    axes[1].axis("equal")
    axes[1].legend()

    plt.tight_layout()
    return fig
