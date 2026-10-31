"""Shared matplotlib style (validated categorical palette, light background) for every stage figure."""
import matplotlib

matplotlib.use("Agg")

PALETTE = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
INK, INK2, GRID, BG = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
STATUS_COL = {"PASS": "#1baf7a", "WARN": "#eda100", "FAIL": "#e34948", "INFO": "#8a8984", "ERROR": "#6250d6"}


def style():
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "figure.facecolor": BG, "axes.facecolor": BG, "axes.edgecolor": INK2, "savefig.facecolor": BG,
        "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2, "axes.grid": True,
        "grid.color": GRID, "grid.linewidth": 0.8, "lines.linewidth": 1.6, "font.size": 9,
        "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False,
        "axes.prop_cycle": matplotlib.cycler(color=PALETTE), "axes.titleweight": "bold", "axes.titlesize": 10,
    })
    return plt


def save(fig, path, dpi=120):
    import matplotlib.pyplot as plt
    fig.tight_layout()
    fig.savefig(path, dpi=dpi)
    plt.close(fig)
    return path
