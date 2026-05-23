"""
================================================================
plotter.py — Real-time histogram display (Step 12)

Displays:
  - One subplot per active channel
  - One combined subplot (sum of all channels)

Uses matplotlib with interactive mode so the window stays open
while data continues to be collected.
================================================================
"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from typing import List


# Use TkAgg or Qt5Agg for interactive windows.
# Falls back to default if not available.
try:
    matplotlib.use("TkAgg")
except Exception:
    pass

# Colour palette — one per channel + combined
CHANNEL_COLOURS = [
    "#378ADD",   # blue
    "#D85A30",   # coral
    "#1D9E75",   # teal
    "#BA7517",   # amber
    "#7F77DD",   # purple
    "#639922",   # green
]
COMBINED_COLOUR = "#444444"


class HistogramPlotter:
    def __init__(self,
                 channels: List[int],
                 bins: int = 512,
                 energy_max: float = 3000.0):

        self._channels   = channels
        self._bins       = bins
        self._energy_max = energy_max

        n_ch = len(channels)
        # Layout: per-channel subplots + 1 combined
        n_plots = n_ch + 1

        self._fig, self._axes = plt.subplots(
            n_plots, 1,
            figsize=(10, 3 * n_plots),
            sharex=True
        )
        if n_plots == 1:
            self._axes = [self._axes]

        self._fig.suptitle("RADIOROC2 — Energy Spectrum",
                           fontsize=13, fontweight="bold")

        # Initialise subplots
        self._bars_per_ch = {}
        for i, ch in enumerate(channels):
            ax    = self._axes[i]
            color = CHANNEL_COLOURS[i % len(CHANNEL_COLOURS)]
            centres = np.linspace(0, energy_max, bins)
            bars = ax.bar(centres, np.zeros(bins),
                          width=energy_max / bins,
                          color=color, alpha=0.75,
                          label=f"Channel {ch}")
            ax.set_ylabel("Counts", fontsize=9)
            ax.set_title(f"Channel {ch}", fontsize=10)
            ax.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))
            ax.legend(loc="upper right", fontsize=8)
            self._bars_per_ch[ch] = bars

        # Combined subplot (last)
        ax_comb = self._axes[-1]
        centres = np.linspace(0, energy_max, bins)
        self._bars_combined = ax_comb.bar(
            centres, np.zeros(bins),
            width=energy_max / bins,
            color=COMBINED_COLOUR, alpha=0.6,
            label="Combined"
        )
        ax_comb.set_xlabel("Energy (keV)", fontsize=10)
        ax_comb.set_ylabel("Counts", fontsize=9)
        ax_comb.set_title("All channels combined", fontsize=10)
        ax_comb.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))
        ax_comb.legend(loc="upper right", fontsize=8)

        self._fig.tight_layout(rect=[0, 0, 1, 0.96])
        plt.ion()   # interactive mode — non-blocking
        plt.show()

    # ----------------------------------------------------------
    def update(self, session):
        """
        Update all histogram bars from the current DAQ session.
        Call this when histogram_ready is received.
        """
        # Per-channel
        for i, ch in enumerate(self._channels):
            centres, counts = session.get_histogram(ch)
            bars = self._bars_per_ch[ch]
            for bar, h in zip(bars, counts):
                bar.set_height(h)

            # Auto-scale Y
            ax = self._axes[i]
            ax.set_ylim(0, max(counts.max() * 1.1, 1))
            ax.set_title(
                f"Channel {ch}  "
                f"({session.events_per_channel(ch)} events)",
                fontsize=10
            )

        # Combined
        centres, counts = session.get_combined_histogram()
        for bar, h in zip(self._bars_combined, counts):
            bar.set_height(h)
        ax_comb = self._axes[-1]
        ax_comb.set_ylim(0, max(counts.max() * 1.1, 1))
        ax_comb.set_title(
            f"All channels combined  "
            f"({session.total_events()} events total)",
            fontsize=10
        )

    # ----------------------------------------------------------
    def show(self):
        """Flush matplotlib draw queue — call after update()."""
        self._fig.canvas.draw()
        self._fig.canvas.flush_events()

    # ----------------------------------------------------------
    def save(self, path: str):
        """Save current histogram figure to a file (PNG, PDF)."""
        self._fig.savefig(path, dpi=150, bbox_inches="tight")
        print(f"[PLOT] Saved histogram to {path}")

    # ----------------------------------------------------------
    def close(self):
        plt.ioff()
        plt.close(self._fig)
