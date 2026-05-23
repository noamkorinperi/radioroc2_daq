"""
================================================================
daq.py — DAQ session and histogram data management

Stores incoming events per channel and combined.
Builds histogram arrays ready for plotting.

Histogram bins are uniform in keV from 0 to energy_max.
================================================================
"""

import numpy as np
from typing import List, Dict


class DAQSession:
    def __init__(self,
                 channels: List[int],
                 bins: int = 512,
                 energy_max: float = 3000.0):
        """
        Parameters
        ----------
        channels    : list of active ASIC channel numbers
        bins        : number of histogram bins
        energy_max  : upper energy limit in keV
        """
        self._channels   = channels
        self._bins       = bins
        self._energy_max = energy_max

        # Bin edges (keV)
        self._edges = np.linspace(0.0, energy_max, bins + 1)

        # Per-channel event lists and histogram counts
        self._events:  Dict[int, List[float]] = {ch: [] for ch in channels}
        self._counts:  Dict[int, np.ndarray]  = {
            ch: np.zeros(bins, dtype=np.int64) for ch in channels
        }

        # Combined histogram
        self._counts_combined = np.zeros(bins, dtype=np.int64)
        self._total_events    = 0

    # ----------------------------------------------------------
    def add_event(self, channel: int, energy_kev: float):
        """Add one event to the appropriate channel and combined."""
        if channel not in self._events:
            return   # channel not in active list — ignore

        if energy_kev < 0.0 or energy_kev > self._energy_max:
            return   # out of range

        self._events[channel].append(energy_kev)

        # Find bin index
        idx = int(energy_kev / self._energy_max * self._bins)
        idx = min(idx, self._bins - 1)   # clamp

        self._counts[channel][idx]    += 1
        self._counts_combined[idx]    += 1
        self._total_events            += 1

    # ----------------------------------------------------------
    def get_histogram(self, channel: int):
        """
        Returns (bin_centres_kev, counts) for a specific channel.
        """
        centres = (self._edges[:-1] + self._edges[1:]) / 2.0
        return centres, self._counts[channel].copy()

    def get_combined_histogram(self):
        """
        Returns (bin_centres_kev, counts) for all channels combined.
        """
        centres = (self._edges[:-1] + self._edges[1:]) / 2.0
        return centres, self._counts_combined.copy()

    def total_events(self) -> int:
        return self._total_events

    def events_per_channel(self, channel: int) -> int:
        return len(self._events.get(channel, []))

    def channels(self) -> List[int]:
        return self._channels

    def bins(self) -> int:
        return self._bins

    def energy_max(self) -> float:
        return self._energy_max

    def reset(self):
        """Clear all accumulated data."""
        for ch in self._channels:
            self._events[ch].clear()
            self._counts[ch][:] = 0
        self._counts_combined[:] = 0
        self._total_events = 0
