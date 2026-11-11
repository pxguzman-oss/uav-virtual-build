"""Lumped-parameter thermal network (nodes with heat capacity, conductances between nodes and to
ambient, heat sources), integrated with implicit Euler over a timeline of phases.

Deliberately small: the uncertainty of the inputs (convection coefficients in a closed printed bay,
ESC losses, sun load) is +-30-50 %, so the stage that uses this runs a nominal and a pessimistic
network instead of pretending to more resolution."""
from __future__ import annotations

import numpy as np

AIR_K, AIR_NU, AIR_PR, AIR_CP = 0.0263, 1.6e-5, 0.71, 1005.0


def h_forced_plate(V, L, rho_ratio=1.0):
    """Average flat-plate convection coefficient [W/m2K] (laminar below Re 5e5, turbulent above)."""
    if V <= 0.3:
        return 0.0
    Re = V * L / AIR_NU * rho_ratio
    Nu = 0.664 * Re ** 0.5 * AIR_PR ** (1 / 3) if Re < 5e5 else 0.037 * Re ** 0.8 * AIR_PR ** (1 / 3)
    return Nu * AIR_K / L


class Network:
    def __init__(self):
        self.names, self.C = [], []

    def node(self, name, C):
        if name in self.names:
            raise ValueError(f"duplicate thermal node {name}")
        self.names.append(name)
        self.C.append(max(float(C), 1e-3))
        return len(self.names) - 1

    def idx(self, name):
        return self.names.index(name)

    def simulate(self, phases, T0, dt=2.0):
        """phases: [{name, duration, T_amb, links: [(a, b, G)], amb: [(a, G)], Q: {node: W}}].
        Returns (t, T[n_steps, n_nodes], phase_index[n_steps])."""
        n = len(self.names)
        C = np.array(self.C)
        T = np.array(T0, float).copy()
        ts, Ts, ph = [0.0], [T.copy()], [0]
        t = 0.0
        for k, p in enumerate(phases):
            L = np.zeros((n, n))
            b = np.zeros(n)
            for a_, b_, g in p.get("links", []):
                i, j = self.idx(a_), self.idx(b_)
                L[i, i] += g; L[j, j] += g; L[i, j] -= g; L[j, i] -= g
            for a_, g in p.get("amb", []):
                i = self.idx(a_)
                L[i, i] += g
                b[i] += g * p["T_amb"]
            for a_, q in (p.get("Q") or {}).items():
                b[self.idx(a_)] += q
            steps = max(1, int(round(p["duration"] / dt)))
            h = p["duration"] / steps
            A = np.diag(C / h) + L
            Ainv = np.linalg.inv(A)
            for _ in range(steps):
                T = Ainv @ (C / h * T + b)
                t += h
                ts.append(t); Ts.append(T.copy()); ph.append(k)
        return np.array(ts), np.array(Ts), np.array(ph)

    def steady(self, phase):
        n = len(self.names)
        L = np.zeros((n, n)); b = np.zeros(n)
        for a_, b_, g in phase.get("links", []):
            i, j = self.idx(a_), self.idx(b_)
            L[i, i] += g; L[j, j] += g; L[i, j] -= g; L[j, i] -= g
        for a_, g in phase.get("amb", []):
            i = self.idx(a_); L[i, i] += g; b[i] += g * phase["T_amb"]
        for a_, q in (phase.get("Q") or {}).items():
            b[self.idx(a_)] += q
        return np.linalg.solve(L + 1e-12 * np.eye(n), b)
