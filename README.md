<div align="center">

# GTM Morphing Simulation

**NASA's Generic Transport Model simulation, extended for morphing-wing flight dynamics and control research.**

![MATLAB](https://img.shields.io/badge/MATLAB-0076A8?style=flat-square&logo=mathworks&logoColor=white)
![Simulink](https://img.shields.io/badge/Simulink-0076A8?style=flat-square)
[![Built on NASA GTM_DesignSim](https://img.shields.io/badge/built%20on-NASA%20GTM__DesignSim-e03c31?style=flat-square&logo=nasa&logoColor=white)](https://github.com/nasa/GTM_DesignSim)
![License](https://img.shields.io/badge/license-NASA%20Open%20Source%20Agreement-lightgrey?style=flat-square)

<img src="https://jihoonlee91.github.io/assets/figures/2023-self-scheduled-lpv-asymmetric-morphing-uav-1.webp" width="520" alt="NASA GTM-T2 subscale aircraft">

<sub>NASA GTM-T2, the 5.5 %-scale generic transport aircraft this simulation models</sub>

</div>

## Why

Morphing-wing control research needs a nonlinear, large-envelope flight simulation that can be trimmed, linearized, and closed-loop tested. NASA's open-source **GTM_DesignSim** provides exactly that for a subscale transport aircraft; this repository adds the morphing-specific dynamics, control law, and analysis on top of it, and serves as the **shared simulation base** for the morphing projects in this account (e.g. [`gtm-lpv-control`](https://github.com/jihoonlee91/gtm-lpv-control)).

## What's added for morphing

| File | Purpose |
|---|---|
| `gtm_design/controller.m` | morphing-aircraft control law |
| `gtm_design/aero_analysis.m` | aerodynamic analysis of morphing configurations |
| `gtm_design/analysis/compare_control.m` | side-by-side controller comparison |
| `gtm_design/plot_result.m` | result plots for the morphing configuration |
| `gtm_design/example_sfunction.m` | example custom S-function (alongside NASA's `example1.m`–`example6.m`) |

## Requirements

- MATLAB R2020b or later with Simulink (`gtm_design.slx` was saved in R2020b; `.slx.r2018b` opens in R2018b)
- Control System Toolbox (`ss`, `damp` in the analysis and example scripts)
- Precompiled S-function MEX binaries for Windows, macOS, and Linux are in `gtm_design/obj/`, so no C compiler is needed

## Quick start

```matlab
cd gtm_design
setup   % add the required paths (run first)
main    % top-level simulation entry point
```

NASA's original quick-start guide is in `gtm_design/docs/gtm_design_QuickStart.doc`.

<details>
<summary><b>Repository layout</b></summary>

- `gtm_design/setup.m` — adds the required MATLAB paths
- `gtm_design/main.m` — top-level simulation entry point
- `gtm_design/config/` — aircraft / damage / noise parameters and the polynomial aero database (`T2_polynomial_aerodatabase.mat`)
- `gtm_design/mfiles/` — simulation utilities (trim, linearization, workspace helpers)
- `gtm_design/libs/` — Simulink library blocks (`UserContent_lib.mdl`)
- `gtm_design/obj/` — precompiled MEX binaries (Windows / macOS / Linux) for the S-function blocks
- `gtm_design/docs/` — original NASA GTM_DesignSim documentation
- `gtm_design/gtm_design.slx` — the Simulink model (`.slx.original` and `.slx.r2018b` kept for compatibility)

</details>

## Research built on the GTM model

| Publication | Code |
|---|---|
| [Modeling and Simulation of Asymmetric Variable-Span Morphing Aircraft](https://jihoonlee91.github.io/papers/2019-asymmetric-morphing-modeling-sim-ko.html) — *KSAS Fall 2019 (in Korean)* | — |
| [Flight Control of Asymmetric Variable-Span Morphing Aircraft Using Nonlinear Dynamic Inversion](https://jihoonlee91.github.io/papers/2019-flight-control-nonlinear-dynamic-inversion-gold.html) — *APISAT 2019, Gold Award* | — |
| [Aircraft Autopilot Design Using Asymmetric Variable-Span Wing Morphing](https://jihoonlee91.github.io/papers/2021-aircraft-autopilot-asymmetric-morphing.html) — *ICAS 2021* | — |
| [Self-Scheduled LPV Control of Asymmetric Variable-Span Morphing UAV](https://jihoonlee91.github.io/papers/2023-self-scheduled-lpv-asymmetric-morphing-uav.html) — *Sensors 2023* | [`gtm-lpv-control`](https://github.com/jihoonlee91/gtm-lpv-control) |
| [Self-Scheduled Parameter-Varying Control of Asymmetric Variable-Span Morphing UAV](https://jihoonlee91.github.io/papers/2023-phd-dissertation-lpv-asymmetric-morphing.html) — *Ph.D. dissertation, SNU 2023* | [`gtm-lpv-control`](https://github.com/jihoonlee91/gtm-lpv-control) |

## License and credits

Base simulation: [NASA GTM_DesignSim](https://github.com/nasa/GTM_DesignSim). This repository is a derivative of it and is distributed under the NASA Open Source Agreement 1.3 — see [`LICENSE.md`](LICENSE.md) and `LicenseAgreement.pdf`. GitHub's **Cite this repository** button uses [`CITATION.cff`](CITATION.cff). Background:

> Cunningham, K., Cox, D. E., Murri, D. G., and Riddick, S. E., "A Piloted Evaluation of Damage Accommodating Flight Control Using a Remotely Piloted Vehicle," AIAA-2011-6451, AIAA GNC Conference, Portland, OR, 2011.

<sub>Part of my Ph.D. research at the Seoul National University Flight Dynamics and Control Lab — more at <a href="https://jihoonlee91.github.io/">jihoonlee91.github.io</a>.</sub>
