# GTM Morphing Simulation

A simulation built on NASA's GTM_DesignSim, extended with morphing-related dynamics and control elements for morphing aircraft research. This is the actively maintained version; [`gtm-morphing-sim-legacy`](https://github.com/jihoonlee91/gtm-morphing-sim-legacy) is an earlier, superseded snapshot kept for reference.

This repository is used as the shared simulation base across several of the morphing-aircraft projects in this account (e.g. [`gtm-lpv-control`](https://github.com/jihoonlee91/gtm-lpv-control)), rather than being tied to a single paper.

## Base simulation: GTM_DesignSim (NASA)

The GTM_DesignSim is a batch simulation intended for design and analysis of flight control laws. It models the
dynamics of a 5.5% scale model of a generic transport aircraft. It was developed to allow fault accomodating control
algorithms to be developed and refined in simulation before being tested on an experimental subscale model.

See:

Cunningham, K., Cox, D. E., Murri, D. G., and Riddick, S. E., *"A Piloted Evaluation of Damage Accommodating Flight Control Using a Remotely Piloted Vehicle,"* **AIAA-2011-6451**, AIAA Guidance, Navigation, and Control Conference and Exhibit, Portland Oregon, August 2011.

and the references therein for a summary of the research.

The simulation, however, was released as open-source and has been found useful in many other applications where a
non-linear large-envelope flight dynamics simulation is required.

## What Was Added for Morphing Research

- `gtm_design/aero_analysis.m`, `gtm_design/analysis/compare_control.m`: aerodynamic and controller-comparison analysis added on top of the base NASA sim
- `gtm_design/controller.m`: morphing-aircraft control law
- `gtm_design/plot_result.m`: result plotting for the morphing configuration
- `gtm_design/example_sfunction.m`: example custom S-function, in addition to NASA's original `example1.m`–`example6.m`

## Repository Structure

- `gtm_design/setup.m`: adds the required MATLAB paths (run this first)
- `gtm_design/main.m`: top-level simulation entry point
- `gtm_design/config/`: aircraft/damage/noise parameters and the polynomial aero database (`T2_polynomial_aerodatabase.mat`)
- `gtm_design/mfiles/`: simulation utilities (trim, linearization, workspace helpers)
- `gtm_design/libs/`: Simulink library blocks (`UserContent_lib.mdl`)
- `gtm_design/obj/`: precompiled MEX binaries (Windows/macOS/Linux) for the S-function blocks
- `gtm_design/docs/`: original NASA GTM_DesignSim documentation (quick start, FAQ, reference, global tags)
- `gtm_design/gtm_design.slx`: the Simulink model (`.slx.original` and `.slx.r2018b` are earlier-format copies kept for compatibility)

## How to Run

Open MATLAB, `cd` into `gtm_design`, run `setup.m`, then `main.m`. See `gtm_design/docs/gtm_design_QuickStart.doc` for the original NASA quick-start guide.

Development environment: MATLAB / Simulink
