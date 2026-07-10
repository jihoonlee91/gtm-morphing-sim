# GTM_DesignSim_Morphing

모핑 항공기(morphing aircraft) 연구를 위해 NASA GTM_DesignSim을 기반으로 형상 변형(morphing) 관련 동역학/제어 요소를 추가한 시뮬레이션입니다.

## Base simulation: GTM_DesignSim (NASA)

The GTM_DesignSim is a batch simulation intended for design and analysis of flight control laws. It models the
dynamics of a 5.5% scale model of a generic transport aircraft. It was developed to allow fault accomodating control
algorithms to be developed and refined in simulation before being tested on an experimental subscale model.

See:

Cunningham, K., Cox, D. E., Murri, D. G., and Riddick, S. E., *"A Piloted Evaluation of Damage Accommodating Flight Control Using a Remotely Piloted Vehicle,"* **AIAA-2011-6451**, AIAA Guidance, Navigation, and Control Conference and Exhibit, Portland Oregon, August 2011.

and the references therein for a summary of the research.

The simulation, however, was released as open-source and has been found useful in many other applications where a
non-linear large-envelope flight dynamics simulation is required.
