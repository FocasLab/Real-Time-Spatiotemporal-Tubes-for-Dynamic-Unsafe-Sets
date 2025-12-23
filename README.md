# Real-Time-Spatiotemporal-Tubes-for-Dynamic-Unsafe-Sets
This repository contains the MATLAB implementation of the paper:
R. Das, S. Upadhyay, and P. Jagtap,
“Real-Time Spatiotemporal Tubes for Dynamic Unsafe Sets”
IEEE Robotics and Automation Letters (RA-L), 2025
DOI: 10.1109/LRA.2025.3645667

Repository Contents
main2d.m:	Real-time STT implementation for a single-integrator system in a 2D environment
main3d.m:	Real-time STT implementation for a double-integrator system in a 3D environment
control_2d.m:	STT-based controller for the 2D system
control_3d.m:	STT-based controller for the 3D system
smoothmin.m:	Smooth minimum function used for adaptive tube radius computation
createRandomObstacleMatrix.m:	Utility function to generate random dynamic obstacles

Requirements
MATLAB R2024b or newer
No additional toolboxes required

# Citation

If you use this code in your research, please cite:

@article{Das2025RAL,
  author  = {R. Das and S. Upadhyay and P. Jagtap},
  title   = {Real-Time Spatiotemporal Tubes for Dynamic Unsafe Sets},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2025},
  doi     = {10.1109/LRA.2025.3645667},
  url     = {https://ieeexplore.ieee.org/document/11302778}
}

# Contact
Ratnangshu Das
Indian Institute of Science (IISc), Bengaluru
Email: ratnangshud@iisc.ac.in

# License
This repository is provided for research and educational use.
Please contact the authors for other usage