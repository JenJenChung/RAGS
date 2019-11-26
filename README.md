# RAGS
Risk-Aware Graph Search (RAGS) code for testing and release.

RAGS is a search technique for planning paths through a graph with uncertain edge costs.

This code release is in conjunction with our paper submission to the IJRR special issue on WAFR2016.

```bibtex
@article{Chung2019risk, 
  author={Chung, Jen Jen and Smith, Andrew J and Skeele, Ryan and Hollinger, Geoffrey A.}, 
  title={Risk-aware graph search with dynamic edge cost discovery},
  journal={The International Journal of Robotics Research},
  year={2019},
  pages={182--195},
  volume={38},
  number={2--3},
  doi={10.1177/0278364918781009},
  URL={https://doi.org/10.1177/0278364918781009}
}
```

Usage:
  - from the command line, navigate to the build directory and run
    - cmake ..
    - make
    - ./test_path_comparisons
  - RAGS domination threshold is determined by the value of PTHRESH, which is defined in main_Planner_Comparisons.cpp
  - toggle values on lines 63 and 64 of main_Planner_Comparisons.cpp to generate new random graphs or read in stored configurations
