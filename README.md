# RAGS
Risk-Aware Graph Search (RAGS) code for testing and release.

RAGS is a search technique for planning paths through a graph with uncertain edge costs.

This code release is in conjunction with our paper submission to the IJRR special issue on WAFR2016.

Usage:
  - from the command line, navigate to the build directory and run
    cmake ..
    make
    ./test_path_comparisons
  - RAGS domination threshold is determined by the value of PTHRESH, which is defined in main_Planner_Comparisons.cpp
  - toggle values on lines 63 and 64 of main_Planner_Comparisons.cpp to generate new random graphs or read in stored configurations
