# Ride Pooling Optimization

Work in progress.

This code reimplements [Alonso-Mora et al. (2017)](https://doi.org/10.1073/pnas.1611675114), which optimizes a ridesourcing fleet's dispatch and use of ridesplitting, and extends it to consider the private costs of operation and the unpriced external costs of emissions and traffic.

It uses (with permission) code from [MetaZuo/RideSharing](https://github.com/MetaZuo/RideSharing) as a starting point, and extends that code to include performance enhancements including some parallelization.

Dependencies:
*   C++17 and the C++ Standard Template Library
*   [OpenMP](https://www.openmp.org/)
*   flat_set.hpp from [Boost C++ Libraries](https://www.boost.org/)
*   [High Performance C++11 Serialization Library](https://github.com/jl2922/hps)
*   [METIS - Serial Graph Partitioning and Fill-reducing Matrix Ordering](http://glaros.dtc.umn.edu/gkhome/metis/metis/overview)
