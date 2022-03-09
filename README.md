# RidePooling

Work in progress; additional documentation to come when the code is complete. 

This code reimplements Alonso-Mora et al. (2017)^[Alonso-Mora J et al. On-demand high-capacity ride-sharing via dynamic trip-vehicle assignment. PNAS, 2017, 114(3): 462-467.], which optimizes a ridesourcing fleet's dispatch and use of ridesplitting, and extends it to consider the private costs of operation and the unpriced external costs of emissions and traffic.

It uses (with permission) code from [MetaZuo/RideSharing](https://github.com/MetaZuo/RideSharing) as a starting point.

Dependencies:
*   C++17 and the C++ Standard Template Library
*   [OpenMP](https://www.openmp.org/)
*   flat_set.hpp from [Boost C++ Libraries](https://www.boost.org/)
*   [High Performance C++11 Serialization Library](https://github.com/jl2922/hps)

