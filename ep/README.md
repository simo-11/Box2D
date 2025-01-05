# Introduction
ep is main contribution of this fork of Box2D.
It contains various attempts to handle stiff elasticity and plasticity within Box2D.

# Attempt using modified constraints

This path was abandoned mainly due to 

 * spring based constrains are often too soft
 * contraint solver is too slow even for weld joints

Changes in core file are documented in [changes_to_core_files.md](changes_to_core_files.md)
More (old, main work was done 2017) details in https://docs.google.com/document/d/1oACqdqwr3fm4IfKLPMXScVii1fb_uPqTpFyLie3QOy4/

# Attempt to use modification of rigid bodies using Beam

# Test scenarios

## Static elastic bending

## sample_joints.cpp:EpCantilever

# Beam with input L,w,h,E,fy,density
Beam is based on b2Polygon and material properties.
Elastic displacements that are smaller than B2_LINEAR_SLOP ( 0.005f * b2_lengthUnitsPerMeter ) are ignored.

## Rigid-plastic approximation based on one dimensional beam analysis 
can be done based on only moment calculation
 * calculate elastic displacement using scenario where moment of Wp is applied at tip of clamped beam
 * if displacement is < B2_LINEAR_SLOP accept this approximation
   
More complex processes have been developed, one example is https://www.jstor.org/stable/43634039 published in 1953.

## update method

## Elastic
 * I=w*h^3/12, https://en.wikipedia.org/wiki/List_of_second_moments_of_area
 * displacement due to own weight q=A*rho, q*x*x*(6*L*L-4*L*x+x*x)/(24*E*I), https://en.wikipedia.org/wiki/Euler%E2%80%93Bernoulli_beam_theory

## Plastic
 * Wp=w*h^2/4

## Based on minimal 2D FE-solver

Meshing e.g. c++ implementation of https://github.com/mapbox/delaunator / https://mapbox.github.io/delaunator/ at https://github.com/delfrrr/delaunator-cpp

### Solve

Using e.g. [Eigen](https://eigen.tuxfamily.org/) sample e.g. https://github.com/podgorskiy/MinimalFem
