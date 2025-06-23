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
Elastic displacements that are smaller than 0.005f * L are ignored.

## Rigid-plastic approximation based on one dimensional beam analysis 
can be done based on only moment calculation
 * calculate elastic displacement using scenario where moment of Wp is applied at tip of clamped beam
   * y=M*x^2/(2*EI) e.g. Tekniikan käsikirja 1, page 52.
 * if displacement is < 0.1*L this model can be used, Beam::isValid
 * compare against e.g. https://civilengineeronline.com/cecalc.php
   * https://civilengineeronline.com/str/sdcantm.php  
   
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

## Wood
Model earlywood and latewood so that earlywood breaks first and joint can be seen broken as wood does.

 * annual ring 1-2 mm, 75 % earlywood and 25 % latewood
 * Y about 10 GPa, earlywood about half of latewood so could be modelled as 8 GPa for earlywood and 16 for latewood.
 * fy about 100 MPa, earlywood about 1/6
 * variation could be modelled to be about 15 %

Some references for these figures
 * https://puuinfo.fi/puutieto/puun-ominaisuuksia/lujuusteknisia-ominaisuuksia/
 * https://www.theseus.fi/bitstream/handle/10024/11739/2008-04-15-12.pdf
