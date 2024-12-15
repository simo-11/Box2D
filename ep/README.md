# Introduction
ep is main contribution of this fork of Box2D.
It contains various attempts to handle stiff elasticity and plasticity within Box2D.

# Attempt using modified constraints

This path was abandoned mainly due to 

 * spring based constrains are often too soft
 * contraint solver is too slow even for weld joints

Changes in core file are documented in [changes_to_core_files.md](changes_to_core_files.md)
More (old, main work was done 2017) details in https://docs.google.com/document/d/1oACqdqwr3fm4IfKLPMXScVii1fb_uPqTpFyLie3QOy4/

# Attempt to use modification of rigid bodies

# Test scenarios

## sample_joints.cpp:Cantilever
