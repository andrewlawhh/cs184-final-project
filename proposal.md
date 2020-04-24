![](https://cs184.eecs.berkeley.edu/uploads/article_images/32_1.jpg)

## 3D Position Based Fluid Simulation and Surfacing

Team Members: Andrew Law, Austin Cheng, Merryle Wang
We are trying to simulate 3D fluids using a particle based method. After getting the particles’ positions based on time, we will use those particles to build a surface and render it like real water. 

## Problem Description

Here you should provide the context for your idea. Describe the problem that you are trying to solve, why it is important, where it is challenging. Give us a general idea on how you are going to solve it.

According to the research paper we are basing most of the project off of, “Fluids, in particular liquids such as water, are responsible for many visually rich phenomena.” We are trying to simulate water that looks realistic, but doesn’t take an insane amount of computational resources. Since we are making an animation, where images need to be rendered quickly, a particle based method that is simple and allows large time steps is suitable. The particular challenge is enforcing incompressibility because that takes a lot of computation. The research paper by Macklin and Muller goes through an algorithm to enforce constant density.

Additionally, we will need to create a rendered fluid surface over our base particles, and we plan to try a variety of methods, in particular ellipsoid splatting.


## Goals and Deliverables

We plan to deliver a tool that allows the realistic rendering of water in a simulation. Our demo will include videos of flowing water through different surfaces. Water has many unique properties, so we plan to investigate what kinds of methods can make the water rendering most realistic. In addition to realistic rendering, we also need to consider computation speed, so we will need to consider the tradeoff between accuracy and speed among the different methods. 

We will measure the quality of our system in terms of visual aesthetics as well as simulation latency. If the simulation lags a lot, it might mean that our algorithms that implement the particle physics were too inefficient, or perhaps we have a memory leak somewhere. Visual aesthetics come in as a qualitative metric. Questions we hope to answer with our analysis include: 
Does the simulation look and “feel” like a real fluid? Does the simulation lag?

If things go well and we are ahead of schedule, stretch goals we may try to achieve are being able to interact with the simulation in real time. This means that a user would be able to apply different forces using some sort of GUI (sliders, drag and drop models, etc.) and have the simulation be able to react accordingly. For instance, as a user, maybe I could disable the force of gravity to see what would happen to the water. Something else I could do is maybe select a solid model and drop it into the water to see what kind of splashes and disturbances occur in the simulation.

## Schedule

Find good set of tools/skeleton code to have a framework from which to build on (April 14)

Code up particle physics (April 21)

Work on rendering so that it looks like water (April 28)

Work on presentation (May 7)

## Resources

[Position Based Fluids, Macklin & Muller (2007) ](http://mmacklin.com/pbf_sig_preprint.pdf)

[Ellipsoid Splatting](http://www.cs.rug.nl/~roe/courses/acg/rendering)

[Rendering Water] (http://developer.download.nvidia.com/presentations/2010/gdc/Direct3D_Effects.pdf)

OpenGL

