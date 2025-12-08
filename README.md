# Feather

A Go physic library, based on the XPBD solver algorithm.

## XPBD
The current implementation simplifies the initial algorithm found on the internet:
````
while simulating do
    CollectCollisionPairs();
    h ← Δt/numSubsteps;
    for numSubsteps do
        for n bodies and particles do
            x_prev ← x;
            v ← v + h*f_ext/m;
            x ← x + h*v;
        end
        for numPosIters do
            SolvePositions(x₁,...,xₙ);
        end
        for n bodies and particles do
            v ← (x - x_prev)/h;
        end
        SolveVelocities(v₁,...,vₙ);
    end
end
````

In the papers, the CollectCollisionPairs is applies once, and seems to use some Continuous Collision Detection.

````
while simulating do
    h ← Δt/numSubsteps;

    for numSubsteps do
        for n bodies and particles do
            x_prev ← x;
            v ← v + h*f_ext/m;
            x ← x + h*v;
        end
        
        cp ← BroadPhaseCollectCollisionPairs();
        contacts ← CollectCollisionPairs(cp);

        SolvePositions(contacts);
        
        for n bodies and particles do
            v ← (x - x_prev)/h;
        end
        
        SolveVelocities(v₁,...,vₙ);
    end
end
````

As explained in https://matthias-research.github.io/pages/publications/PBDBodies.pdf:
"Finally, the concern regarding slow convergence was addressed
most recently in [MSL∗19]. By replacing solver iterations with
substeps, Gauss-Seidel and Jacobi methods become competitors of
global solvers in terms of convergence. Substepping in combination with one NPGS iteration per substep yields a method that looks
computationally almost identical to an explicit integration step, but
with the advantage of being unconditionally stable due to the usage
of compliance. We call it a quasi-explicit method.".

- It means we can remove the iterations of the solver, using substepping.
- We also apply the velocity/forces first, per body, and then look at if any constraint/collision.

Note: XPBD computes the position, with an implicit velocity. But this paradigm stops for the friction and the restitution forces.
The last computation in the substeps is SolveVelocities, a dedicated and required step to compute any optional force.
This given velocity is computed by the constraints, and then used in the next world.Step.

### Constraints
- ContactConstraint: temporary constraint, generated when a collision is detected between two rigid bodies.

A not exhaustive list of possible constraints (not implemented yet):
- Friction: Opposes tangential motion at contact points. Usage: Realistic sliding, grip, objects staying on slopes
- Manifold (multi point contact): multiple contact points. Usage: Stacking stable, boxes
- Distance: Maintains constant distance between two points. Usage: Ropes, chains, rigid connections, ragdoll bones
- Distance Range: Keeps distance within [min, max] range. Usage: Elastic ropes, springs with limits, telescopic joints
- Hinge: Allows rotation around one axis only (like a door). Usage: Doors, wheels, joints, rotating platforms
- Angular Range: limits rotation within [min/max]. Usage: articulation

## GJK

## EPA

## Sources
- https://matthias-research.github.io/pages/publications/PBDBodies.pdf
- https://matthias-research.github.io/pages/publications/smallsteps.pdf
- https://matthias-research.github.io/pages/tenMinutePhysics/09-xpbd.pdf
- https://matthias-research.github.io/pages/tenMinutePhysics/22-rigidBodies.pdf
- https://blog.mmacklin.com/2016/10/12/xpbd-slides-and-stiffness/
- https://johanhelsing.studio/posts/bevy-xpbd/
- https://cse442-17f.github.io/Gilbert-Johnson-Keerthi-Distance-Algorithm/
- https://medium.com/@mbayburt/walkthrough-of-the-gjk-collision-detection-algorithm-80823ef5c774 (this method seems valid only for 2D)
- https://winter.dev/articles/epa-algorithm

## Contributing Guidelines

See [how to contribute](CONTRIBUTING.md).

## Licence
This project is distributed under the [Apache 2.0 licence](LICENCE.md).
