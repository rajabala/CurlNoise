# CurlNoise
A C++ implementation of curl noise for adding turbulence to particles. Also contains a sample Unity project that uses it to affect particle velocities.


The underlying potential (force) field uses Perlin3D noise and is modulated in the presence of obstacles to make the flow tangential at the boundary.
This is done via distance field estimation for obstacle primitives such as Sphere, Box and Cylinder.

###Note:
This is a work in progress.

Obstacle detection and avoidance hasn't been validated yet.

The Unity usage is only a demo. Updating particles on a single thread every frame is not practical.

## References:
For a great write up of using curl for turbulence, check out:
http://www.cs.ubc.ca/~rbridson/docs/bridson-siggraph2007-curlnoise.pdf

For a fantastic demonstration of the above concept, see:
http://prideout.net/blog/?p=63

To learn about noise generation and derivatives for calculating curl, see:
http://catlikecoding.com/unity/tutorials/noise-derivatives/

Distance field functions for common and complex primitives: http://iquilezles.org/www/articles/distfunctions/distfunctions.htm
