https://github.com/danieldidiobalsamo/boids/assets/79797812/dd61766b-6962-4ac8-957f-da5a2f3b936d

# About

Rust boids simulation using Reynolds model running with Bevy engine.

The following parameters can be updated while the simulation is running:
- turn factor: boids ability to turn fast
- visual range: radius (in px) of the circle in which boids can see
- protected range: radius (in px) of the circle in which boids wants to be alone
- centering factor (cohesion rule) : boids move toward the center of mass of their neighbors
- avoid factor (separation rule): boids move away from other boids that are in protected range
- matching factor (alignment rule): boids try to match the average velocity of boids located in its visual range
- Maximum boids velocity
- Minimum boids velocity
- bias: some boids are searching for food, and are not exactly following the flock

While all the parameters can be updated live, you can also press 'P' to pause/resume the simulation.
Press 'Q' to leave.

# How to launch
## Using cargo (recommended)

Install [bevy dependencies](https://github.com/bevyengine/bevy/blob/main/docs/linux_dependencies.md) and then just launch:
~~~
cargo install boids_rs_bevy
boids_rs_bevy
~~~

## Build manually

Firstly, install [bevy dependencies](https://github.com/bevyengine/bevy/blob/main/docs/linux_dependencies.md)

Then clone this repository and launch :
~~~
cargo run --release
~~~

Note: if you want to launch as dev, make sure to add the following feature to decrease compilation time :
~~~
cargo run --features bevy/dynamic_linking 
~~~ 
