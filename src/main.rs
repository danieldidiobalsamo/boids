use bevy::prelude::*;

mod boids_plugin;

fn main() {
    App::new()
        .add_plugins((DefaultPlugins, boids_plugin::BoidsPlugin))
        .run();
}
