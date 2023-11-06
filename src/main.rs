use bevy::{prelude::*, window::WindowMode};

mod boids_plugin;

fn main() {
    let window_plugin = WindowPlugin {
        primary_window: Some(Window {
            mode: WindowMode::Fullscreen,
            ..default()
        }),
        ..default()
    };

    App::new()
        .add_plugins((DefaultPlugins.set(window_plugin), boids_plugin::BoidsPlugin))
        .run();
}
