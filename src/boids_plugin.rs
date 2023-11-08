use bevy::{prelude::*, sprite::MaterialMesh2dBundle};
use bevy_inspector_egui::prelude::*;
use bevy_inspector_egui::quick::ResourceInspectorPlugin;
use rand::Rng;

#[derive(Debug, Clone, Copy, Default, Eq, PartialEq, Hash, States)]
enum AppState {
    #[default]
    Running,
    Paused,
}

pub struct BoidsPlugin;

impl Plugin for BoidsPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<Settings>()
            .insert_resource(Settings {
                ..Default::default()
            })
            .add_state::<AppState>()
            .add_systems(Startup, (spawn_camera, spawn_boids))
            .add_systems(Update, check_keyboard_input)
            .add_systems(
                Update,
                (move_boids, project_positions.after(move_boids))
                    .run_if(in_state(AppState::Running)),
            )
            .add_plugins(ResourceInspectorPlugin::<Settings>::new());
    }
}

/// Simulation settings; everything can be updated through UI except the number of boids
#[derive(Resource, Reflect, InspectorOptions)]
#[reflect(Resource, InspectorOptions)]
struct Settings {
    /// Number of boids
    #[inspector(min = 400, max = 400)]
    nb_boids: u32,
    /// Boids ability to turn fast
    #[inspector(min = 0., max = 2., speed = 0.01)]
    turn_factor: f32,
    /// Radius (in px) of the circle in which boids can see
    #[inspector(min = 0, max = 100, speed = 1.)]
    visual_range: u32,
    /// Radius (in px) of the circle in which boids wants to be alone
    #[inspector(min = 0, max = 20, speed = 1.)]
    protected_range: u32,
    /// Cohesion rule : boids move toward the center of mass of their neighbors
    #[inspector(min = 0., max = 0.001, speed = 0.0001)]
    centering_factor: f32,
    /// Separation rule: boids move away from other boids that are in protected range
    #[inspector(min = 0., max = 0.2, speed = 0.01)]
    avoid_factor: f32,
    /// Alignment rule: boids try to match the average velocity of boids located in its visual range
    #[inspector(min = 0., max = 0.3, speed = 0.001)]
    matching_factor: f32,
    /// Max boids speed
    #[inspector(min = 3., max = 10., speed = 1.)]
    max_speed: f32,
    /// Min boids speed
    #[inspector(min = 1., max = 10., speed = 1.)]
    min_speed: f32,
    /// Some boids are searching for food, and are not exactly following the flock
    #[inspector(min = 0., max = 0.1, speed = 0.001)]
    bias: f32,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            nb_boids: 400,
            turn_factor: 0.2,
            visual_range: 40,
            protected_range: 8,
            centering_factor: 0.0005,
            avoid_factor: 0.05,
            matching_factor: 0.05,
            max_speed: 6.,
            min_speed: 3.,
            bias: 0.001,
        }
    }
}

/// Different kind of boids
#[derive(Component, Debug, Clone)]
enum BoidRole {
    /// No specific role, the boid just follows the flock
    Common,
    /// Scouts try to find food and don't exactly follow the flock
    ///
    /// group 1 tends to search on the right
    ///
    /// group 2 tends to search on the left
    Scout(u8),
}

#[derive(Component, Debug)]
struct Boid;

#[derive(Component, Debug, Clone)]
struct Position(Vec3);

#[derive(Component, Debug, Clone)]
struct Velocity(Vec3);

#[derive(Bundle)]
struct BoidBundle {
    boid: Boid,
    position: Position,
    velocity: Velocity,
    role: BoidRole,
}

impl BoidBundle {
    fn new(pos: Vec2, role: BoidRole) -> Self {
        Self {
            boid: Boid,
            position: Position(Vec3::new(pos.x, pos.y, 0.)),
            velocity: Velocity(Vec3::new(0., 0., 0.)),
            role,
        }
    }
}

fn spawn_camera(mut commands: Commands) {
    commands.spawn_empty().insert(Camera2dBundle::default());
}

fn project_positions(mut positionables: Query<(&mut Transform, &Position)>) {
    for (mut transform, position) in &mut positionables {
        transform.translation = position.0;
    }
}

fn spawn_boids(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    settings: Res<Settings>,
) {
    let mesh = Mesh::from(shape::Circle::new(2.));
    let material = ColorMaterial::from(Color::rgb(1., 1., 1.));

    let mesh_handle = meshes.add(mesh);
    let material_handle = materials.add(material);

    for _ in 0..settings.nb_boids as u32 {
        let pos = Vec2::new(
            rand::thread_rng().gen_range(-500..=300) as f32,
            rand::thread_rng().gen_range(-500..=300) as f32,
        );

        let role = match rand::thread_rng().gen_range(0..=100) {
            x if x >= 95 => BoidRole::Scout(2),
            x if x >= 90 => BoidRole::Scout(1),
            x if x >= 0 => BoidRole::Common,
            _ => BoidRole::Common,
        };

        let bundle = BoidBundle::new(pos, role);

        commands.spawn((
            bundle,
            MaterialMesh2dBundle {
                mesh: mesh_handle.clone().into(), // Handle<Mesh> into a Mesh2dHandle
                material: material_handle.clone(),
                ..default()
            },
        ));
    }
}

/// Describes what a boid sees within visual range
struct BoidEstimate {
    /// average x position of neighboring boids
    xpos_avg: f32,
    /// average y position of neighboring boids
    ypos_avg: f32,
    /// average vx velocity of neighboring boids
    xvel_avg: f32,
    /// average vy velocity of neighboring boids
    yvel_avg: f32,
    /// number of boids withing visual range
    neighboring_boids: u32,
    /// closest boid x coord
    close_dx: f32,
    /// closest boid y coord
    close_dy: f32,
}

impl BoidEstimate {
    fn new() -> BoidEstimate {
        Self {
            xpos_avg: 0.,
            ypos_avg: 0.,
            xvel_avg: 0.,
            yvel_avg: 0.,
            neighboring_boids: 0,
            close_dx: 0.,
            close_dy: 0.,
        }
    }
}

fn evaluate_situation(
    current: &(Mut<Position>, Mut<Velocity>, &BoidRole),
    other: &(Position, Velocity, BoidRole),
    estimate: &mut BoidEstimate,
    settings: &Res<Settings>,
) {
    let (pos, _, _) = current;
    let visual_range = settings.visual_range as f32;
    let protected_range = settings.protected_range as f32;

    let (other_pos, other_v, _) = other;

    let dx = pos.0.x - other_pos.0.x;
    let dy = pos.0.y - other_pos.0.y;

    if dx.abs() < visual_range && dy.abs() < visual_range {
        let squared_distance = dx * dx + dy * dy;

        if squared_distance < protected_range * protected_range {
            estimate.close_dx += pos.0.x - other_pos.0.x;
            estimate.close_dy += pos.0.y - other_pos.0.y;
        } else if squared_distance < visual_range * visual_range {
            estimate.xpos_avg += other_pos.0.x;
            estimate.ypos_avg += other_pos.0.y;
            estimate.xvel_avg += other_v.0.x;
            estimate.yvel_avg += other_v.0.y;

            estimate.neighboring_boids += 1
        }
    }
}

fn set_average_speed_and_pos(estimate: &mut BoidEstimate) {
    let neighboring_boids = estimate.neighboring_boids as f32;

    estimate.xpos_avg /= neighboring_boids;
    estimate.ypos_avg /= neighboring_boids;
    estimate.xvel_avg /= neighboring_boids;
    estimate.yvel_avg /= neighboring_boids;
}

fn apply_cohesion(
    current: &mut (Mut<Position>, Mut<Velocity>, &BoidRole),
    estimate: &mut BoidEstimate,
    settings: &Res<Settings>,
) {
    let (pos, v, _) = current;
    let centering_factor = settings.centering_factor;
    let matching_factor = settings.matching_factor;

    v.0.x += (estimate.xpos_avg - pos.0.x) * centering_factor
        + (estimate.xvel_avg - v.0.x) * matching_factor;

    v.0.y += (estimate.ypos_avg - pos.0.y) * centering_factor
        + (estimate.yvel_avg - v.0.y) * matching_factor;
}

fn apply_alignment(
    current: &mut (Mut<Position>, Mut<Velocity>, &BoidRole),
    estimate: &mut BoidEstimate,
    settings: &Res<Settings>,
) {
    let (_, v, _) = current;
    let matching_factor = settings.matching_factor;

    v.0.x += (estimate.xvel_avg - v.0.x) * matching_factor;

    v.0.y += (estimate.yvel_avg - v.0.y) * matching_factor;
}

fn apply_avoidance(
    current: &mut (Mut<Position>, Mut<Velocity>, &BoidRole),
    estimate: &BoidEstimate,
    settings: &Res<Settings>,
) {
    let (_, v, _) = current;
    let avoid_factor = settings.avoid_factor;

    v.0.x += estimate.close_dx * avoid_factor;
    v.0.y += estimate.close_dy * avoid_factor;
}

fn turn_if_edge(
    current: &mut (Mut<Position>, Mut<Velocity>, &BoidRole),
    screen_dimensions: (f32, f32),
    settings: &Res<Settings>,
) {
    let (pos, v, _) = current;
    let (x, y) = (pos.0.x, pos.0.y);
    let (width, height) = screen_dimensions;
    let turn_factor = settings.turn_factor;

    if x <= -width / 2. + 200. {
        v.0.x += turn_factor;
    } else if x >= width / 2. - 200. {
        v.0.x -= turn_factor;
    }

    if y <= -height / 2. + 200. {
        v.0.y += turn_factor;
    } else if y >= height / 2. - 200. {
        v.0.y -= turn_factor;
    }
}

fn apply_bias(current: &mut (Mut<Position>, Mut<Velocity>, &BoidRole), settings: &Res<Settings>) {
    let (_, v, role) = current;
    let bias = settings.bias;

    match **role {
        BoidRole::Scout(1) => v.0.x = (1. - bias) * v.0.x + bias,
        BoidRole::Scout(2) => v.0.x = (1. - bias) * v.0.x - bias,
        BoidRole::Common => (),
        _ => (),
    };
}
fn compute_new_speed(
    current: &mut (Mut<Position>, Mut<Velocity>, &BoidRole),
    settings: &Res<Settings>,
) {
    let (_, v, _) = current;
    let min_speed = settings.min_speed;
    let max_speed = settings.max_speed;

    let speed = f32::sqrt(v.0.x * v.0.x + v.0.y * v.0.y);

    if speed < min_speed {
        v.0.x = (v.0.x / speed) * min_speed;
        v.0.y = (v.0.y / speed) * min_speed;
    }
    if speed > max_speed {
        v.0.x = (v.0.x / speed) * max_speed;
        v.0.y = (v.0.y / speed) * max_speed;
    }
}

fn compute_new_position(
    current: &mut (Mut<Position>, Mut<Velocity>, &BoidRole),
    screen_dimensions: (f32, f32),
) {
    let (pos, v, _) = current;
    pos.0.x += v.0.x;
    pos.0.y += v.0.y;

    let (width, height) = (screen_dimensions.0 / 2.0, screen_dimensions.1 / 2.0);

    if pos.0.x > width {
        pos.0.x = width;
    } else if pos.0.x <= -width {
        pos.0.x = -width;
    }

    if pos.0.y > height {
        pos.0.y = height;
    } else if pos.0.y <= -height {
        pos.0.y = -height;
    }
}

fn move_boids(
    mut boids: Query<(&mut Position, &mut Velocity, &BoidRole), With<Boid>>,
    window: Query<&Window>,
    settings: Res<Settings>,
) {
    if let Ok(window) = window.get_single() {
        let (width, height) = (window.resolution.width(), window.resolution.height());

        /*
            Here the only solution is to clone query results, here's why :
            - boids.iter_combinations() would return 1:1 comparisons, while boids algo needs 1:others for each boid
            - using RefCell would imply to use RefCell<&Position, &Velocity, &BoidRole> : since
            RefCell wouldn't hold an owned value here, this doesn't solve the issue
            - pushing updated values in a Vec, and updating boids after the nested "for" loops would result
            in boids considering static flock during iteration, and not previously updated neighbors (moving).
            This would create less accurate simulation and more "solid shape" appearance.

        */
        let tmp = boids
            .iter()
            .map(|x| (x.0.clone(), x.1.clone(), x.2.clone()))
            .collect::<Vec<(Position, Velocity, BoidRole)>>();

        for mut boid in boids.iter_mut() {
            let mut estimate = BoidEstimate::new();
            for other in tmp.iter() {
                evaluate_situation(&boid, other, &mut estimate, &settings);
            }

            if estimate.neighboring_boids > 0 {
                set_average_speed_and_pos(&mut estimate);

                apply_cohesion(&mut boid, &mut estimate, &settings);
                apply_alignment(&mut boid, &mut estimate, &settings);
                apply_avoidance(&mut boid, &estimate, &settings);
            }

            turn_if_edge(&mut boid, (width as f32, height as f32), &settings);
            apply_bias(&mut boid, &settings);
            compute_new_speed(&mut boid, &settings);
            compute_new_position(&mut boid, (width as f32, height as f32));
        }
    }
}

fn check_keyboard_input(
    keyboard_input: Res<Input<KeyCode>>,
    app_state: Res<State<AppState>>,
    mut next_state: ResMut<NextState<AppState>>,
    mut app_exit_events: ResMut<Events<bevy::app::AppExit>>,
) {
    if keyboard_input.just_pressed(KeyCode::P) {
        match app_state.get() {
            AppState::Paused => next_state.set(AppState::Running),
            _ => next_state.set(AppState::Paused),
        };
    } else if keyboard_input.just_pressed(KeyCode::Q) {
        app_exit_events.send(bevy::app::AppExit);
    }
}
