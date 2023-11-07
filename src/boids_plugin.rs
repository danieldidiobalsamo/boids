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

#[derive(Resource, Reflect, InspectorOptions)]
#[reflect(Resource, InspectorOptions)]
struct Settings {
    #[inspector(min = 100, max = 100)]
    nb_boids: u32,
    #[inspector(min = 0., max = 1., speed = 0.01)]
    turn_factor: f32,
    #[inspector(min = 0, max = 100, speed = 1.)]
    visual_range: u32,
    #[inspector(min = 0, max = 20, speed = 1.)]
    protected_range: u32,
    #[inspector(min = 0., max = 0.0005, speed = 0.0001)]
    centering_factor: f32,
    #[inspector(min = 0., max = 1., speed = 0.01)]
    avoid_factor: f32,
    #[inspector(min = 0., max = 0.7, speed = 0.01)]
    matching_factor: f32,
    #[inspector(min = 3., max = 10., speed = 1.)]
    max_speed: f32,
    #[inspector(min = 1., max = 10., speed = 1.)]
    min_speed: f32,
    #[inspector(min = 0., max = 0.01, speed = 0.001)]
    bias: f32,
}

impl Settings {
    fn new(
        nb_boids: u32,
        turn_factor: f32,
        visual_range: u32,
        protected_range: u32,
        centering_factor: f32,
        avoid_factor: f32,
        matching_factor: f32,
        max_speed: f32,
        min_speed: f32,
        bias: f32,
    ) -> Self {
        Self {
            nb_boids,
            turn_factor,
            visual_range,
            protected_range,
            centering_factor,
            avoid_factor,
            matching_factor,
            max_speed,
            min_speed,
            bias,
        }
    }
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            nb_boids: 100,
            turn_factor: 0.2,
            visual_range: 40,
            protected_range: 10,
            centering_factor: 0.0002,
            avoid_factor: 0.05,
            matching_factor: 0.5,
            max_speed: 10.,
            min_speed: 2.,
            bias: 0.005,
        }
    }
}

#[derive(Component, Debug)]
enum BoidRole {
    Common,
    Scout(u8),
}

#[derive(Component, Debug)]
struct Boid;

#[derive(Component, Debug)]
struct Position(Vec3);

#[derive(Component, Debug)]
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
    let mesh = Mesh::from(shape::Circle::new(4.));
    let material = ColorMaterial::from(Color::rgb(1., 1., 1.));

    let mesh_handle = meshes.add(mesh);
    let material_handle = materials.add(material);

    for _ in 0..settings.nb_boids as u32 {
        let pos = Vec2::new(
            rand::thread_rng().gen_range(100..=300) as f32,
            rand::thread_rng().gen_range(100..=300) as f32,
        );

        let role = match rand::thread_rng().gen_range(0..=100) {
            x if x >= 66 => BoidRole::Scout(2),
            x if x >= 33 => BoidRole::Scout(1),
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

struct BoidEstimate {
    xpos_avg: f32,
    ypos_avg: f32,
    xvel_avg: f32,
    yvel_avg: f32,
    neighboring_boids: u32,
    close_dx: f32,
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
    current: &(&Position, &Velocity, &BoidRole),
    other: &(&Position, &Velocity, &BoidRole),
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

fn apply_cohesion_and_alignment(
    current: &mut (Mut<Position>, Mut<Velocity>, &BoidRole),
    estimate: &mut BoidEstimate,
    settings: &Res<Settings>,
) {
    let (pos, v, _) = current;
    let centering_factor = settings.centering_factor;
    let matching_factor = settings.matching_factor;

    // if some boids are in the visual range
    if estimate.neighboring_boids > 0 {
        let neighboring_boids = estimate.neighboring_boids as f32;

        // divide average pos / velocity by number of boids in visual range
        estimate.xpos_avg /= neighboring_boids;
        estimate.ypos_avg /= neighboring_boids;
        estimate.xvel_avg /= neighboring_boids;
        estimate.yvel_avg /= neighboring_boids;

        v.0.x += (estimate.xpos_avg - pos.0.x) * centering_factor
            + (estimate.xvel_avg - v.0.x) * matching_factor;

        v.0.y += (estimate.ypos_avg - pos.0.y) * centering_factor
            + (estimate.yvel_avg - v.0.y) * matching_factor;
    }
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

    if x <= -width / 2. {
        v.0.x += turn_factor;
    } else if x >= width / 2. {
        v.0.x -= turn_factor;
    }

    if y <= -height / 2. {
        v.0.y += turn_factor;
    } else if y >= height / 2. {
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

        let mut combinations = boids.iter_combinations();

        let mut estimate = BoidEstimate::new();
        while let Some([current, other]) = combinations.fetch_next() {
            evaluate_situation(&current, &other, &mut estimate, &settings);
            evaluate_situation(&other, &current, &mut estimate, &settings);
        }

        for mut boid in &mut boids {
            apply_cohesion_and_alignment(&mut boid, &mut estimate, &settings);
            apply_avoidance(&mut boid, &estimate, &settings);
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
