use bevy::{prelude::*, sprite::MaterialMesh2dBundle};

pub struct BoidsPlugin;

impl Plugin for BoidsPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Settings::new(10))
            .add_systems(Startup, (spawn_camera, spawn_boids))
            .add_systems(Update, project_positions);
    }
}

#[derive(Resource)]
pub struct Settings {
    nb_boids: u32,
}

impl Settings {
    fn new(nb_boids: u32) -> Self {
        Self { nb_boids }
    }
}

#[derive(Component)]
struct Boid;

#[derive(Component)]
struct Position(Vec3);

#[derive(Bundle)]
struct BoidBundle {
    boid: Boid,
    position: Position,
}

impl BoidBundle {
    fn new(pos: Vec2) -> Self {
        Self {
            boid: Boid,
            position: Position(Vec3::new(pos.x, pos.y, 0.)),
        }
    }
}

const BOID_SIZE: f32 = 10.;

fn spawn_camera(mut commands: Commands) {
    commands.spawn_empty().insert(Camera2dBundle::default());
}

fn project_positions(mut positionables: Query<(&mut Transform, &Position)>) {
    for (mut transform, position) in &mut positionables {
        transform.translation = position.0;
    }
}

pub fn spawn_boids(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    settings: Res<Settings>,
) {
    let mesh = Mesh::from(shape::RegularPolygon::new(BOID_SIZE, 3));
    let material = ColorMaterial::from(Color::rgb(0., 0., 1.));

    let mesh_handle = meshes.add(mesh);
    let material_handle = materials.add(material);

    for i in 0..settings.nb_boids as u32 {
        let pos = Vec2::new(i as f32 * 30., 0.);
        let bundle = BoidBundle::new(pos);

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
