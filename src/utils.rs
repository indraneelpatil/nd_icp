use crate::types::{Point3D, PointSet};
use nalgebra::{Dyn, Matrix3, OMatrix, Quaternion, Rotation3, UnitQuaternion};
use ply_rs::parser::Parser;
use std::fs::File;
use three_d::FrameOutput;
use three_d::{
    degrees, vec3, Axes, Camera, ClearState, ColorMaterial, CpuMesh, Gm, InstancedMesh, Mat4,
    OrbitControl, PointCloud, Srgba, Vec3, Window, WindowSettings,
};

pub fn load_ply_as_point_set(
    file_path: &str,
) -> Result<PointSet<Point3D>, Box<dyn std::error::Error>> {
    // Open the PLY file
    let file = File::open(file_path)?;
    let mut f = std::io::BufReader::new(file);

    // Initialize the PLY parser
    let point_3d_parser = Parser::<Point3D>::new();

    let header = point_3d_parser.read_header(&mut f).unwrap();

    let mut points = vec![];
    for (_, element) in &header.elements {
        // we could also just parse them in sequence, but the file format might change
        match element.name.as_ref() {
            "vertex" => {
                points = point_3d_parser
                    .read_payload_for_element(&mut f, &element, &header)
                    .unwrap();
            }
            _ => {}
        }
    }
    // println!("point list: {:#?}", points);
    Ok(PointSet { points })
}

pub fn visualise_points(model_cloud: &PointSet<Point3D>, target_cloud: &PointSet<Point3D>) {
    // Create a window to render the scene
    let window = Window::new(WindowSettings {
        title: "3D Point Cloud Visualization".to_string(),
        max_size: Some((1280, 720)),
        ..Default::default()
    })
    .unwrap();

    let context = window.gl();

    // Create a camera to view the scene
    let mut camera = Camera::new_perspective(
        window.viewport(),
        vec3(0.125, -0.25, -0.5),
        vec3(0.0, 0.0, 0.0),
        vec3(0.0, 1.0, 0.0),
        degrees(45.0),
        0.01,
        100.0,
    );

    let mut control = OrbitControl::new(*camera.target(), 0.1, 3.0);

    // Convert the PointSets into a format that can be rendered (using CpuMesh)
    let model_points: Vec<Vec3> = model_cloud
        .points
        .iter()
        .map(|p| vec3(p.x as f32, p.y as f32, p.z as f32))
        .collect();
    let target_points: Vec<Vec3> = target_cloud
        .points
        .iter()
        .map(|p| vec3(p.x as f32, p.y as f32, p.z as f32))
        .collect();

    // Create point clouds
    let model_cloud = PointCloud {
        positions: three_d::Positions::F32(model_points.clone()),
        ..Default::default()
    };

    let target_cloud = PointCloud {
        positions: three_d::Positions::F32(target_points.clone()),
        ..Default::default()
    };

    let mut point_mesh = CpuMesh::sphere(4);
    point_mesh.transform(&Mat4::from_scale(0.0001)).unwrap();

    let model_cloud_gm = Gm {
        geometry: InstancedMesh::new(&context, &model_cloud.into(), &point_mesh),
        material: ColorMaterial {
            color: Srgba::new(0, 0, 255, 255), // Blue
            ..Default::default()
        },
    };
    // let c = -model_cloud_gm.aabb().center();
    // model_cloud_gm.set_transformation(Mat4::from_translation(c));

    let target_cloud_gm = Gm {
        geometry: InstancedMesh::new(&context, &target_cloud.into(), &point_mesh),
        material: ColorMaterial {
            color: Srgba::new(255, 0, 0, 255), // Red
            ..Default::default()
        },
    };

    // main loop
    window.render_loop(move |mut frame_input| {
        let mut redraw = frame_input.first_frame;
        redraw |= camera.set_viewport(frame_input.viewport);
        redraw |= control.handle_events(&mut camera, &mut frame_input.events);

        if redraw {
            frame_input
                .screen()
                .clear(ClearState::color_and_depth(1.0, 1.0, 1.0, 1.0, 1.0))
                .render(
                    &camera,
                    model_cloud_gm
                        .into_iter()
                        .chain(&Axes::new(&context, 0.01, 0.1))
                        .chain(target_cloud_gm.into_iter()),
                    &[],
                );
        }

        FrameOutput {
            swap_buffers: redraw,
            ..Default::default()
        }
    });
}

pub fn get_quaternion_from_dynamic_rotation(rotation: &OMatrix<f32, Dyn, Dyn>) -> Quaternion<f32> {
    assert!(
        rotation.nrows() == 3 && rotation.ncols() == 3,
        "Matrix must be 3x3"
    );

    let static_rotation = Matrix3::from_iterator(rotation.iter().cloned());
    let rot = Rotation3::from_matrix(&static_rotation);

    *UnitQuaternion::from_rotation_matrix(&rot)
}
