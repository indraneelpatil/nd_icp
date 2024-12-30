use nalgebra::{Rotation, Rotation3};
use rust_icp::types::Point;
use rust_icp::utils::get_quaternion_from_dynamic_rotation;
use rust_icp::{
    icp::Icp,
    utils::{load_ply_as_point_set, visualise_points},
};

fn main() {
    // Model point cloud
    let model_cloud_path = "data/bun000.ply";
    let model_point_set = match load_ply_as_point_set(model_cloud_path) {
        Ok(point_set) => {
            println!(
                "Successfully loaded model point set of {:?} points",
                point_set.points.len()
            );
            point_set
        }
        Err(err) => {
            panic!("Could not load model point set {err:?}");
        }
    };

    // Target point cloud
    let target_cloud_path = "data/bun045.ply";
    let mut target_point_set = match load_ply_as_point_set(target_cloud_path) {
        Ok(point_set) => {
            println!(
                "Successfully loaded target point set of {:?} points",
                point_set.points.len()
            );
            point_set
        }
        Err(err) => {
            panic!("Could not load target point set {err:?}");
        }
    };

    // Initialise ICP
    let max_iterations = 3;
    let cost_change_threshold = 1e-3;
    let icp = Icp::new(
        model_point_set.clone(),
        max_iterations,
        cost_change_threshold,
    );

    // Run ICP
    let result = icp.register(&target_point_set);

    // Print in readable form
    let dimension = target_point_set
        .points
        .first()
        .expect("No points in cloud")
        .get_dimensions();
    let translation = result.view((0, dimension), (dimension, 1));
    let rotation = result.view((0, 0), (dimension, dimension)).clone_owned();
    let quaternion = get_quaternion_from_dynamic_rotation(&rotation);
    println!("Translation {} {}", translation, quaternion);

    // Visualise points
    visualise_points(&model_point_set, &target_point_set);
}
