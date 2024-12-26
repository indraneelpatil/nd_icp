use rust_icp::types::Point;
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
    let dist_delta = 1e-2;
    let icp = Icp::new(model_point_set.clone(), max_iterations, dist_delta);

    // Run ICP
    let result = icp.register(&mut target_point_set);
    println!("{}", result);

    // Visualise points
    visualise_points(&model_point_set, &target_point_set);
}
