use rust_icp::utils::load_ply_as_point_set;

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
    let target_point_set = match load_ply_as_point_set(target_cloud_path) {
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
}
