use rust_icp::utils::load_ply_as_point_cloud;

fn main() {
    // Model point cloud
    let model_cloud_path = "data/bun000.ply";
    match load_ply_as_point_cloud(model_cloud_path) {
        Ok(point_set) => {
            println!("Successfully loaded point set {:?}", point_set);
        }
        Err(err) => {
            println!("error!");
        }
    }

    // Target point cloud
    let model_cloud_path = "data/bun045.ply";
}
