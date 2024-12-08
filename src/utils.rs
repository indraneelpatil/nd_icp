use crate::types::{Point3D, PointSet};
use ply_rs::{parser::Parser, ply::DefaultElement};
use std::{fs::File, io::BufReader};

pub fn load_ply_as_point_cloud(file_path: &str) -> Result<(), Box<dyn std::error::Error>> {
    // Open the PLY file
    let mut file = File::open(file_path)?;

    // Initialize the PLY parser
    let parser = Parser::<DefaultElement>::new();
    let ply = parser.read_ply(&mut file)?;

    // proof that data has been read
    println!("Ply header: {:#?}", ply.header);
    println!("Ply data: {:?}", ply.payload);

    // let header = parser.read_header(&mut reader)?;

    // // Ensure the file has a "vertex" element with x, y, z properties
    // let vertex_def = header
    //     .elements
    //     .get("vertex")
    //     .ok_or("PLY file does not contain a 'vertex' element")?;

    // if !(vertex_def. .contains("x") && vertex_def.contains("y") && vertex_def.contains("z")) {
    //     return Err("Vertex element does not contain x, y, z properties".into());
    // }
    // // Convert to nalgebra points
    // let points: Vec<Vector3<f32>> = vertices
    //     .into_iter()
    //     .map(|vertex| {
    //         let x = vertex["x"].as_float().unwrap_or(0.0) as f32;
    //         let y = vertex["y"].as_float().unwrap_or(0.0) as f32;
    //         let z = vertex["z"].as_float().unwrap_or(0.0) as f32;
    //         Vector3::new(x, y, z)
    //     })
    //     .collect();

    Ok(())
}
