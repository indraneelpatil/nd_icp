use crate::types::{Point, Point3D, PointSet};
use nalgebra::{Dyn, OMatrix};
use ply_rs::parser::Parser;
use std::fs::File;

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

pub fn visualise_points(
    model_cloud: &PointSet<Point3D>,
    target_cloud: &PointSet<Point3D>,
    transformation: OMatrix<f64, Dyn, Dyn>,
) {
}
