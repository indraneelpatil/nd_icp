use nalgebra::{Dyn, OMatrix, Vector3, Vector4, VectorN, U1, U4};
// Created by Indraneel on 7th Dec
use nalgebra::{SVector, U3};
use ply_rs::ply;

#[derive(Clone, Debug)]
pub struct PointSet<T>
where
    T: Point,
{
    pub points: Vec<T>,
}

pub trait Point {
    fn get_dimensions(&self) -> usize;
    fn apply_transformation(&mut self, transformation: &OMatrix<f32, Dyn, Dyn>);
}

#[derive(Debug, Clone)]
pub struct Point3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Point for Point3D {
    fn get_dimensions(&self) -> usize {
        3
    }

    fn apply_transformation(&mut self, transformation: &OMatrix<f32, Dyn, Dyn>) {
        // Express point in homogenous coordinates
        let homogenous_coordinates: OMatrix<f32, U1, U4> =
            OMatrix::from([self.x, self.y, self.z, 1.0]);

        let transformed_coordinates = homogenous_coordinates * transformation;

        // Update point
        self.x = *transformed_coordinates.get(0).expect("Missing x");
        self.y = *transformed_coordinates.get(1).expect("Missing y");
        self.z = *transformed_coordinates.get(2).expect("Missing z");
    }
}

impl ply::PropertyAccess for Point3D {
    fn new() -> Self {
        Point3D {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }
    fn set_property(&mut self, key: String, property: ply::Property) {
        match (key.as_ref(), property) {
            ("x", ply::Property::Float(v)) => self.x = v,
            ("y", ply::Property::Float(v)) => self.y = v,
            ("z", ply::Property::Float(v)) => self.z = v,
            (k, _) => panic!("Vertex: Unexpected key/value combination: key: {}", k),
        }
    }
}
