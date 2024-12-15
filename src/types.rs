use nalgebra::{Dyn, OMatrix, Vector3, Vector4, VectorN};
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
    fn apply_transformation(&mut self, transformation: &OMatrix<f64, Dyn, Dyn>);
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

    fn apply_transformation(&mut self, transformation: &OMatrix<f64, Dyn, Dyn>) {}
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
