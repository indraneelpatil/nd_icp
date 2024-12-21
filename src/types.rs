use std::iter::Sum;
use std::ops::{Div, Sub};

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
    fn find_distance(&self, other_point: &Self) -> f32;
    fn find_distance_squared(&self, other_point: &Self) -> f32;
    fn to_vec(&self) -> Vec<f32>;
}

#[derive(Debug, Clone, Copy)]
pub struct Point3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Div<f32> for Point3D {
    type Output = Point3D;

    fn div(self, rhs: f32) -> Self::Output {
        assert!(rhs != 0.0, "Division by zero is not allowed");
        Point3D {
            x: self.x / rhs,
            y: self.y / rhs,
            z: self.z / rhs,
        }
    }
}

impl Sub<Self> for Point3D {
    type Output = Point3D;

    fn sub(self, rhs: Self) -> Self::Output {
        Point3D {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl Sum for Point3D {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(
            Point3D {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            |acc, point| Point3D {
                x: acc.x + point.x,
                y: acc.y + point.y,
                z: acc.z + point.z,
            },
        )
    }
}

impl Point for Point3D {
    fn get_dimensions(&self) -> usize {
        3
    }

    fn apply_transformation(&mut self, transformation: &OMatrix<f32, Dyn, Dyn>) {
        // Express point in homogenous coordinates
        let homogenous_coordinates: OMatrix<f32, U4, U1> =
            OMatrix::from([self.x, self.y, self.z, 1.0]);

        let transformed_coordinates = transformation * homogenous_coordinates;

        // Update point
        self.x = transformed_coordinates[(0, 0)];
        self.y = transformed_coordinates[(1, 0)];
        self.z = transformed_coordinates[(2, 0)];
    }

    fn find_distance(&self, other_point: &Self) -> f32 {
        ((other_point.x - self.x) * (other_point.x - self.x)
            + (other_point.y - self.y) * (other_point.y - self.y)
            + ((other_point.z - self.z) * (other_point.z - self.z)))
            .sqrt()
    }

    fn find_distance_squared(&self, other_point: &Self) -> f32 {
        (other_point.x - self.x) * (other_point.x - self.x)
            + (other_point.y - self.y) * (other_point.y - self.y)
            + ((other_point.z - self.z) * (other_point.z - self.z))
    }

    fn to_vec(&self) -> Vec<f32> {
        vec![self.x, self.y, self.z]
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
