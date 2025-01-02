// Created by Indraneel on 7th Dec

use nalgebra::{Const, Dyn, Matrix, OMatrix, Storage, U1, U4};
use ply_rs::ply;

/// Generic container for holding a set of points, enforces
/// the type of points to implement the Point trait
#[derive(Clone, Debug)]
pub struct PointSet<T>
where
    T: Point,
{
    pub points: Vec<T>,
}

/// trait definition for a point, implement these functions
/// for your point struct
pub trait Point {
    /// Number of dimensions for the point
    fn get_dimensions(&self) -> usize;
    /// Takes in a transformation matrix, applies it and updates the point
    fn apply_transformation(&mut self, transformation: &OMatrix<f32, Dyn, Dyn>);
    /// Euclidean distance squared between two points
    fn find_distance_squared(&self, other_point: &Self) -> f32;
    /// Converts the point into a vector container
    fn to_vec(&self) -> Vec<f32>;
    /// Builds the point struct from a 1 dimensional matrix
    fn from_matrix<S>(matrix: &Matrix<f32, Const<1>, Dyn, S>) -> Self
    where
        S: Storage<f32, Const<1>, Dyn>;
}

/// Example 3D implementation of a point for use with ICP
#[derive(Debug, Clone, Copy)]
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
        let homogenous_coordinates: OMatrix<f32, U4, U1> =
            OMatrix::from([self.x, self.y, self.z, 1.0]);

        let transformed_coordinates = transformation * homogenous_coordinates;

        // Update point
        self.x = transformed_coordinates[(0, 0)];
        self.y = transformed_coordinates[(1, 0)];
        self.z = transformed_coordinates[(2, 0)];
    }

    fn find_distance_squared(&self, other_point: &Self) -> f32 {
        (other_point.x - self.x) * (other_point.x - self.x)
            + (other_point.y - self.y) * (other_point.y - self.y)
            + ((other_point.z - self.z) * (other_point.z - self.z))
    }

    fn to_vec(&self) -> Vec<f32> {
        vec![self.x, self.y, self.z]
    }

    fn from_matrix<S>(matrix: &Matrix<f32, Const<1>, Dyn, S>) -> Self
    where
        S: Storage<f32, Const<1>, Dyn>,
    {
        if matrix.ncols() < 3 {
            panic!("Matrix must have at least 3 columns to convert to Point3D");
        }

        Point3D {
            x: matrix[(0, 0)],
            y: matrix[(0, 1)],
            z: matrix[(0, 2)],
        }
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
