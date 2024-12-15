// Created by Indraneel on 12/8/24

use nalgebra::{Dyn, Dynamic, OMatrix, U4};

use crate::types::{Point, PointSet};

/// N dimensional Icp
///
/// Type can be a any n dimensional point
pub struct Icp<T>
where
    T: Point,
{
    /// Model reference to register against
    model_point_set: PointSet<T>,
    /// Max number of iterations to run
    max_iterations: i32,
    /// Distance threshold to terminate after
    dist_delta: f32,
}

impl<T> Icp<T>
where
    T: Point,
{
    pub fn new(model_point_set: PointSet<T>, max_iterations: i32, dist_delta: f32) -> Self {
        Self {
            model_point_set,
            max_iterations,
            dist_delta,
        }
    }

    /// ICP registrations
    ///
    /// 1. Initialise the transformation given number of dimensions
    /// 2. For each point
    ///     - Find closest point in reference cloud
    /// 3. Find transformaton and rotation which will minimise the error
    /// 4. Transform the target cloud
    /// 5. Iterate until within error threshold or max iterations
    pub fn register(&self, target_point_set: PointSet<T>) -> OMatrix<f64, Dyn, Dyn> {
        let dimension = target_point_set
            .points
            .iter()
            .next()
            .expect("Input set is empty")
            .get_dimensions();

        // Initialise transformation
        let identity_matrix: OMatrix<f64, Dyn, Dyn> =
            OMatrix::identity_generic(nalgebra::Dyn(dimension + 1), nalgebra::Dyn(dimension + 1));

        identity_matrix
    }
}
