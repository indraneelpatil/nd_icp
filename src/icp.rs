// Created by Indraneel on 12/8/24

use core::f32;

use nalgebra::{Dyn, OMatrix};

use crate::types::{Point, PointSet};

/// N dimensional Icp
///
/// Type can be a any n dimensional point
/// TODO:
/// 1. Outlier rejection of input data
/// 2. Voxel binning
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

    pub fn get_point_correspondences(&self, target_point_set: &PointSet<T>) -> Vec<&T> {
        let mut model_point_correspondences = vec![];
        for target_point in &target_point_set.points {
            let mut closest_point_idx = None;
            let mut closest_dist = f32::MAX;
            for (model_point_idx, model_point) in self.model_point_set.points.iter().enumerate() {
                let point_dist = target_point.find_distance_squared(model_point);
                if point_dist < closest_dist {
                    closest_dist = point_dist;
                    closest_point_idx = Some(model_point_idx);
                }
            }

            // Save point correspondence
            model_point_correspondences.push(
                self.model_point_set
                    .points
                    .get(closest_point_idx.expect("Failed to get closest point"))
                    .expect("Invalid closest point"),
            );
        }
        model_point_correspondences
    }

    /// ICP registrations
    ///
    /// 1. Initialise the transformation given number of dimensions
    /// 2. For each point
    ///     - Find closest point in reference cloud
    /// 3. Find transformaton and rotation which will minimise the error
    /// 4. Transform the target cloud
    /// 5. Iterate until within error threshold or max iterations
    pub fn register(&self, target_point_set: PointSet<T>) -> OMatrix<f32, Dyn, Dyn> {
        let dimension = target_point_set
            .points
            .iter()
            .next()
            .expect("Input set is empty")
            .get_dimensions();

        // Initialise transformation
        let identity_matrix: OMatrix<f32, Dyn, Dyn> =
            OMatrix::identity_generic(nalgebra::Dyn(dimension + 1), nalgebra::Dyn(dimension + 1));

        // Begin iterations
        for iteration in 0..self.max_iterations {
            // Find point correspondences
            let mut model_point_correspondences = self.get_point_correspondences(&target_point_set);

            println!("=== Finished iteration {} with cost {}", iteration, 0.0);
        }

        identity_matrix
    }
}
